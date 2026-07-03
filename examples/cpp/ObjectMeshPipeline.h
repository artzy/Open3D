// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------
//
// Region partitioning + freeze pipeline for online SLAM examples.
// The extracted surface is partitioned into planar patches (walls, floors,
// ceilings) subdivided into fixed-size tiles, plus DBSCAN object clusters for
// the non-planar remainder. A RegionRegistry indexes every region by its TSDF
// block keys, tracks stability, and freezes regions that stay stable.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "open3d/Open3D.h"
#include "open3d/core/TensorFunction.h"
#include "open3d/t/pipelines/slam/Model.h"

namespace open3d {
namespace examples {
namespace object_mesh {

enum class ObjectType { kWall, kFloor, kCeiling, kBox, kCylinder, kGeneric };

inline const char* ObjectTypeName(ObjectType type) {
    switch (type) {
        case ObjectType::kWall:
            return "wall";
        case ObjectType::kFloor:
            return "floor";
        case ObjectType::kCeiling:
            return "ceiling";
        case ObjectType::kBox:
            return "box";
        case ObjectType::kCylinder:
            return "cylinder";
        default:
            return "generic";
    }
}

inline bool IsPlanarType(ObjectType type) {
    return type == ObjectType::kWall || type == ObjectType::kFloor ||
           type == ObjectType::kCeiling;
}

/// Region lifecycle: newly seen -> repeatedly confirmed -> frozen (blocks
/// excluded from live extraction, mesh emitted once).
enum class RegionState { kObserved, kStable, kFrozen };

inline const char* RegionStateName(RegionState state) {
    switch (state) {
        case RegionState::kObserved:
            return "observed";
        case RegionState::kStable:
            return "stable";
        default:
            return "frozen";
    }
}

struct SegmentationConfig {
    double dbscan_eps = 0.01;
    int min_cluster_points = 5000;
    int stability_frames = 5;
    bool auto_freeze = true;
    float voxel_size = 3.0f / 512.0f;
    float trunc_multiplier = 8.0f;
    float mesh_weight_threshold = 3.0f;
    // Planar region partitioning: split walls/floors into plane-local tiles
    // so long surfaces can be frozen incrementally, tile by tile.
    bool use_planar_patches = true;
    double tile_size = 1.0;    // tile edge length in meters
    int min_tile_points = 1500;  // minimum points for a tile to count
    // Growing plane surfaces: frozen planar tiles are meshed as flat
    // occupancy-grid quads (open surface, not a closed box) and merged into
    // growing PlaneSurfaces; nearby surfaces get their boundaries snapped to
    // the plane-plane intersection line to close corners incrementally.
    int min_cell_points = 10;  // absolute floor of points per occupied cell
    // A cell only renders when the observed points cover at least this
    // fraction of the expected full-cell density (extraction yields roughly
    // one point per voxel on a surface), so the plane mesh cannot extend
    // beyond areas confirmed by cloud points.
    double min_cell_coverage = 0.2;
    double surface_merge_angle_deg = 5.0;   // coplanar merge tolerance
    double surface_merge_dist = 0.02;       // coplanar |d| tolerance (m)
    double snap_angle_deg = 30.0;      // min angle between connected planes
    double snap_dist_factor = 1.5;     // snap distance = factor * cell_size
    // Architectural surface filter: only large / height-consistent patches
    // become plane surfaces; furniture-sized patches (sofa seats, table
    // tops, backrests) stay in the object path and keep their real shape.
    double min_wall_height = 0.8;      // min vertical extent of a wall (m)
    double min_patch_area_m2 = 1.5;    // alt. acceptance by patch area
    double floor_band = 0.15;          // height band around floor/ceiling (m)
};

/// One partitioned region candidate produced from a single extracted surface:
/// either a planar tile (wall/floor/ceiling) or a DBSCAN object cluster.
struct RegionCandidate {
    ObjectType type = ObjectType::kGeneric;
    Eigen::Vector4d plane = Eigen::Vector4d::Zero();  // (n, d), planar only
    int tile_i = 0;  // plane-local tile index, planar only
    int tile_j = 0;
    double area_m2 = 0.0;
    t::geometry::PointCloud points;  // CPU
};

struct FrozenObjectCandidate {
    int id = -1;
    ObjectType type = ObjectType::kGeneric;
    t::geometry::TriangleMesh mesh;
    core::Tensor block_keys;
    geometry::AxisAlignedBoundingBox bounds;
    Eigen::Vector4d plane = Eigen::Vector4d::Zero();
    int tile_i = 0;
    int tile_j = 0;
    double area_m2 = 0.0;
    // Growing plane surface fields (planar types only). A candidate with the
    // id of an earlier one is a grown replacement, not a new object.
    double cell_size = 0.0;
    int cell_count = 0;
    double closure = 0.0;
    std::vector<int> connected;
};

// ---------------------------------------------------------------------------
// Region registry: spatial index of regions keyed by TSDF block coordinates.
// ---------------------------------------------------------------------------

struct BlockKey {
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;
    bool operator==(const BlockKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct BlockKeyHash {
    std::size_t operator()(const BlockKey& key) const {
        // Same spatial hash primes as the TSDF voxel block hash map.
        return static_cast<std::size_t>((key.x * 73856093) ^
                                        (key.y * 19349669) ^
                                        (key.z * 83492791));
    }
};

struct Region {
    int id = -1;
    ObjectType type = ObjectType::kGeneric;
    RegionState state = RegionState::kObserved;
    Eigen::Vector4d plane = Eigen::Vector4d::Zero();
    int tile_i = 0;
    int tile_j = 0;
    int stable_frames = 0;
    double area_m2 = 0.0;
    core::Tensor block_keys;  // (N, 3) Int32 CPU
};

/// Thread-safe registry of scene regions. The TSDF volume already partitions
/// space into 16^3-voxel blocks, so a block-key -> region-id hash map serves
/// as the spatial index: O(1) membership lookup, deterministic re-matching of
/// candidates across frames, and duplicate-freeze prevention.
class RegionRegistry {
public:
    /// Match a batch of candidates against known regions (majority vote of
    /// their block keys), update stability counters, register new regions and
    /// prune unfrozen regions that were not re-observed.
    /// Frozen-overlap and matching votes use "core" keys (blocks that
    /// actually contain candidate points, no truncation inflation): a frozen
    /// neighbor's inflated block halo must not permanently poison island
    /// candidates surrounded by frozen surfaces. Keys owned by frozen
    /// regions are stripped from \p block_keys (shrunk in place) so the
    /// candidate proceeds with the remainder instead of being dropped.
    /// \returns per-candidate region id (-1 if the candidate was dropped
    /// because it lies mostly inside an already-frozen region), and fills
    /// \p ready_to_freeze with region ids whose stability just reached
    /// \p config.stability_frames.
    std::vector<int> UpdateBatch(const std::vector<RegionCandidate>& candidates,
                                 std::vector<core::Tensor>& block_keys,
                                 const SegmentationConfig& config,
                                 int& next_object_id,
                                 std::vector<int>& ready_to_freeze) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<int> candidate_region_ids(candidates.size(), -1);
        ready_to_freeze.clear();

        const float block_size = config.voxel_size * 16.0f;
        std::unordered_map<int, bool> seen_this_pass;
        for (size_t c = 0; c < candidates.size(); ++c) {
            const std::vector<BlockKey> keys = ToKeyVector(block_keys[c]);
            if (keys.empty()) {
                continue;
            }
            const std::vector<BlockKey> core_keys =
                    CoreKeysFromPoints(candidates[c].points, block_size);
            const std::vector<BlockKey>& vote_keys =
                    core_keys.empty() ? keys : core_keys;

            // Majority vote: which region do this candidate's blocks map to?
            std::map<int, int> votes;
            int frozen_hits = 0;
            for (const BlockKey& key : vote_keys) {
                auto it = block_to_region_.find(key);
                if (it == block_to_region_.end()) {
                    continue;
                }
                ++votes[it->second];
                if (RegionById(it->second)->state == RegionState::kFrozen) {
                    ++frozen_hits;
                }
            }

            // Mostly inside an already-frozen region: nothing new to manage.
            if (frozen_hits * 10 >= static_cast<int>(vote_keys.size()) * 6) {
                utility::LogDebug(
                        "Region candidate dropped ({}, {} core blocks, "
                        "{:.0f}% frozen overlap).",
                        ObjectTypeName(candidates[c].type), vote_keys.size(),
                        100.0 * frozen_hits /
                                static_cast<double>(vote_keys.size()));
                continue;
            }

            // Proceed with the blocks not owned by frozen regions; ownership
            // of shared border blocks stays with whoever froze first.
            std::vector<BlockKey> kept;
            kept.reserve(keys.size());
            for (const BlockKey& key : keys) {
                auto it = block_to_region_.find(key);
                if (it != block_to_region_.end() &&
                    RegionById(it->second)->state == RegionState::kFrozen) {
                    continue;
                }
                kept.push_back(key);
            }
            if (kept.empty()) {
                continue;
            }
            if (kept.size() < keys.size()) {
                block_keys[c] = KeysToTensor(kept);
            }

            int best_id = -1;
            int best_votes = 0;
            for (const auto& [region_id, count] : votes) {
                if (RegionById(region_id)->state != RegionState::kFrozen &&
                    count > best_votes) {
                    best_id = region_id;
                    best_votes = count;
                }
            }

            Region* region = nullptr;
            if (best_id >= 0 &&
                best_votes * 2 >= static_cast<int>(vote_keys.size())) {
                // Same region as a previous pass: refresh and grow.
                region = RegionById(best_id);
                ++region->stable_frames;
            } else {
                // New region.
                regions_.push_back({});
                region = &regions_.back();
                region->id = next_object_id++;
                region->stable_frames = 1;
            }
            region->type = candidates[c].type;
            region->plane = candidates[c].plane;
            region->tile_i = candidates[c].tile_i;
            region->tile_j = candidates[c].tile_j;
            region->area_m2 = candidates[c].area_m2;
            region->block_keys = block_keys[c];
            for (const BlockKey& key : kept) {
                // First region to claim a block keeps it (stable ownership at
                // tile borders).
                block_to_region_.emplace(key, region->id);
            }
            seen_this_pass[region->id] = true;
            candidate_region_ids[c] = region->id;

            if (region->stable_frames >= config.stability_frames &&
                region->state == RegionState::kObserved) {
                region->state = RegionState::kStable;
                if (config.auto_freeze) {
                    ready_to_freeze.push_back(region->id);
                }
            }
        }

        // Unfrozen regions that were not re-observed lost tracking: drop them
        // and release their block mappings (mirrors the old tracker reset).
        std::vector<int> dropped;
        for (const Region& region : regions_) {
            if (region.state != RegionState::kFrozen &&
                !seen_this_pass.count(region.id)) {
                dropped.push_back(region.id);
            }
        }
        for (int region_id : dropped) {
            EraseRegionLocked(region_id);
        }
        return candidate_region_ids;
    }

    void MarkFrozen(int region_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        Region* region = RegionById(region_id);
        if (region) {
            region->state = RegionState::kFrozen;
        }
    }

    std::vector<Region> Snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return regions_;
    }

    int FrozenCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        int count = 0;
        for (const auto& region : regions_) {
            if (region.state == RegionState::kFrozen) {
                ++count;
            }
        }
        return count;
    }

    std::unordered_map<ObjectType, int> FrozenTypeCounts() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::unordered_map<ObjectType, int> counts;
        for (const auto& region : regions_) {
            if (region.state == RegionState::kFrozen) {
                ++counts[region.type];
            }
        }
        return counts;
    }

    /// Multi-line human readable region list for the GUI info panel.
    std::string FormatSummary(size_t max_lines) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (regions_.empty()) {
            return {};
        }
        std::ostringstream out;
        out << "Regions (" << regions_.size() << "):\n";
        size_t lines = 0;
        // Frozen regions first (most informative), then in-progress ones.
        for (int pass = 0; pass < 2 && lines < max_lines; ++pass) {
            for (const auto& region : regions_) {
                const bool frozen = region.state == RegionState::kFrozen;
                if ((pass == 0) != frozen || lines >= max_lines) {
                    continue;
                }
                out << fmt::format("  #{:<3} {:<8} {:<8}", region.id,
                                   ObjectTypeName(region.type),
                                   RegionStateName(region.state));
                if (IsPlanarType(region.type)) {
                    out << fmt::format(" tile({},{})", region.tile_i,
                                       region.tile_j);
                }
                out << fmt::format(" {:.1f} m2\n", region.area_m2);
                ++lines;
            }
        }
        if (regions_.size() > lines) {
            out << fmt::format("  ... {} more\n", regions_.size() - lines);
        }
        return out.str();
    }

private:
    Region* RegionById(int region_id) {
        for (auto& region : regions_) {
            if (region.id == region_id) {
                return &region;
            }
        }
        return nullptr;
    }
    const Region* RegionById(int region_id) const {
        return const_cast<RegionRegistry*>(this)->RegionById(region_id);
    }

    void EraseRegionLocked(int region_id) {
        for (auto it = block_to_region_.begin();
             it != block_to_region_.end();) {
            if (it->second == region_id) {
                it = block_to_region_.erase(it);
            } else {
                ++it;
            }
        }
        regions_.erase(std::remove_if(regions_.begin(), regions_.end(),
                                      [region_id](const Region& region) {
                                          return region.id == region_id;
                                      }),
                       regions_.end());
    }

    /// Blocks that actually contain candidate points (no truncation
    /// inflation), used for frozen-overlap and matching votes.
    static std::vector<BlockKey> CoreKeysFromPoints(
            const t::geometry::PointCloud& points, float block_size) {
        std::vector<BlockKey> keys;
        if (!points.HasPointPositions() || block_size <= 0.0f) {
            return keys;
        }
        core::Tensor pos = points.GetPointPositions()
                                   .To(core::Device("CPU:0"), core::Float32)
                                   .Contiguous();
        const float* data = pos.GetDataPtr<float>();
        const int64_t n = pos.GetLength();
        std::unordered_set<BlockKey, BlockKeyHash> unique;
        for (int64_t i = 0; i < n; ++i) {
            unique.insert({static_cast<int32_t>(
                                   std::floor(data[i * 3 + 0] / block_size)),
                           static_cast<int32_t>(
                                   std::floor(data[i * 3 + 1] / block_size)),
                           static_cast<int32_t>(
                                   std::floor(data[i * 3 + 2] / block_size))});
        }
        keys.assign(unique.begin(), unique.end());
        return keys;
    }

    static core::Tensor KeysToTensor(const std::vector<BlockKey>& keys) {
        core::Tensor tensor({static_cast<int64_t>(keys.size()), 3},
                            core::Int32, core::Device("CPU:0"));
        int32_t* data = tensor.GetDataPtr<int32_t>();
        for (size_t i = 0; i < keys.size(); ++i) {
            data[i * 3 + 0] = keys[i].x;
            data[i * 3 + 1] = keys[i].y;
            data[i * 3 + 2] = keys[i].z;
        }
        return tensor;
    }

    static std::vector<BlockKey> ToKeyVector(const core::Tensor& block_keys) {
        std::vector<BlockKey> keys;
        if (block_keys.NumElements() == 0) {
            return keys;
        }
        core::Tensor cpu =
                block_keys.To(core::Device("CPU:0")).Contiguous();
        const int32_t* data = cpu.GetDataPtr<int32_t>();
        const int64_t n = cpu.GetLength();
        keys.reserve(static_cast<size_t>(n));
        for (int64_t i = 0; i < n; ++i) {
            keys.push_back({data[i * 3 + 0], data[i * 3 + 1],
                            data[i * 3 + 2]});
        }
        return keys;
    }

    mutable std::mutex mutex_;
    std::vector<Region> regions_;
    std::unordered_map<BlockKey, int, BlockKeyHash> block_to_region_;
};

// ---------------------------------------------------------------------------
// Growing plane surfaces: freeze planes as open quad meshes first, merge
// coplanar tiles into one surface, and snap boundaries of neighboring
// surfaces onto their plane-plane intersection line so corners close
// incrementally (instead of freezing closed boxes from the start).
// ---------------------------------------------------------------------------

/// Measured surface area estimate: TSDF extraction yields roughly one point
/// per voxel on a surface, so area ~= point_count * voxel_size^2. More
/// accurate than the OBB rectangle, which overestimates sparse patches.
inline double EstimateSurfaceArea(size_t point_count, float voxel_size) {
    return static_cast<double>(point_count) * voxel_size * voxel_size;
}

/// Canonical in-plane axes derived only from the normal and anchored to the
/// world axes, so cell coordinates are reproducible across frames.
inline void CanonicalPlaneAxes(const Eigen::Vector3d& normal,
                               Eigen::Vector3d& u,
                               Eigen::Vector3d& v) {
    int ref_axis = 0;
    for (int k = 1; k < 3; ++k) {
        if (std::abs(normal(k)) < std::abs(normal(ref_axis))) {
            ref_axis = k;
        }
    }
    Eigen::Vector3d ref = Eigen::Vector3d::Zero();
    ref(ref_axis) = 1.0;
    u = (ref - normal * normal.dot(ref)).normalized();
    v = normal.cross(u);
}

class PlaneSurfaceAtlas {
public:
    struct SurfaceInfo {
        int id = -1;
        ObjectType type = ObjectType::kWall;
        Eigen::Vector4d plane = Eigen::Vector4d::Zero();
        double cell_size = 0.25;
        int cell_count = 0;
        double area_m2 = 0.0;
        double closure = 0.0;  // snapped boundary corner ratio [0, 1]
        std::vector<int> connected;
        core::Tensor block_keys;  // union of frozen tile keys (CPU Int32)
    };

    /// Accumulate a frozen planar tile into a matching surface (coplanar
    /// within merge tolerances) or create a new surface. Marks the surface
    /// dirty for the next RebuildDirtyMeshes pass. Returns the surface id.
    int AddTile(ObjectType type,
                const Eigen::Vector4d& plane_in,
                const geometry::PointCloud& points,
                const core::Tensor& tile_block_keys,
                const SegmentationConfig& config,
                int& next_object_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        Eigen::Vector4d plane = plane_in;
        if (plane.head<3>().norm() < 1e-9) {
            return -1;
        }
        plane /= plane.head<3>().norm();

        Surface* surface = nullptr;
        const double cos_tol =
                std::cos(config.surface_merge_angle_deg * kDegToRad);
        for (auto& existing : surfaces_) {
            if (existing.type != type) {
                continue;
            }
            Eigen::Vector4d aligned = plane;
            if (existing.plane.head<3>().dot(aligned.head<3>()) < 0.0) {
                aligned = -aligned;
            }
            if (existing.plane.head<3>().dot(aligned.head<3>()) >= cos_tol &&
                std::abs(existing.plane(3) - aligned(3)) <=
                        config.surface_merge_dist) {
                surface = &existing;
                plane = aligned;
                break;
            }
        }
        if (surface == nullptr) {
            surfaces_.push_back({});
            surface = &surfaces_.back();
            surface->id = next_object_id++;
            surface->type = type;
            surface->plane = plane;
            surface->cell_size = std::max(0.05, config.tile_size / 4.0);
            // Axes are fixed at creation so cell indices never shift when the
            // plane estimate is later refined.
            CanonicalPlaneAxes(plane.head<3>(), surface->u, surface->v);
        }

        // Refine the plane as a weighted average to suppress drift.
        const double sample_weight =
                static_cast<double>(points.points_.size());
        Eigen::Vector4d refined =
                (surface->plane * surface->weight + plane * sample_weight) /
                (surface->weight + sample_weight);
        const double norm = refined.head<3>().norm();
        if (norm > 1e-9) {
            surface->plane = refined / norm;
        }
        surface->weight += sample_weight;

        // Accumulate occupancy cells with average colors. Points farther
        // from the plane than the merge tolerance belong to other geometry
        // (patch OBB membership is approximate) and must not occupy cells.
        const bool has_colors =
                points.colors_.size() == points.points_.size();
        const Eigen::Vector3d n = surface->plane.head<3>();
        const double max_plane_dist = 1.5 * config.surface_merge_dist;
        for (size_t k = 0; k < points.points_.size(); ++k) {
            const Eigen::Vector3d& p = points.points_[k];
            if (std::abs(n.dot(p) + surface->plane(3)) > max_plane_dist) {
                continue;
            }
            const double a = surface->u.dot(p);
            const double b = surface->v.dot(p);
            const int ci = static_cast<int>(
                    std::floor(a / surface->cell_size));
            const int cj = static_cast<int>(
                    std::floor(b / surface->cell_size));
            Cell& cell = surface->cells[{ci, cj}];
            ++cell.points;
            cell.color_sum += has_colors ? points.colors_[k]
                                         : Eigen::Vector3d(0.7, 0.7, 0.7);
            cell.min_u = std::min(cell.min_u, a);
            cell.max_u = std::max(cell.max_u, a);
            cell.min_v = std::min(cell.min_v, b);
            cell.max_v = std::max(cell.max_v, b);
        }
        if (tile_block_keys.NumElements() > 0) {
            if (surface->block_keys.NumElements() == 0) {
                surface->block_keys = tile_block_keys;
            } else {
                surface->block_keys = core::Concatenate(
                        {surface->block_keys, tile_block_keys}, 0);
            }
        }
        dirty_.insert(surface->id);
        return surface->id;
    }

    /// Rebuild the quad meshes of surfaces that gained cells (plus their
    /// connected neighbors) and snap boundary corners to intersection lines.
    /// Returns the ids of surfaces whose mesh changed.
    std::vector<int> RebuildDirtyMeshes(const SegmentationConfig& config) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (dirty_.empty()) {
            return {};
        }
        // Growing one surface can create a new corner with a neighbor, so
        // rebuild neighbors of dirty surfaces as well.
        std::set<int> rebuild = dirty_;
        for (int id : dirty_) {
            Surface* surface = FindSurface(id);
            if (surface == nullptr) {
                continue;
            }
            for (auto& other : surfaces_) {
                if (other.id != id && ArePotentialNeighbors(*surface, other,
                                                            config)) {
                    rebuild.insert(other.id);
                }
            }
        }
        dirty_.clear();

        std::vector<int> updated;
        for (int id : rebuild) {
            Surface* surface = FindSurface(id);
            if (surface == nullptr) {
                continue;
            }
            BuildSurfaceMesh(*surface, config);
            SnapToNeighbors(*surface, config);
            surface->mesh.ComputeVertexNormals();
            updated.push_back(id);
        }
        return updated;
    }

    /// Copy of the last built mesh (legacy, CPU) for a surface.
    geometry::TriangleMesh GetMesh(int id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const Surface* surface = FindSurface(id);
        return surface ? surface->mesh : geometry::TriangleMesh();
    }

    SurfaceInfo GetInfo(int id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const Surface* surface = FindSurface(id);
        return surface ? MakeInfo(*surface) : SurfaceInfo();
    }

    std::vector<SurfaceInfo> Snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<SurfaceInfo> infos;
        infos.reserve(surfaces_.size());
        for (const auto& surface : surfaces_) {
            infos.push_back(MakeInfo(surface));
        }
        return infos;
    }

    /// Multi-line human readable surface list for the GUI info panel.
    std::string FormatSummary(size_t max_lines) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (surfaces_.empty()) {
            return {};
        }
        std::ostringstream out;
        out << "Surfaces (" << surfaces_.size() << "):\n";
        size_t lines = 0;
        for (const auto& surface : surfaces_) {
            if (lines >= max_lines) {
                out << fmt::format("  ... {} more\n",
                                   surfaces_.size() - lines);
                break;
            }
            out << fmt::format(
                    "  #{:<3} {:<8} {:>5.1f} m2  closure {:>3.0f}%\n",
                    surface.id, ObjectTypeName(surface.type),
                    static_cast<double>(surface.occupied_cells) *
                            surface.cell_size * surface.cell_size,
                    surface.closure * 100.0);
            ++lines;
        }
        return out.str();
    }

private:
    static constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

    struct Cell {
        Eigen::Vector3d color_sum = Eigen::Vector3d::Zero();
        int points = 0;
        // Observed point extent in plane coordinates: boundary quads are
        // clipped to this so the mesh never extends past confirmed points.
        double min_u = std::numeric_limits<double>::max();
        double max_u = std::numeric_limits<double>::lowest();
        double min_v = std::numeric_limits<double>::max();
        double max_v = std::numeric_limits<double>::lowest();
    };

    struct Surface {
        int id = -1;
        ObjectType type = ObjectType::kWall;
        Eigen::Vector4d plane = Eigen::Vector4d::Zero();  // unit n, d
        Eigen::Vector3d u = Eigen::Vector3d::UnitX();
        Eigen::Vector3d v = Eigen::Vector3d::UnitY();
        double cell_size = 0.25;
        double weight = 0.0;  // accumulated samples for plane averaging
        int occupied_cells = 0;  // cells that passed the coverage threshold
        // Threshold of the last mesh build; used by neighbors to test
        // whether a snap target lies next to actually observed cells.
        int occupy_threshold = std::numeric_limits<int>::max();
        std::map<std::pair<int, int>, Cell> cells;
        core::Tensor block_keys;  // union of frozen tile keys (CPU Int32)
        std::vector<int> connected;
        double closure = 0.0;
        geometry::TriangleMesh mesh;
        Eigen::Vector3d aabb_min = Eigen::Vector3d::Zero();
        Eigen::Vector3d aabb_max = Eigen::Vector3d::Zero();
        // Lattice corner -> mesh vertex index of the last build; boundary
        // corner flags for snapping.
        std::vector<int> boundary_vertices;
    };

    Surface* FindSurface(int id) {
        for (auto& surface : surfaces_) {
            if (surface.id == id) {
                return &surface;
            }
        }
        return nullptr;
    }
    const Surface* FindSurface(int id) const {
        return const_cast<PlaneSurfaceAtlas*>(this)->FindSurface(id);
    }

    SurfaceInfo MakeInfo(const Surface& surface) const {
        SurfaceInfo info;
        info.id = surface.id;
        info.type = surface.type;
        info.plane = surface.plane;
        info.cell_size = surface.cell_size;
        info.cell_count = surface.occupied_cells;
        info.area_m2 = static_cast<double>(surface.occupied_cells) *
                       surface.cell_size * surface.cell_size;
        info.closure = surface.closure;
        info.connected = surface.connected;
        info.block_keys = surface.block_keys;
        return info;
    }

    /// World position of the lattice corner (a, b) projected onto the
    /// (refined) plane.
    static Eigen::Vector3d CornerPosition(const Surface& surface,
                                          double a,
                                          double b) {
        const Eigen::Vector3d n = surface.plane.head<3>();
        Eigen::Vector3d p = a * surface.u + b * surface.v;
        p += (-(n.dot(p) + surface.plane(3))) * n;
        return p;
    }

    /// Flat open quad mesh from occupied cells; shared lattice vertices, one
    /// quad (two triangles) per cell. Also records boundary vertices (corners
    /// of cell edges not shared by two occupied cells) for corner snapping.
    void BuildSurfaceMesh(Surface& surface,
                          const SegmentationConfig& config) {
        surface.mesh.Clear();
        surface.boundary_vertices.clear();
        surface.occupied_cells = 0;

        // Expected points of a fully observed cell: the TSDF extraction
        // yields roughly one point per voxel on a surface, so a cell must be
        // covered by confirmed cloud points to render (no extrapolation).
        const double voxels_per_cell =
                (surface.cell_size / std::max(1e-4f, config.voxel_size)) *
                (surface.cell_size / std::max(1e-4f, config.voxel_size));
        const int occupy_threshold = std::max(
                config.min_cell_points,
                static_cast<int>(config.min_cell_coverage * voxels_per_cell));
        surface.occupy_threshold = occupy_threshold;

        // Pass 1: collect the cells that pass the coverage threshold.
        std::map<std::pair<int, int>, const Cell*> occupied;
        for (const auto& [cell_ij, cell] : surface.cells) {
            if (cell.points >= occupy_threshold) {
                occupied.emplace(cell_ij, &cell);
            }
        }
        surface.occupied_cells = static_cast<int>(occupied.size());

        std::map<std::pair<int, int>, int> corner_to_vertex;
        std::map<std::pair<int, int>, Eigen::Vector3d> corner_color_sum;
        std::map<std::pair<int, int>, int> corner_color_count;
        // Lattice edge occupancy: key = (i, j, 0 horizontal / 1 vertical).
        std::map<std::tuple<int, int, int>, int> edge_count;

        // Lattice corner clamped into the observed point extent of its
        // adjacent occupied cells: interior corners stay on the lattice
        // (fully covered cells), boundary corners are pulled inward so the
        // mesh ends where the confirmed cloud points end.
        auto corner_vertex = [&](int i, int j) -> int {
            auto it = corner_to_vertex.find({i, j});
            if (it != corner_to_vertex.end()) {
                return it->second;
            }
            double a = i * surface.cell_size;
            double b = j * surface.cell_size;
            double u_lo = std::numeric_limits<double>::max();
            double u_hi = std::numeric_limits<double>::lowest();
            double v_lo = u_lo, v_hi = u_hi;
            for (int di = -1; di <= 0; ++di) {
                for (int dj = -1; dj <= 0; ++dj) {
                    auto itc = occupied.find({i + di, j + dj});
                    if (itc == occupied.end()) {
                        continue;
                    }
                    u_lo = std::min(u_lo, itc->second->min_u);
                    u_hi = std::max(u_hi, itc->second->max_u);
                    v_lo = std::min(v_lo, itc->second->min_v);
                    v_hi = std::max(v_hi, itc->second->max_v);
                }
            }
            if (u_lo <= u_hi) {
                a = std::min(std::max(a, u_lo), u_hi);
                b = std::min(std::max(b, v_lo), v_hi);
            }
            const int index =
                    static_cast<int>(surface.mesh.vertices_.size());
            surface.mesh.vertices_.push_back(CornerPosition(surface, a, b));
            corner_to_vertex[{i, j}] = index;
            return index;
        };

        for (const auto& [cell_ij, cell_ptr] : occupied) {
            const Cell& cell = *cell_ptr;
            const int i = cell_ij.first;
            const int j = cell_ij.second;
            const int v00 = corner_vertex(i, j);
            const int v10 = corner_vertex(i + 1, j);
            const int v11 = corner_vertex(i + 1, j + 1);
            const int v01 = corner_vertex(i, j + 1);
            surface.mesh.triangles_.push_back({v00, v10, v11});
            surface.mesh.triangles_.push_back({v00, v11, v01});

            const Eigen::Vector3d color =
                    cell.color_sum / std::max(1, cell.points);
            const std::array<std::pair<int, int>, 4> corners = {
                    std::pair<int, int>{i, j}, {i + 1, j}, {i + 1, j + 1},
                    {i, j + 1}};
            for (const auto& corner : corners) {
                corner_color_sum[corner] += color;
                ++corner_color_count[corner];
            }
            ++edge_count[{i, j, 0}];
            ++edge_count[{i, j + 1, 0}];
            ++edge_count[{i, j, 1}];
            ++edge_count[{i + 1, j, 1}];
        }

        surface.mesh.vertex_colors_.resize(surface.mesh.vertices_.size(),
                                           Eigen::Vector3d(0.7, 0.7, 0.7));
        for (const auto& [corner, index] : corner_to_vertex) {
            const int count = corner_color_count[corner];
            if (count > 0) {
                surface.mesh.vertex_colors_[index] =
                        (corner_color_sum[corner] / count)
                                .cwiseMax(Eigen::Vector3d::Zero())
                                .cwiseMin(Eigen::Vector3d::Ones());
            }
        }

        // Boundary corners: endpoints of lattice edges used by one cell only.
        std::set<int> boundary;
        for (const auto& [edge, count] : edge_count) {
            if (count != 1) {
                continue;
            }
            const int i = std::get<0>(edge);
            const int j = std::get<1>(edge);
            const bool horizontal = std::get<2>(edge) == 0;
            boundary.insert(corner_to_vertex[{i, j}]);
            boundary.insert(corner_to_vertex[horizontal
                                                     ? std::pair<int, int>{
                                                               i + 1, j}
                                                     : std::pair<int, int>{
                                                               i, j + 1}]);
        }
        surface.boundary_vertices.assign(boundary.begin(), boundary.end());

        // AABB for neighbor tests.
        surface.aabb_min = Eigen::Vector3d::Constant(
                std::numeric_limits<double>::max());
        surface.aabb_max = Eigen::Vector3d::Constant(
                std::numeric_limits<double>::lowest());
        for (const auto& p : surface.mesh.vertices_) {
            surface.aabb_min = surface.aabb_min.cwiseMin(p);
            surface.aabb_max = surface.aabb_max.cwiseMax(p);
        }
    }

    bool ArePotentialNeighbors(const Surface& a,
                               const Surface& b,
                               const SegmentationConfig& config) const {
        if (a.mesh.vertices_.empty() || b.mesh.vertices_.empty()) {
            return false;
        }
        const double cos_snap = std::cos(config.snap_angle_deg * kDegToRad);
        if (std::abs(a.plane.head<3>().dot(b.plane.head<3>())) >= cos_snap) {
            return false;  // nearly coplanar/parallel: no corner to close
        }
        const double margin = config.snap_dist_factor *
                              std::max(a.cell_size, b.cell_size);
        for (int k = 0; k < 3; ++k) {
            if (a.aabb_min(k) > b.aabb_max(k) + margin ||
                b.aabb_min(k) > a.aabb_max(k) + margin) {
                return false;
            }
        }
        return true;
    }

    /// Move boundary corners of \p surface that lie close to the
    /// intersection line with a neighboring plane onto that line, sealing
    /// the corner gap. Updates the closure ratio and connectivity list.
    void SnapToNeighbors(Surface& surface, const SegmentationConfig& config) {
        surface.connected.clear();
        if (surface.boundary_vertices.empty()) {
            surface.closure = 0.0;
            return;
        }
        const double snap_dist =
                config.snap_dist_factor * surface.cell_size;
        std::set<int> snapped;

        for (const auto& other : surfaces_) {
            if (other.id == surface.id ||
                !ArePotentialNeighbors(surface, other, config)) {
                continue;
            }
            // Intersection line of the two planes: solve for a point with
            // [n1; n2; dir]^T p = [-d1; -d2; 0], direction = n1 x n2.
            const Eigen::Vector3d n1 = surface.plane.head<3>();
            const Eigen::Vector3d n2 = other.plane.head<3>();
            Eigen::Vector3d dir = n1.cross(n2);
            if (dir.norm() < 1e-9) {
                continue;
            }
            dir.normalize();
            Eigen::Matrix3d A;
            A.row(0) = n1;
            A.row(1) = n2;
            A.row(2) = dir;
            const Eigen::Vector3d rhs(-surface.plane(3), -other.plane(3),
                                      0.0);
            const Eigen::Vector3d p0 = A.fullPivLu().solve(rhs);

            // Only snap where the neighbor actually has observed cells: the
            // intersection line is infinite, and pulling edges toward parts
            // of it without confirmed points would enlarge the surface.
            auto near_other_cells = [&](const Eigen::Vector3d& p) {
                const double oa = other.u.dot(p);
                const double ob = other.v.dot(p);
                const int oi = static_cast<int>(
                        std::floor(oa / other.cell_size));
                const int oj = static_cast<int>(
                        std::floor(ob / other.cell_size));
                for (int di = -1; di <= 1; ++di) {
                    for (int dj = -1; dj <= 1; ++dj) {
                        auto itc = other.cells.find({oi + di, oj + dj});
                        if (itc != other.cells.end() &&
                            itc->second.points >= other.occupy_threshold) {
                            return true;
                        }
                    }
                }
                return false;
            };

            bool any_snapped = false;
            for (int index : surface.boundary_vertices) {
                Eigen::Vector3d& p = surface.mesh.vertices_[index];
                const Eigen::Vector3d rel = p - p0;
                const Eigen::Vector3d on_line = p0 + rel.dot(dir) * dir;
                if ((p - on_line).norm() <= snap_dist &&
                    near_other_cells(on_line)) {
                    p = on_line;
                    snapped.insert(index);
                    any_snapped = true;
                }
            }
            if (any_snapped) {
                surface.connected.push_back(other.id);
            }
        }
        surface.closure = static_cast<double>(snapped.size()) /
                          static_cast<double>(surface.boundary_vertices.size());
    }

    mutable std::mutex mutex_;
    std::vector<Surface> surfaces_;
    std::set<int> dirty_;
};

// ---------------------------------------------------------------------------
// Geometry helpers (unchanged behavior).
// ---------------------------------------------------------------------------

inline core::Tensor CollectBlockKeys(
        t::geometry::VoxelBlockGrid& vbg,
        const t::geometry::PointCloud& cluster,
        float trunc_multiplier,
        float voxel_size) {
    const core::Tensor empty({}, core::Int32, core::Device("CPU:0"));
    if (!cluster.HasPointPositions()) {
        return empty;
    }
    // Downsample to half a block (16 voxels) so the shared frustum hash map,
    // sized for depth-image touches, cannot overflow; move to the volume
    // device (the touch kernel runs there and must not read host memory).
    t::geometry::PointCloud touch_pcd(cluster.GetPointPositions());
    touch_pcd = touch_pcd.VoxelDownSample(voxel_size * 8.0);
    touch_pcd = touch_pcd.To(vbg.GetHashMap().GetDevice());
    if (!touch_pcd.HasPointPositions()) {
        return empty;
    }
    core::Tensor block_coords;
    try {
        block_coords =
                vbg.GetUniqueBlockCoordinates(touch_pcd, trunc_multiplier);
    } catch (const std::exception& e) {
        utility::LogWarning("CollectBlockKeys failed: {}", e.what());
        return empty;
    }
    if (block_coords.GetLength() == 0) {
        return empty;
    }
    return block_coords.To(core::Device("CPU:0")).Contiguous();
}

inline ObjectType ClassifyCluster(const t::geometry::PointCloud& cluster,
                                  double plane_inlier_ratio_threshold = 0.85,
                                  double wall_thin_ratio = 0.15,
                                  double cylinder_circle_rmse = 0.03) {
    geometry::PointCloud legacy = cluster.ToLegacy();
    if (legacy.points_.empty()) {
        return ObjectType::kGeneric;
    }

    auto obb = legacy.GetOrientedBoundingBox();
    Eigen::Vector3d extent = obb.extent_;
    std::sort(extent.data(), extent.data() + 3);
    const double min_extent = extent[0];
    const double mid_extent = extent[1];
    const double max_extent = extent[2];

    const double plane_distance = std::max(0.01, min_extent * 0.5);
    auto [plane, inliers] =
            legacy.SegmentPlane(plane_distance, 3, 1000, 0.999);
    const double inlier_ratio =
            static_cast<double>(inliers.size()) /
            static_cast<double>(legacy.points_.size());
    if (inlier_ratio >= plane_inlier_ratio_threshold &&
        min_extent / std::max(mid_extent, 1e-6) < wall_thin_ratio) {
        return ObjectType::kWall;
    }

    if (min_extent / std::max(max_extent, 1e-6) > 0.08 &&
        mid_extent / std::max(max_extent, 1e-6) > 0.08 &&
        inlier_ratio < plane_inlier_ratio_threshold) {
        return ObjectType::kBox;
    }

    if (max_extent > 1e-6 &&
        std::abs(min_extent - mid_extent) / max_extent < 0.25 &&
        min_extent / max_extent < 0.35) {
        const Eigen::Matrix3d rotation = obb.R_;
        Eigen::Vector3d axis = rotation.col(2);
        axis.normalize();
        const Eigen::Vector3d center = obb.center_;
        std::vector<double> radii;
        radii.reserve(legacy.points_.size());
        for (const auto& point : legacy.points_) {
            const Eigen::Vector3d rel = point - center;
            const Eigen::Vector3d radial = rel - axis.dot(rel) * axis;
            radii.push_back(radial.norm());
        }
        if (!radii.empty()) {
            const double mean_radius =
                    std::accumulate(radii.begin(), radii.end(), 0.0) /
                    static_cast<double>(radii.size());
            double rmse = 0.0;
            for (double radius : radii) {
                const double diff = radius - mean_radius;
                rmse += diff * diff;
            }
            rmse = std::sqrt(rmse / static_cast<double>(radii.size()));
            if (rmse < cylinder_circle_rmse) {
                return ObjectType::kCylinder;
            }
        }
    }

    return ObjectType::kGeneric;
}

inline t::geometry::TriangleMesh CreateBoxMeshFromObb(
        const geometry::OrientedBoundingBox& obb,
        const core::Device& device) {
    Eigen::Vector3d extent = obb.extent_;
    auto mesh = t::geometry::TriangleMesh::CreateBox(
            extent[0], extent[1], extent[2], core::Float32, core::Int64,
            device);
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = obb.R_;
    transform.block<3, 1>(0, 3) = obb.center_ - obb.R_ * extent / 2.0;
    mesh.Transform(core::eigen_converter::EigenMatrixToTensor(transform));
    return mesh;
}

inline t::geometry::TriangleMesh CreateWallMesh(
        const t::geometry::PointCloud& cluster, const core::Device& device) {
    geometry::PointCloud legacy = cluster.ToLegacy();
    auto obb = legacy.GetOrientedBoundingBox();
    Eigen::Vector3d extent = obb.extent_;
    const int thin_axis =
            static_cast<int>(std::distance(
                    extent.data(),
                    std::min_element(extent.data(), extent.data() + 3)));
    extent[thin_axis] = std::max(extent[thin_axis], 0.01);
    obb.extent_ = extent;
    return CreateBoxMeshFromObb(obb, device);
}

inline t::geometry::TriangleMesh CreateCylinderMeshFromObb(
        const geometry::OrientedBoundingBox& obb,
        const core::Device& device) {
    Eigen::Vector3d extent = obb.extent_;
    std::sort(extent.data(), extent.data() + 3);
    const double radius = 0.5 * (extent[0] + extent[1]);
    const double height = extent[2];
    auto mesh = t::geometry::TriangleMesh::CreateCylinder(
            radius, height, 20, 4, core::Float32, core::Int64, device);
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = obb.R_;
    transform.block<3, 1>(0, 3) =
            obb.center_ - obb.R_ * Eigen::Vector3d(radius, radius, height / 2);
    mesh.Transform(core::eigen_converter::EigenMatrixToTensor(transform));
    return mesh;
}

inline t::geometry::TriangleMesh CreatePrimitiveMesh(
        ObjectType type,
        const t::geometry::PointCloud& cluster,
        const core::Device& device) {
    geometry::PointCloud legacy = cluster.ToLegacy();
    auto obb = legacy.GetOrientedBoundingBox();
    switch (type) {
        case ObjectType::kWall:
        case ObjectType::kFloor:
        case ObjectType::kCeiling:
            // Planar regions are meshed as growing open surfaces by the
            // PlaneSurfaceAtlas, never as closed boxes.
            return t::geometry::TriangleMesh(device);
        case ObjectType::kBox:
            return CreateBoxMeshFromObb(obb, device);
        case ObjectType::kCylinder:
            return CreateCylinderMeshFromObb(obb, device);
        default:
            return t::geometry::TriangleMesh(device);
    }
}

inline t::geometry::PointCloud SelectCluster(
        const t::geometry::PointCloud& pcd,
        const core::Tensor& labels,
        int cluster_id) {
    core::Tensor mask = labels.Eq(cluster_id);
    return pcd.SelectByMask(mask);
}

// ---------------------------------------------------------------------------
// Planar partitioning.
// ---------------------------------------------------------------------------

/// Classify a planar patch by its normal. World frame equals the first camera
/// frame (x right, y down, z forward), so horizontal surfaces have |n.y|~1;
/// the floor lies below the camera (centroid y > 0 with y pointing down).
inline ObjectType ClassifyPatch(const Eigen::Vector3d& normal,
                                const Eigen::Vector3d& centroid) {
    if (std::abs(normal.y()) > 0.8) {
        return centroid.y() > 0.0 ? ObjectType::kFloor : ObjectType::kCeiling;
    }
    return ObjectType::kWall;
}

/// Detect planar patches and split each into plane-local tiles of
/// \p config.tile_size. Marks the claimed full-resolution points in
/// \p claimed. Patch detection runs on a downsampled copy for speed; point
/// membership uses the patch OBB on the full-resolution cloud.
/// Only architectural surfaces are promoted to plane regions: horizontal
/// patches must lie in the floor/ceiling height band (\p floor_y_hint /
/// \p ceiling_y_hint, world y points DOWN so the floor has the largest y)
/// and vertical patches must be wall-sized. Furniture-sized patches (sofa
/// seats, table tops, backrests) are left unclaimed so the DBSCAN object
/// path preserves their real shape.
inline std::vector<RegionCandidate> PartitionPlanarTiles(
        const geometry::PointCloud& legacy,
        const SegmentationConfig& config,
        const std::optional<double>& floor_y_hint,
        const std::optional<double>& ceiling_y_hint,
        std::vector<bool>& claimed) {
    std::vector<RegionCandidate> tiles;
    if (!legacy.HasNormals()) {
        return tiles;
    }

    auto down = legacy.VoxelDownSample(0.025);
    if (!down || !down->HasNormals() || down->points_.size() < 100) {
        return tiles;
    }
    std::vector<std::shared_ptr<geometry::OrientedBoundingBox>> patches;
    try {
        patches = down->DetectPlanarPatches(
                60.0, 75.0, 0.75,
                /*min_plane_edge_length=*/config.tile_size * 0.5,
                /*min_num_points=*/0, geometry::KDTreeSearchParamKNN(30));
    } catch (const std::exception& e) {
        utility::LogWarning("DetectPlanarPatches failed: {}", e.what());
        return tiles;
    }

    for (const auto& patch : patches) {
        if (!patch) {
            continue;
        }
        // Inflate the thin (normal) extent so full-resolution points slightly
        // off the downsampled plane still belong to the patch.
        geometry::OrientedBoundingBox obb = *patch;
        Eigen::Vector3d extent = obb.extent_;
        extent(2) = std::max(extent(2) + 0.04, 0.05);
        obb.extent_ = extent;

        const std::vector<size_t> member_indices =
                obb.GetPointIndicesWithinBoundingBox(legacy.points_);
        if (member_indices.size() <
            static_cast<size_t>(config.min_tile_points)) {
            continue;
        }

        Eigen::Vector3d normal = patch->R_.col(2).normalized();
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (size_t idx : member_indices) {
            centroid += legacy.points_[idx];
        }
        centroid /= static_cast<double>(member_indices.size());
        const ObjectType type = ClassifyPatch(normal, centroid);
        const double plane_d = -normal.dot(patch->center_);

        // Architectural surface filter. Area is measured from the point
        // count, not the OBB rectangle (which overestimates sparse patches).
        const double patch_area =
                EstimateSurfaceArea(member_indices.size(), config.voxel_size);
        if (type == ObjectType::kWall) {
            // Vertical extent of the patch box along world y.
            const double y_extent =
                    std::abs(patch->R_.col(0).y()) * patch->extent_(0) +
                    std::abs(patch->R_.col(1).y()) * patch->extent_(1);
            if (y_extent < config.min_wall_height &&
                patch_area < config.min_patch_area_m2) {
                continue;  // backrest-sized: leave to the object path
            }
        } else if (type == ObjectType::kFloor) {
            // Furniture tops sit above the floor (smaller y with y down).
            if (floor_y_hint) {
                if (centroid.y() < *floor_y_hint - config.floor_band) {
                    continue;
                }
            } else if (patch_area < config.min_patch_area_m2) {
                continue;  // no floor known yet: only a large patch may seed
            }
        } else {  // kCeiling
            if (ceiling_y_hint) {
                if (centroid.y() > *ceiling_y_hint + config.floor_band) {
                    continue;
                }
            } else if (patch_area < config.min_patch_area_m2) {
                continue;
            }
        }

        // Canonical in-plane axes so tile coordinates are reproducible.
        Eigen::Vector3d u, v;
        CanonicalPlaneAxes(normal, u, v);

        // Bucket member points into (i, j) tiles in plane coordinates.
        std::map<std::pair<int, int>, std::vector<size_t>> buckets;
        const double tile = std::max(0.25, config.tile_size);
        for (size_t idx : member_indices) {
            if (claimed[idx]) {
                continue;
            }
            const Eigen::Vector3d& p = legacy.points_[idx];
            const int i = static_cast<int>(std::floor(u.dot(p) / tile));
            const int j = static_cast<int>(std::floor(v.dot(p) / tile));
            buckets[{i, j}].push_back(idx);
        }

        for (const auto& [tile_ij, indices] : buckets) {
            if (indices.size() <
                static_cast<size_t>(config.min_tile_points)) {
                continue;
            }
            double min_a = std::numeric_limits<double>::max();
            double max_a = std::numeric_limits<double>::lowest();
            double min_b = min_a, max_b = max_a;
            for (size_t idx : indices) {
                const Eigen::Vector3d& p = legacy.points_[idx];
                const double a = u.dot(p);
                const double b = v.dot(p);
                min_a = std::min(min_a, a);
                max_a = std::max(max_a, a);
                min_b = std::min(min_b, b);
                max_b = std::max(max_b, b);
            }

            RegionCandidate candidate;
            candidate.type = type;
            candidate.plane =
                    Eigen::Vector4d(normal(0), normal(1), normal(2), plane_d);
            candidate.tile_i = tile_ij.first;
            candidate.tile_j = tile_ij.second;
            candidate.area_m2 = (max_a - min_a) * (max_b - min_b);
            candidate.points = t::geometry::PointCloud::FromLegacy(
                    *legacy.SelectByIndex(indices));
            tiles.push_back(std::move(candidate));
            for (size_t idx : indices) {
                claimed[idx] = true;
            }
        }
    }
    return tiles;
}

// ---------------------------------------------------------------------------
// Main entry: partition extracted surface, update registry, freeze regions.
// ---------------------------------------------------------------------------

inline std::vector<FrozenObjectCandidate> ProcessExtractedSurface(
        const t::geometry::PointCloud& surface_pcd,
        t::pipelines::slam::Model& model,
        RegionRegistry& registry,
        PlaneSurfaceAtlas& atlas,
        const SegmentationConfig& config,
        int& next_object_id) {
    std::vector<FrozenObjectCandidate> frozen_now;
    if (!surface_pcd.HasPointPositions() ||
        surface_pcd.GetPointPositions().GetLength() <
                config.min_cluster_points) {
        return frozen_now;
    }

    // Meshes are only consumed by the CPU-side renderer/writer; building
    // them on CPU avoids per-mesh GPU->CPU copy warnings.
    const core::Device mesh_device("CPU:0");

    t::geometry::PointCloud pcd_cpu = surface_pcd.To(core::Device("CPU:0"));
    const geometry::PointCloud legacy = pcd_cpu.ToLegacy();
    std::vector<bool> claimed(legacy.points_.size(), false);

    // 1. Planar regions (walls/floors/ceilings) as plane-local tiles.
    // Known floor/ceiling heights from already-frozen surfaces gate which
    // horizontal patches may become floor/ceiling (world y points down, so
    // the floor is at the largest y).
    std::vector<RegionCandidate> candidates;
    if (config.use_planar_patches) {
        std::optional<double> floor_y_hint, ceiling_y_hint;
        for (const auto& s : atlas.Snapshot()) {
            const double ny = s.plane(1);
            if (std::abs(ny) < 0.5) {
                continue;
            }
            const double y = -s.plane(3) / ny;
            if (s.type == ObjectType::kFloor) {
                floor_y_hint = floor_y_hint ? std::max(*floor_y_hint, y) : y;
            } else if (s.type == ObjectType::kCeiling) {
                ceiling_y_hint =
                        ceiling_y_hint ? std::min(*ceiling_y_hint, y) : y;
            }
        }
        candidates = PartitionPlanarTiles(legacy, config, floor_y_hint,
                                          ceiling_y_hint, claimed);
    }

    // 2. Non-planar remainder: DBSCAN object clusters (box/cylinder/generic).
    std::vector<size_t> claimed_indices;
    for (size_t i = 0; i < claimed.size(); ++i) {
        if (claimed[i]) {
            claimed_indices.push_back(i);
        }
    }
    t::geometry::PointCloud residual =
            claimed_indices.empty()
                    ? pcd_cpu
                    : t::geometry::PointCloud::FromLegacy(
                              *legacy.SelectByIndex(claimed_indices, true));
    if (residual.HasPointPositions() &&
        residual.GetPointPositions().GetLength() >=
                config.min_cluster_points) {
        core::Tensor labels = residual.ClusterDBSCAN(
                config.dbscan_eps, config.min_cluster_points, false);
        const int64_t num_points = labels.GetLength();
        int max_label = -1;
        const int32_t* label_data = labels.GetDataPtr<int32_t>();
        for (int64_t i = 0; i < num_points; ++i) {
            max_label = std::max(max_label, label_data[i]);
        }
        for (int cluster_id = 0; cluster_id <= max_label; ++cluster_id) {
            t::geometry::PointCloud cluster =
                    SelectCluster(residual, labels, cluster_id);
            if (cluster.GetPointPositions().GetLength() <
                config.min_cluster_points) {
                continue;
            }
            RegionCandidate candidate;
            candidate.type = ClassifyCluster(cluster);
            if (candidate.type == ObjectType::kWall) {
                // The architectural filter only guards the patch path, so a
                // wall-like cluster must pass the same size gate before it
                // may join the plane atlas; otherwise (e.g. a sofa backrest)
                // it is demoted to a generic object and keeps its scanned
                // shape via the marching-cubes mesh.
                geometry::PointCloud cluster_legacy = cluster.ToLegacy();
                auto obb = cluster_legacy.GetOrientedBoundingBox();
                Eigen::Vector3d extent = obb.extent_;
                const int thin_axis = static_cast<int>(std::distance(
                        extent.data(),
                        std::min_element(extent.data(), extent.data() + 3)));
                double y_extent = 0.0;
                for (int k = 0; k < 3; ++k) {
                    if (k != thin_axis) {
                        y_extent += std::abs(obb.R_.col(k).y()) * extent(k);
                    }
                }
                const double area = EstimateSurfaceArea(
                        cluster_legacy.points_.size(), config.voxel_size);
                if (y_extent >= config.min_wall_height ||
                    area >= config.min_patch_area_m2) {
                    const Eigen::Vector3d n =
                            obb.R_.col(thin_axis).normalized();
                    candidate.plane << n(0), n(1), n(2), -n.dot(obb.center_);
                } else {
                    candidate.type = ObjectType::kGeneric;
                }
            }
            const auto aabb = cluster.GetAxisAlignedBoundingBox().ToLegacy();
            const Eigen::Vector3d ext = aabb.GetExtent();
            // Rough footprint area for the info panel.
            std::array<double, 3> sorted{ext(0), ext(1), ext(2)};
            std::sort(sorted.begin(), sorted.end());
            candidate.area_m2 = sorted[1] * sorted[2];
            candidate.points = std::move(cluster);
            candidates.push_back(std::move(candidate));
        }
    }
    if (candidates.empty()) {
        return frozen_now;
    }

    // 3. Block keys per candidate: the registry's spatial index unit.
    std::vector<core::Tensor> block_keys(candidates.size());
    for (size_t c = 0; c < candidates.size(); ++c) {
        block_keys[c] = CollectBlockKeys(model.voxel_grid_,
                                         candidates[c].points,
                                         config.trunc_multiplier,
                                         config.voxel_size);
    }

    // 4. Stability tracking via the registry.
    std::vector<int> ready_ids;
    const std::vector<int> candidate_region_ids = registry.UpdateBatch(
            candidates, block_keys, config, next_object_id, ready_ids);

    // 5. Freeze regions that just became stable.
    for (int region_id : ready_ids) {
        int c = -1;
        for (size_t i = 0; i < candidate_region_ids.size(); ++i) {
            if (candidate_region_ids[i] == region_id) {
                c = static_cast<int>(i);
                break;
            }
        }
        if (c < 0 || block_keys[c].NumElements() == 0) {
            continue;
        }

        model.FreezeBlocks(block_keys[c]);
        registry.MarkFrozen(region_id);

        // Planar tiles feed the growing surface atlas: the mesh (an open
        // quad surface, not a closed box) is emitted below after the atlas
        // rebuilds the affected surfaces.
        if (IsPlanarType(candidates[c].type) &&
            candidates[c].plane.head<3>().norm() > 1e-9) {
            atlas.AddTile(candidates[c].type, candidates[c].plane,
                          candidates[c].points.ToLegacy(), block_keys[c],
                          config, next_object_id);
            continue;
        }

        FrozenObjectCandidate candidate;
        candidate.id = region_id;
        candidate.type = candidates[c].type;
        candidate.plane = candidates[c].plane;
        candidate.tile_i = candidates[c].tile_i;
        candidate.tile_j = candidates[c].tile_j;
        candidate.area_m2 = candidates[c].area_m2;
        candidate.block_keys = block_keys[c];
        candidate.bounds =
                candidates[c].points.GetAxisAlignedBoundingBox().ToLegacy();

        // Objects keep their real scanned shape: marching-cubes mesh from
        // the frozen TSDF blocks. Primitive box/cylinder shapes are only a
        // fallback (the type label is still reported in GUI/JSON).
        candidate.mesh = model.ExtractTriangleMeshIncluding(
                                      config.mesh_weight_threshold, -1,
                                      candidate.block_keys)
                                 .To(mesh_device);
        if (!candidate.mesh.HasVertexPositions()) {
            candidate.mesh = CreatePrimitiveMesh(
                    candidate.type == ObjectType::kGeneric
                            ? ObjectType::kBox
                            : candidate.type,
                    candidates[c].points, mesh_device);
        }
        if (candidate.mesh.HasVertexPositions()) {
            frozen_now.push_back(std::move(candidate));
        }
    }

    // 6. Rebuild grown/connected surfaces and emit them as replacements
    // (same id => consumers replace the previous mesh instead of adding).
    for (int surface_id : atlas.RebuildDirtyMeshes(config)) {
        const geometry::TriangleMesh mesh_legacy = atlas.GetMesh(surface_id);
        if (mesh_legacy.vertices_.empty()) {
            continue;
        }
        const PlaneSurfaceAtlas::SurfaceInfo info =
                atlas.GetInfo(surface_id);
        FrozenObjectCandidate candidate;
        candidate.id = surface_id;
        candidate.type = info.type;
        candidate.plane = info.plane;
        candidate.area_m2 = info.area_m2;
        candidate.cell_size = info.cell_size;
        candidate.cell_count = info.cell_count;
        candidate.closure = info.closure;
        candidate.connected = info.connected;
        candidate.block_keys = info.block_keys;
        candidate.mesh = t::geometry::TriangleMesh::FromLegacy(
                mesh_legacy, core::Float32, core::Int64, mesh_device);
        candidate.bounds = mesh_legacy.GetAxisAlignedBoundingBox();
        frozen_now.push_back(std::move(candidate));
    }
    return frozen_now;
}

}  // namespace object_mesh
}  // namespace examples
}  // namespace open3d
