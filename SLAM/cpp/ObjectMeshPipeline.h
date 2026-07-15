// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "open3d/Open3D.h"
#include "open3d/t/pipelines/slam/Model.h"

namespace open3d {
namespace examples {
namespace object_mesh {

enum class ObjectType { kWall, kBox, kCylinder, kGeneric };

inline const char* ObjectTypeName(ObjectType type) {
    switch (type) {
        case ObjectType::kWall:
            return "wall";
        case ObjectType::kBox:
            return "box";
        case ObjectType::kCylinder:
            return "cylinder";
        default:
            return "generic";
    }
}

struct ClusterSignature {
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d obb_extent = Eigen::Vector3d::Zero();
    ObjectType type = ObjectType::kGeneric;

    bool Matches(const ClusterSignature& other,
                 double centroid_eps,
                 double extent_iou_min) const {
        if (type != other.type) {
            return false;
        }
        if ((centroid - other.centroid).norm() > centroid_eps) {
            return false;
        }
        const Eigen::Vector3d min_extent =
                obb_extent.cwiseMin(other.obb_extent);
        const Eigen::Vector3d max_extent =
                obb_extent.cwiseMax(other.obb_extent);
        const double intersection = min_extent.prod();
        const double union_volume = max_extent.prod();
        if (union_volume <= 0.0) {
            return false;
        }
        return (intersection / union_volume) >= extent_iou_min;
    }
};

struct SegmentationConfig {
    double dbscan_eps = 0.01;
    int dbscan_min_points = 15;
    int min_cluster_points = 5000;
    int stability_frames = 5;
    bool auto_freeze = true;
    /// When true, always extract TSDF triangle mesh via ExtractTriangleMeshIncluding
    /// instead of analytic primitive meshes (wall/box/cylinder).
    bool tsdf_mesh_only = false;
    /// When true, ProcessExtractedSurface only clusters/tracks; call
    /// ApplyFreezeAndExtractMesh under model_mutex afterward.
    bool defer_model_ops = false;
    float voxel_size = 3.0f / 512.0f;
    float trunc_multiplier = 8.0f;
    float mesh_weight_threshold = 3.0f;
    double centroid_match_eps = 0.15;
    double extent_iou_min = 0.7;
    int block_resolution = 16;
    double max_triangle_edge_multiplier = 2.0;
    double cluster_outlier_std_ratio = 2.0;
    double max_cluster_extent_m = 1.2;
    double max_triangle_aspect_ratio = 20.0;
    /// Block-grid dilation for freeze/mesh extract; 0 = auto from trunc_multiplier.
    int region_seam_block_radius = 0;
    /// Max mesh-extract retries before dropping a pending-freeze cluster.
    int max_pending_freeze_attempts = 20;
    /// Minimum readiness score (0-1) before committing a non-empty mesh.
    float min_region_readiness = 0.80f;
    /// Max hole-defer cycles before flush / give-up escalation.
    int max_holey_defer_attempts = 40;
    /// When true, defer mesh commit for low-readiness (holey) regions.
    bool defer_holey_regions = true;
    /// Flush holey regions on session stop / exit.
    bool flush_holey_on_exit = true;
    /// Flush when all pending clusters have been hole-deferred at least once.
    bool flush_when_only_holey_left = true;
    /// Set by caller each region check: true if camera moved enough in window.
    bool camera_moved_since_last_check = true;
    /// When false, stationary/noise motion blocks mesh commit and stable_frames.
    bool require_camera_motion = true;
    /// Net pose displacement thresholds (anchor → current), not summed jitter.
    double min_motion_translation_m = 0.05;
    double min_motion_rotation_deg = 8.0;
    /// Require map growth together with pose motion.
    int min_hash_delta_blocks = 2;
    /// Skip region mesh until this many frames (warmup).
    int region_motion_warmup_frames = 45;
    /// Local void (surface cell without mesh) hard-reject thresholds.
    double max_void_ratio = 0.08;
    int max_void_blob_cells = 32;
    double max_boundary_void_ratio = 0.10;
};

struct RegionReadinessReport {
    float score = 0.f;
    double yield_ratio = 0.0;
    double occupancy = 0.0;
    double boundary_ratio = 1.0;
    int64_t mesh_vertex_count = 0;
    int64_t surface_point_count = 0;
};

struct RegionVoidReport {
    double void_ratio = 0.0;
    double boundary_void_ratio = 0.0;
    int void_cells = 0;
    int surface_cells = 0;
    int max_void_blob = 0;
    double void_ratio_before_sanitize = 0.0;
    double void_ratio_after_sanitize = 0.0;
};

inline void RelativePoseMetrics(const core::Tensor& T_from,
                                const core::Tensor& T_to,
                                double& translation_m,
                                double& rotation_deg) {
    translation_m = 0.0;
    rotation_deg = 0.0;
    if (T_from.NumElements() == 0 || T_to.NumElements() == 0) {
        return;
    }
    const Eigen::Matrix4d from =
            core::eigen_converter::TensorToEigenMatrixXd(T_from);
    const Eigen::Matrix4d to =
            core::eigen_converter::TensorToEigenMatrixXd(T_to);
    const Eigen::Matrix4d delta = from.inverse() * to;
    translation_m = delta.block<3, 1>(0, 3).norm();
    const double trace = delta.block<3, 3>(0, 0).trace();
    const double cos_angle =
            std::max(-1.0, std::min(1.0, (trace - 1.0) * 0.5));
    rotation_deg = std::acos(cos_angle) * 180.0 / 3.14159265358979323846;
}

struct CameraMotionSample {
    double net_translation_m = 0.0;
    double net_rotation_deg = 0.0;
    int64_t hash_delta = 0;
    int frame_id = 0;
};

/// True when net pose moved enough AND hash map grew (unless motion gate off).
inline bool CameraMovedEnough(const SegmentationConfig& config,
                              const CameraMotionSample& sample) {
    if (!config.require_camera_motion) {
        return true;
    }
    if (sample.frame_id < config.region_motion_warmup_frames) {
        return false;
    }
    const bool pose_ok =
            sample.net_translation_m >= config.min_motion_translation_m ||
            sample.net_rotation_deg >= config.min_motion_rotation_deg;
    const bool hash_ok =
            sample.hash_delta >=
            static_cast<int64_t>(config.min_hash_delta_blocks);
    return pose_ok && hash_ok;
}

inline bool CameraMovedEnough(const SegmentationConfig& config,
                              double translation_m,
                              double rotation_deg) {
    CameraMotionSample sample;
    sample.net_translation_m = translation_m;
    sample.net_rotation_deg = rotation_deg;
    sample.hash_delta = config.min_hash_delta_blocks;
    sample.frame_id = config.region_motion_warmup_frames;
    return CameraMovedEnough(config, sample);
}

inline bool IsValidRegionMesh(const t::geometry::TriangleMesh& mesh,
                              int min_triangles = 1) {
    if (!mesh.HasVertexPositions() || !mesh.HasTriangleIndices()) {
        return false;
    }
    return mesh.GetTriangleIndices().GetLength() >= min_triangles;
}

inline double ComputeClusterOccupancy(const t::geometry::PointCloud& cluster,
                                      float voxel_size) {
    geometry::PointCloud legacy = cluster.ToLegacy();
    if (legacy.points_.empty()) {
        return 0.0;
    }
    const geometry::AxisAlignedBoundingBox aabb =
            legacy.GetAxisAlignedBoundingBox();
    const double cell = std::max(0.01, static_cast<double>(voxel_size) * 2.0);
    const Eigen::Vector3d extent = aabb.GetExtent();
    const int nx =
            std::max(1, static_cast<int>(extent.x() / cell) + 1);
    const int ny =
            std::max(1, static_cast<int>(extent.y() / cell) + 1);
    const int nz =
            std::max(1, static_cast<int>(extent.z() / cell) + 1);
    const int64_t expected =
            static_cast<int64_t>(nx) * ny * nz;
    if (expected <= 0) {
        return 0.0;
    }

    std::unordered_set<int64_t> cells;
    cells.reserve(legacy.points_.size());
    const Eigen::Vector3d origin = aabb.min_bound_;
    for (const auto& point : legacy.points_) {
        int ix = static_cast<int>((point.x() - origin.x()) / cell);
        int iy = static_cast<int>((point.y() - origin.y()) / cell);
        int iz = static_cast<int>((point.z() - origin.z()) / cell);
        ix = std::clamp(ix, 0, nx - 1);
        iy = std::clamp(iy, 0, ny - 1);
        iz = std::clamp(iz, 0, nz - 1);
        const int64_t key =
                (static_cast<int64_t>(ix) * ny + iy) * nz + iz;
        cells.insert(key);
    }
    return static_cast<double>(cells.size()) /
           static_cast<double>(expected);
}

inline double ComputeMeshBoundaryRatio(const geometry::TriangleMesh& mesh) {
    if (mesh.triangles_.empty()) {
        return 1.0;
    }
    std::unordered_map<uint64_t, int> edge_count;
    edge_count.reserve(mesh.triangles_.size() * 3);
    auto edge_key = [](int a, int b) {
        if (a > b) {
            std::swap(a, b);
        }
        return (static_cast<uint64_t>(a) << 32) |
               static_cast<uint32_t>(b);
    };
    for (const auto& tri : mesh.triangles_) {
        edge_count[edge_key(tri(0), tri(1))]++;
        edge_count[edge_key(tri(1), tri(2))]++;
        edge_count[edge_key(tri(2), tri(0))]++;
    }
    int boundary_edges = 0;
    const int total_edges = static_cast<int>(edge_count.size());
    for (const auto& entry : edge_count) {
        if (entry.second == 1) {
            ++boundary_edges;
        }
    }
    if (total_edges == 0) {
        return 1.0;
    }
    return static_cast<double>(boundary_edges) /
           static_cast<double>(total_edges);
}

struct GridIndex3 {
    int x = 0;
    int y = 0;
    int z = 0;
    bool operator==(const GridIndex3& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct GridIndex3Hash {
    size_t operator()(const GridIndex3& k) const {
        return (static_cast<size_t>(k.x) * 73856093u) ^
               (static_cast<size_t>(k.y) * 19349663u) ^
               (static_cast<size_t>(k.z) * 83492791u);
    }
};

inline void CollectPointCells(
        const std::vector<Eigen::Vector3d>& points,
        const Eigen::Vector3d& origin,
        double cell,
        int nx,
        int ny,
        int nz,
        std::unordered_set<GridIndex3, GridIndex3Hash>& out) {
    for (const auto& point : points) {
        int ix = static_cast<int>((point.x() - origin.x()) / cell);
        int iy = static_cast<int>((point.y() - origin.y()) / cell);
        int iz = static_cast<int>((point.z() - origin.z()) / cell);
        ix = std::clamp(ix, 0, nx - 1);
        iy = std::clamp(iy, 0, ny - 1);
        iz = std::clamp(iz, 0, nz - 1);
        out.insert(GridIndex3{ix, iy, iz});
    }
}

inline int MaxConnectedBlobSize(
        const std::unordered_set<GridIndex3, GridIndex3Hash>& cells) {
    if (cells.empty()) {
        return 0;
    }
    std::unordered_set<GridIndex3, GridIndex3Hash> visited;
    visited.reserve(cells.size());
    int max_blob = 0;
    const int offsets[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
                               {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (const auto& start : cells) {
        if (visited.count(start)) {
            continue;
        }
        int blob = 0;
        std::queue<GridIndex3> q;
        q.push(start);
        visited.insert(start);
        while (!q.empty()) {
            const GridIndex3 cur = q.front();
            q.pop();
            ++blob;
            for (const auto& off : offsets) {
                GridIndex3 nxt{cur.x + off[0], cur.y + off[1],
                               cur.z + off[2]};
                if (!cells.count(nxt) || visited.count(nxt)) {
                    continue;
                }
                visited.insert(nxt);
                q.push(nxt);
            }
        }
        max_blob = std::max(max_blob, blob);
    }
    return max_blob;
}

inline RegionVoidReport ComputeLocalVoidFromPoints(
        const geometry::AxisAlignedBoundingBox& aabb,
        const std::vector<Eigen::Vector3d>& surface_points,
        const std::vector<Eigen::Vector3d>& mesh_vertices,
        float voxel_size,
        const std::vector<Eigen::Vector3d>& boundary_strip_centers,
        double boundary_radius) {
    RegionVoidReport report;
    if (surface_points.empty()) {
        return report;
    }
    const double cell = std::max(0.01, static_cast<double>(voxel_size) * 2.0);
    const Eigen::Vector3d extent = aabb.GetExtent();
    const int nx = std::max(1, static_cast<int>(extent.x() / cell) + 1);
    const int ny = std::max(1, static_cast<int>(extent.y() / cell) + 1);
    const int nz = std::max(1, static_cast<int>(extent.z() / cell) + 1);
    const Eigen::Vector3d origin = aabb.min_bound_;

    std::unordered_set<GridIndex3, GridIndex3Hash> surface_cells;
    std::unordered_set<GridIndex3, GridIndex3Hash> mesh_cells;
    CollectPointCells(surface_points, origin, cell, nx, ny, nz, surface_cells);
    CollectPointCells(mesh_vertices, origin, cell, nx, ny, nz, mesh_cells);

    std::unordered_set<GridIndex3, GridIndex3Hash> void_cells;
    for (const auto& cell_idx : surface_cells) {
        if (!mesh_cells.count(cell_idx)) {
            void_cells.insert(cell_idx);
        }
    }
    report.surface_cells = static_cast<int>(surface_cells.size());
    report.void_cells = static_cast<int>(void_cells.size());
    report.void_ratio =
            static_cast<double>(report.void_cells) /
            static_cast<double>(std::max(report.surface_cells, 1));
    report.max_void_blob = MaxConnectedBlobSize(void_cells);
    report.void_ratio_after_sanitize = report.void_ratio;

    if (!boundary_strip_centers.empty() && boundary_radius > 0.0) {
        int boundary_surface = 0;
        int boundary_void = 0;
        const double r2 = boundary_radius * boundary_radius;
        for (const auto& cell_idx : surface_cells) {
            const Eigen::Vector3d center =
                    origin + Eigen::Vector3d((cell_idx.x + 0.5) * cell,
                                             (cell_idx.y + 0.5) * cell,
                                             (cell_idx.z + 0.5) * cell);
            bool near_boundary = false;
            for (const auto& strip : boundary_strip_centers) {
                if ((center - strip).squaredNorm() <= r2) {
                    near_boundary = true;
                    break;
                }
            }
            if (!near_boundary) {
                continue;
            }
            ++boundary_surface;
            if (void_cells.count(cell_idx)) {
                ++boundary_void;
            }
        }
        report.boundary_void_ratio =
                static_cast<double>(boundary_void) /
                static_cast<double>(std::max(boundary_surface, 1));
    }
    return report;
}

inline bool PassesVoidGate(const RegionVoidReport& void_report,
                           const SegmentationConfig& config,
                           bool flush) {
    if (flush) {
        return void_report.void_ratio <= config.max_void_ratio * 1.5 &&
               void_report.max_void_blob <=
                       static_cast<int>(config.max_void_blob_cells * 1.5);
    }
    return void_report.void_ratio <= config.max_void_ratio &&
           void_report.max_void_blob <= config.max_void_blob_cells &&
           void_report.boundary_void_ratio <= config.max_boundary_void_ratio;
}

inline float EffectiveMinReadiness(const SegmentationConfig& config,
                                   int hole_defer_attempts,
                                   bool flush) {
    if (!config.defer_holey_regions) {
        return 0.f;
    }
    if (!flush) {
        return config.min_region_readiness;
    }
    if (hole_defer_attempts >= config.max_holey_defer_attempts * 3 / 4) {
        return 0.30f;
    }
    if (hole_defer_attempts >= config.max_holey_defer_attempts / 2) {
        return 0.45f;
    }
    return config.min_region_readiness;
}

struct RegionMeshSanitizeStats {
    int tris_before = 0;
    int tris_after = 0;
    int removed_invalid = 0;
    int removed_long_edge = 0;
    int removed_out_of_bounds = 0;
    int removed_needle = 0;
    double max_edge_seen = 0.0;
};

inline t::geometry::PointCloud TightenClusterForFreeze(
        const t::geometry::PointCloud& cluster,
        double std_ratio) {
    geometry::PointCloud legacy = cluster.ToLegacy();
    if (legacy.points_.size() < 50) {
        return cluster;
    }
    auto [filtered, _] =
            legacy.RemoveStatisticalOutliers(20, std_ratio, false);
    if (!filtered || filtered->points_.empty()) {
        return cluster;
    }
    return t::geometry::PointCloud::FromLegacy(*filtered, core::Float32,
                                               core::Device("CPU:0"));
}

inline RegionMeshSanitizeStats SanitizeRegionTriangleMesh(
        geometry::TriangleMesh& mesh,
        const geometry::AxisAlignedBoundingBox& cluster_bounds,
        double max_edge_length,
        double bounds_margin,
        double max_aspect_ratio,
        double long_edge_relax_scale = 1.0) {
    RegionMeshSanitizeStats stats;
    if (mesh.vertices_.empty() || mesh.triangles_.empty()) {
        return stats;
    }
    stats.tris_before = static_cast<int>(mesh.triangles_.size());
    const double edge_limit =
            max_edge_length * std::max(1.0, long_edge_relax_scale);

    geometry::AxisAlignedBoundingBox expanded = cluster_bounds;
    expanded.min_bound_ -= Eigen::Vector3d::Constant(bounds_margin);
    expanded.max_bound_ += Eigen::Vector3d::Constant(bounds_margin);

    const int n_verts = static_cast<int>(mesh.vertices_.size());
    std::vector<bool> remove_mask(static_cast<size_t>(stats.tris_before), false);
    auto inside_bounds = [&](const Eigen::Vector3d& p) {
        return (p.array() >= expanded.min_bound_.array()).all() &&
               (p.array() <= expanded.max_bound_.array()).all();
    };

    for (int i = 0; i < stats.tris_before; ++i) {
        const Eigen::Vector3i& tri = mesh.triangles_[static_cast<size_t>(i)];
        if (tri(0) < 0 || tri(1) < 0 || tri(2) < 0 || tri(0) >= n_verts ||
            tri(1) >= n_verts || tri(2) >= n_verts) {
            remove_mask[static_cast<size_t>(i)] = true;
            ++stats.removed_invalid;
            continue;
        }
        const Eigen::Vector3d& v0 = mesh.vertices_[static_cast<size_t>(tri(0))];
        const Eigen::Vector3d& v1 = mesh.vertices_[static_cast<size_t>(tri(1))];
        const Eigen::Vector3d& v2 = mesh.vertices_[static_cast<size_t>(tri(2))];
        if (!v0.allFinite() || !v1.allFinite() || !v2.allFinite()) {
            remove_mask[static_cast<size_t>(i)] = true;
            ++stats.removed_invalid;
            continue;
        }
        const double e01 = (v0 - v1).norm();
        const double e12 = (v1 - v2).norm();
        const double e20 = (v2 - v0).norm();
        stats.max_edge_seen =
                std::max({stats.max_edge_seen, e01, e12, e20});
        const double min_edge =
                std::min({e01, e12, e20, std::numeric_limits<double>::max()});
        const double max_edge = std::max({e01, e12, e20});
        if (min_edge > 1e-9 && max_edge / min_edge > max_aspect_ratio &&
            max_edge > edge_limit * 0.25) {
            remove_mask[static_cast<size_t>(i)] = true;
            ++stats.removed_needle;
            continue;
        }
        if (e01 > edge_limit || e12 > edge_limit || e20 > edge_limit) {
            remove_mask[static_cast<size_t>(i)] = true;
            ++stats.removed_long_edge;
            continue;
        }
        if (!inside_bounds(v0) || !inside_bounds(v1) || !inside_bounds(v2)) {
            remove_mask[static_cast<size_t>(i)] = true;
            ++stats.removed_out_of_bounds;
            continue;
        }
    }

    mesh.RemoveTrianglesByMask(remove_mask);
    mesh.RemoveUnreferencedVertices();
    mesh.RemoveDegenerateTriangles();
    stats.tris_after = static_cast<int>(mesh.triangles_.size());
    return stats;
}

struct FrozenObjectCandidate {
    int id = -1;
    ObjectType type = ObjectType::kGeneric;
    t::geometry::TriangleMesh mesh;
    core::Tensor block_keys;
    geometry::AxisAlignedBoundingBox bounds;
    ClusterSignature signature;
    /// Populated when defer_model_ops is true; consumed by ApplyFreezeAndExtractMesh.
    t::geometry::PointCloud source_cluster;
    /// Retry counter for pending freeze (lower mesh weight threshold over time).
    int freeze_attempt = 0;
    RegionReadinessReport readiness;
    RegionVoidReport void_report;
};

struct TrackedCluster {
    ClusterSignature signature;
    int stable_frames = 0;
    bool frozen = false;
    bool pending_freeze = false;
    int assigned_id = -1;
    int freeze_attempts = 0;
    int hole_defer_attempts = 0;
    float last_readiness = 0.f;
};

class ObjectFreezeTracker {
public:
    explicit ObjectFreezeTracker(const SegmentationConfig& config)
        : config_(config) {}

    void SetConfig(const SegmentationConfig& config) { config_ = config; }

    void MarkFreezeCommitted(int id) {
        for (auto& tracked : tracked_clusters_) {
            if (tracked.assigned_id == id && tracked.pending_freeze) {
                tracked.frozen = true;
                tracked.pending_freeze = false;
                return;
            }
        }
    }

    void MarkFreezeFailed(int id) {
        for (auto& tracked : tracked_clusters_) {
            if (tracked.assigned_id != id || tracked.frozen ||
                !tracked.pending_freeze) {
                continue;
            }
            ++tracked.freeze_attempts;
            if (tracked.freeze_attempts >= config_.max_pending_freeze_attempts) {
                utility::LogWarning(
                        "Region candidate {} gave up after {} mesh attempts.",
                        id, tracked.freeze_attempts);
                tracked.pending_freeze = false;
                tracked.stable_frames = 0;
                tracked.assigned_id = -1;
            } else {
                utility::LogInfo(
                        "Region candidate {} pending freeze, mesh not ready, "
                        "retry {}/{}.",
                        id, tracked.freeze_attempts,
                        config_.max_pending_freeze_attempts);
            }
            return;
        }
    }

    void MarkFreezeDeferred(int id,
                            float readiness,
                            double yield_ratio,
                            double occupancy,
                            double boundary_ratio) {
        for (auto& tracked : tracked_clusters_) {
            if (tracked.assigned_id != id || tracked.frozen ||
                !tracked.pending_freeze) {
                continue;
            }
            ++tracked.hole_defer_attempts;
            tracked.last_readiness = readiness;
            utility::LogInfo(
                    "Region candidate {} deferred (holey): readiness={:.2f} "
                    "yield={:.2f} occupancy={:.2f} boundary={:.2f} ({}/{})",
                    id, readiness, yield_ratio, occupancy, boundary_ratio,
                    tracked.hole_defer_attempts,
                    config_.max_holey_defer_attempts);
            return;
        }
    }

    void MarkFreezeDeferredVoid(int id, const RegionVoidReport& void_report,
                                float readiness) {
        for (auto& tracked : tracked_clusters_) {
            if (tracked.assigned_id != id || tracked.frozen ||
                !tracked.pending_freeze) {
                continue;
            }
            ++tracked.hole_defer_attempts;
            tracked.last_readiness = readiness;
            utility::LogInfo(
                    "Region candidate {} deferred (void): void_ratio={:.2f} "
                    "(before_sanitize={:.2f} after={:.2f}) max_blob={} "
                    "boundary_void={:.2f} readiness={:.2f} ({}/{})",
                    id, void_report.void_ratio,
                    void_report.void_ratio_before_sanitize,
                    void_report.void_ratio_after_sanitize,
                    void_report.max_void_blob, void_report.boundary_void_ratio,
                    readiness, tracked.hole_defer_attempts,
                    config_.max_holey_defer_attempts);
            return;
        }
    }

    void MarkFreezeDeferredStationary(int id) {
        for (auto& tracked : tracked_clusters_) {
            if (tracked.assigned_id != id || tracked.frozen ||
                !tracked.pending_freeze) {
                continue;
            }
            utility::LogInfo(
                    "Region candidate {} deferred: camera nearly stationary.",
                    id);
            return;
        }
    }

    int GetHoleDeferAttempts(int id) const {
        for (const auto& tracked : tracked_clusters_) {
            if (tracked.assigned_id == id) {
                return tracked.hole_defer_attempts;
            }
        }
        return 0;
    }

    bool ShouldFlushHoley(int id, bool session_stopping) const {
        if (!config_.defer_holey_regions) {
            return true;
        }
        const TrackedCluster* target = nullptr;
        for (const auto& tracked : tracked_clusters_) {
            if (tracked.assigned_id == id && tracked.pending_freeze &&
                !tracked.frozen) {
                target = &tracked;
                break;
            }
        }
        if (!target) {
            return false;
        }
        if (session_stopping && config_.flush_holey_on_exit) {
            return true;
        }
        if (target->hole_defer_attempts >= config_.max_holey_defer_attempts) {
            return true;
        }
        if (config_.flush_when_only_holey_left) {
            int pending = 0;
            int pending_holey = 0;
            for (const auto& tracked : tracked_clusters_) {
                if (tracked.pending_freeze && !tracked.frozen) {
                    ++pending;
                    if (tracked.hole_defer_attempts > 0) {
                        ++pending_holey;
                    }
                }
            }
            if (pending > 0 && pending == pending_holey) {
                return true;
            }
        }
        return false;
    }

    std::vector<FrozenObjectCandidate> Update(
            const std::vector<ClusterSignature>& cluster_signatures,
            int& next_object_id,
            const std::function<FrozenObjectCandidate(int, ObjectType,
                                                      const ClusterSignature&)>&
                    build_candidate) {
        std::vector<FrozenObjectCandidate> ready_to_freeze;
        if (!config_.auto_freeze) {
            return ready_to_freeze;
        }

        std::vector<bool> matched(cluster_signatures.size(), false);
        std::vector<bool> tracked_matched(tracked_clusters_.size(), false);
        for (size_t ti = 0; ti < tracked_clusters_.size(); ++ti) {
            auto& tracked = tracked_clusters_[ti];
            if (tracked.frozen) {
                continue;
            }
            int best_idx = -1;
            for (size_t i = 0; i < cluster_signatures.size(); ++i) {
                if (matched[i]) {
                    continue;
                }
                if (tracked.signature.Matches(
                            cluster_signatures[i], config_.centroid_match_eps,
                            config_.extent_iou_min)) {
                    best_idx = static_cast<int>(i);
                    break;
                }
            }
            if (best_idx >= 0) {
                matched[best_idx] = true;
                tracked_matched[ti] = true;
                tracked.signature = cluster_signatures[best_idx];
                const bool allow_stable =
                        !config_.require_camera_motion ||
                        config_.camera_moved_since_last_check ||
                        tracked.pending_freeze;
                if (allow_stable) {
                    ++tracked.stable_frames;
                }
                if (tracked.stable_frames >= config_.stability_frames) {
                    if (!tracked.pending_freeze) {
                        tracked.pending_freeze = true;
                        tracked.assigned_id = next_object_id++;
                    }
                    FrozenObjectCandidate candidate = build_candidate(
                            tracked.assigned_id, tracked.signature.type,
                            tracked.signature);
                    candidate.freeze_attempt = tracked.freeze_attempts;
                    ready_to_freeze.push_back(std::move(candidate));
                }
            } else if (!tracked.pending_freeze) {
                tracked.stable_frames = 0;
            }
        }

        {
            size_t write = 0;
            for (size_t ti = 0; ti < tracked_clusters_.size(); ++ti) {
                const TrackedCluster& cluster = tracked_clusters_[ti];
                const bool drop = !cluster.frozen && !cluster.pending_freeze &&
                                  cluster.stable_frames == 0 &&
                                  !(ti < tracked_matched.size() &&
                                    tracked_matched[ti]);
                if (!drop) {
                    if (write != ti) {
                        tracked_clusters_[write] = cluster;
                    }
                    ++write;
                }
            }
            tracked_clusters_.resize(write);
        }

        for (size_t i = 0; i < cluster_signatures.size(); ++i) {
            if (matched[i]) {
                continue;
            }
            TrackedCluster cluster;
            cluster.signature = cluster_signatures[i];
            cluster.stable_frames =
                    (config_.require_camera_motion &&
                     !config_.camera_moved_since_last_check)
                            ? 0
                            : 1;
            tracked_clusters_.push_back(cluster);
        }
        return ready_to_freeze;
    }

    int FrozenCount() const {
        int count = 0;
        for (const auto& tracked : tracked_clusters_) {
            if (tracked.frozen) {
                ++count;
            }
        }
        return count;
    }

    std::unordered_map<ObjectType, int> FrozenTypeCounts() const {
        std::unordered_map<ObjectType, int> counts;
        for (const auto& tracked : tracked_clusters_) {
            if (tracked.frozen) {
                ++counts[tracked.signature.type];
            }
        }
        return counts;
    }

    int PendingCount() const {
        int count = 0;
        for (const auto& tracked : tracked_clusters_) {
            if (tracked.pending_freeze && !tracked.frozen) {
                ++count;
            }
        }
        return count;
    }

    int PendingHoleyCount() const {
        int count = 0;
        for (const auto& tracked : tracked_clusters_) {
            if (tracked.pending_freeze && !tracked.frozen &&
                tracked.hole_defer_attempts > 0) {
                ++count;
            }
        }
        return count;
    }

    int TrackedCount() const {
        return static_cast<int>(tracked_clusters_.size());
    }

private:
    SegmentationConfig config_;
    std::vector<TrackedCluster> tracked_clusters_;
};

struct BlockKey3 {
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;
    bool operator==(const BlockKey3& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct BlockKey3Hash {
    size_t operator()(const BlockKey3& key) const {
        return (static_cast<size_t>(key.x) * 73856093u) ^
               (static_cast<size_t>(key.y) * 19349663u) ^
               (static_cast<size_t>(key.z) * 83492791u);
    }
};

// Open3D: Tensor({}) is 0-D and NumElements()==1, but GetLength() throws.
// Always use (0, 3) Int32 for empty block-key lists.
inline core::Tensor EmptyBlockKeys() {
    return core::Tensor({0, 3}, core::Int32, core::Device("CPU:0"));
}

inline bool IsEmptyBlockKeys(const core::Tensor& block_keys) {
    if (block_keys.NumDims() < 1) {
        return true;
    }
    return block_keys.GetShape()[0] == 0;
}

inline int64_t BlockKeyCount(const core::Tensor& block_keys) {
    if (IsEmptyBlockKeys(block_keys)) {
        return 0;
    }
    return block_keys.GetShape()[0];
}

/// Marching-cubes mesh extract needs neighbor blocks in inverse_index_map.
inline core::Tensor ExpandBlockKeysByBlockRadius(const core::Tensor& block_keys,
                                                 int radius_blocks) {
    if (IsEmptyBlockKeys(block_keys) || radius_blocks <= 0) {
        return IsEmptyBlockKeys(block_keys) ? EmptyBlockKeys() : block_keys;
    }
    core::Tensor keys_cpu =
            block_keys.To(core::Device("CPU:0")).Contiguous();
    const int32_t* data = keys_cpu.GetDataPtr<int32_t>();
    const int64_t n = BlockKeyCount(keys_cpu);
    const int32_t r = static_cast<int32_t>(radius_blocks);
    const size_t est = static_cast<size_t>(n) *
                       static_cast<size_t>((2 * r + 1) * (2 * r + 1) * (2 * r + 1));
    std::unordered_set<BlockKey3, BlockKey3Hash> expanded;
    expanded.reserve(est);
    for (int64_t i = 0; i < n; ++i) {
        const int32_t x = data[i * 3 + 0];
        const int32_t y = data[i * 3 + 1];
        const int32_t z = data[i * 3 + 2];
        for (int32_t dx = -r; dx <= r; ++dx) {
            for (int32_t dy = -r; dy <= r; ++dy) {
                for (int32_t dz = -r; dz <= r; ++dz) {
                    expanded.insert({x + dx, y + dy, z + dz});
                }
            }
        }
    }
    std::vector<int32_t> flat;
    flat.reserve(expanded.size() * 3);
    for (const auto& key : expanded) {
        flat.push_back(key.x);
        flat.push_back(key.y);
        flat.push_back(key.z);
    }
    return core::Tensor(flat, {static_cast<int64_t>(expanded.size()), 3},
                        core::Int32, core::Device("CPU:0"));
}

inline core::Tensor ExpandBlockKeysForMeshExtract(
        const core::Tensor& block_keys) {
    return ExpandBlockKeysByBlockRadius(block_keys, 1);
}

inline core::Tensor MergeBlockKeysTensor(const core::Tensor& keys_a,
                                         const core::Tensor& keys_b) {
    if (IsEmptyBlockKeys(keys_a)) {
        return IsEmptyBlockKeys(keys_b) ? EmptyBlockKeys() : keys_b;
    }
    if (IsEmptyBlockKeys(keys_b)) {
        return keys_a;
    }
    core::Tensor a_cpu = keys_a.To(core::Device("CPU:0")).Contiguous();
    core::Tensor b_cpu = keys_b.To(core::Device("CPU:0")).Contiguous();
    std::unordered_set<BlockKey3, BlockKey3Hash> merged;
    const int32_t* a_data = a_cpu.GetDataPtr<int32_t>();
    const int64_t a_n = BlockKeyCount(a_cpu);
    const int64_t b_n = BlockKeyCount(b_cpu);
    merged.reserve(static_cast<size_t>(a_n + b_n));
    for (int64_t i = 0; i < a_n; ++i) {
        merged.insert({a_data[i * 3 + 0], a_data[i * 3 + 1], a_data[i * 3 + 2]});
    }
    const int32_t* b_data = b_cpu.GetDataPtr<int32_t>();
    for (int64_t i = 0; i < b_n; ++i) {
        merged.insert({b_data[i * 3 + 0], b_data[i * 3 + 1], b_data[i * 3 + 2]});
    }
    if (merged.empty()) {
        return EmptyBlockKeys();
    }
    std::vector<int32_t> flat;
    flat.reserve(merged.size() * 3);
    for (const auto& key : merged) {
        flat.push_back(key.x);
        flat.push_back(key.y);
        flat.push_back(key.z);
    }
    return core::Tensor(flat, {static_cast<int64_t>(merged.size()), 3},
                        core::Int32, core::Device("CPU:0"));
}

inline int ComputeRegionSeamBlockRadius(const SegmentationConfig& config) {
    if (config.region_seam_block_radius > 0) {
        return config.region_seam_block_radius;
    }
    return std::max(2, static_cast<int>(std::ceil(
                               static_cast<double>(config.trunc_multiplier) / 4.0)));
}

inline core::Tensor CollectNeighborBlockKeys(const core::Tensor& source_keys,
                                             const core::Tensor& reference_keys,
                                             int radius_blocks) {
    if (IsEmptyBlockKeys(source_keys) || IsEmptyBlockKeys(reference_keys) ||
        radius_blocks <= 0) {
        return EmptyBlockKeys();
    }
    core::Tensor ref_cpu =
            reference_keys.To(core::Device("CPU:0")).Contiguous();
    const int32_t* ref_data = ref_cpu.GetDataPtr<int32_t>();
    const int64_t ref_n = BlockKeyCount(ref_cpu);
    const int32_t r = static_cast<int32_t>(radius_blocks);

    core::Tensor src_cpu = source_keys.To(core::Device("CPU:0")).Contiguous();
    const int32_t* src_data = src_cpu.GetDataPtr<int32_t>();
    const int64_t src_n = BlockKeyCount(src_cpu);
    std::vector<int32_t> flat;
    flat.reserve(static_cast<size_t>(src_n) * 3);
    for (int64_t i = 0; i < src_n; ++i) {
        const int32_t sx = src_data[i * 3 + 0];
        const int32_t sy = src_data[i * 3 + 1];
        const int32_t sz = src_data[i * 3 + 2];
        bool near_reference = false;
        for (int64_t j = 0; j < ref_n; ++j) {
            const int32_t dx = std::abs(sx - ref_data[j * 3 + 0]);
            const int32_t dy = std::abs(sy - ref_data[j * 3 + 1]);
            const int32_t dz = std::abs(sz - ref_data[j * 3 + 2]);
            if (dx <= r && dy <= r && dz <= r) {
                near_reference = true;
                break;
            }
        }
        if (near_reference) {
            flat.push_back(sx);
            flat.push_back(sy);
            flat.push_back(sz);
        }
    }
    if (flat.empty()) {
        return EmptyBlockKeys();
    }
    return core::Tensor(flat, {static_cast<int64_t>(flat.size() / 3), 3},
                        core::Int32, core::Device("CPU:0"));
}

inline geometry::AxisAlignedBoundingBox BlockKeysWorldAABB(
        const core::Tensor& block_keys,
        float voxel_size,
        int block_resolution) {
    if (IsEmptyBlockKeys(block_keys)) {
        return geometry::AxisAlignedBoundingBox();
    }
    const double block_extent =
            static_cast<double>(voxel_size) * block_resolution;
    core::Tensor keys_cpu =
            block_keys.To(core::Device("CPU:0")).Contiguous();
    const int32_t* data = keys_cpu.GetDataPtr<int32_t>();
    const int64_t n = BlockKeyCount(keys_cpu);
    Eigen::Vector3d min_b =
            Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    Eigen::Vector3d max_b =
            Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
    for (int64_t i = 0; i < n; ++i) {
        const Eigen::Vector3d block_min(data[i * 3 + 0] * block_extent,
                                        data[i * 3 + 1] * block_extent,
                                        data[i * 3 + 2] * block_extent);
        const Eigen::Vector3d block_max =
                block_min + Eigen::Vector3d::Constant(block_extent);
        min_b = min_b.cwiseMin(block_min);
        max_b = max_b.cwiseMax(block_max);
    }
    return geometry::AxisAlignedBoundingBox(min_b, max_b);
}

inline core::Tensor CollectBlockKeys(
        t::geometry::VoxelBlockGrid& vbg,
        const t::geometry::PointCloud& cluster,
        float trunc_multiplier) {
    const core::Device grid_device = vbg.GetHashMap().GetDevice();
    t::geometry::PointCloud cluster_on_grid = cluster;
    if (cluster.HasPointPositions() &&
        cluster.GetPointPositions().GetDevice() != grid_device) {
        cluster_on_grid = cluster.To(grid_device);
    }
    core::Tensor block_coords;
    try {
        block_coords = vbg.GetUniqueBlockCoordinates(cluster_on_grid,
                                                   trunc_multiplier);
    } catch (const std::exception&) {
        return EmptyBlockKeys();
    }
    if (IsEmptyBlockKeys(block_coords) || BlockKeyCount(block_coords) == 0) {
        return EmptyBlockKeys();
    }
    return block_coords.To(core::Device("CPU:0")).Contiguous();
}

inline ClusterSignature BuildClusterSignature(
        const t::geometry::PointCloud& cluster, ObjectType type) {
    ClusterSignature signature;
    signature.type = type;
    geometry::PointCloud legacy = cluster.ToLegacy();
    signature.centroid = legacy.GetCenter();
    auto obb = legacy.GetOrientedBoundingBox();
    signature.obb_extent = obb.extent_;
    return signature;
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
            return CreateWallMesh(cluster, device);
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

inline void ApplyFreezeAndExtractMesh(
        FrozenObjectCandidate& candidate,
        t::pipelines::slam::Model& model,
        const SegmentationConfig& config) {
    t::geometry::PointCloud cluster =
            TightenClusterForFreeze(candidate.source_cluster,
                                    config.cluster_outlier_std_ratio);
    if (!cluster.HasPointPositions()) {
        return;
    }

    const geometry::AxisAlignedBoundingBox cluster_bounds =
            cluster.GetAxisAlignedBoundingBox().ToLegacy();
    const double max_edge_length = std::min(
            static_cast<double>(config.voxel_size) * config.block_resolution *
                    config.max_triangle_edge_multiplier,
            std::max(0.05, cluster_bounds.GetMaxExtent() * 0.35));
    const double block_extent =
            static_cast<double>(config.voxel_size) * config.block_resolution;
    const double bounds_margin =
            std::max(static_cast<double>(config.voxel_size) *
                             config.trunc_multiplier,
                     block_extent);

    const int seam_radius = ComputeRegionSeamBlockRadius(config);
    core::Tensor raw_block_keys =
            CollectBlockKeys(model.voxel_grid_, cluster, config.trunc_multiplier);
    core::Tensor region_block_keys =
            ExpandBlockKeysByBlockRadius(raw_block_keys, seam_radius);
    const core::Tensor existing_frozen = model.GetFrozenBlockKeys();
    if (!IsEmptyBlockKeys(existing_frozen)) {
        region_block_keys = MergeBlockKeysTensor(
                region_block_keys,
                CollectNeighborBlockKeys(existing_frozen, raw_block_keys,
                                         seam_radius));
    }
    candidate.block_keys = region_block_keys;
    const core::Device mesh_device =
            model.voxel_grid_.GetHashMap().GetDevice();
    if (IsEmptyBlockKeys(candidate.block_keys)) {
        candidate.bounds = cluster_bounds;
        return;
    }

    geometry::AxisAlignedBoundingBox sanitize_bounds = cluster_bounds;
    const geometry::AxisAlignedBoundingBox block_bounds =
            BlockKeysWorldAABB(region_block_keys, config.voxel_size,
                               config.block_resolution);
    sanitize_bounds.min_bound_ =
            sanitize_bounds.min_bound_.cwiseMin(block_bounds.min_bound_);
    sanitize_bounds.max_bound_ =
            sanitize_bounds.max_bound_.cwiseMax(block_bounds.max_bound_);

    float weight_threshold = config.mesh_weight_threshold;
    if (candidate.freeze_attempt > 0) {
        weight_threshold = std::max(
                1.0f, config.mesh_weight_threshold -
                              0.5f * static_cast<float>(candidate.freeze_attempt));
    }

    if (config.tsdf_mesh_only || candidate.type == ObjectType::kGeneric) {
        try {
            utility::LogInfo(
                    "Region mesh extract start (candidate {}, blocks {}, "
                    "attempt {}).",
                    candidate.id, BlockKeyCount(candidate.block_keys),
                    candidate.freeze_attempt);
            candidate.mesh = model.ExtractTriangleMeshIncluding(
                    weight_threshold, -1, candidate.block_keys);
            utility::LogInfo(
                    "Region mesh extract done (candidate {}, verts {}).",
                    candidate.id,
                    candidate.mesh.HasVertexPositions()
                            ? candidate.mesh.GetVertexPositions().GetLength()
                            : 0);
        } catch (const std::exception&) {
            candidate.mesh = t::geometry::TriangleMesh(core::Device("CPU:0"));
        }
        candidate.mesh = candidate.mesh.To(core::Device("CPU:0"));
        if (candidate.mesh.HasVertexPositions() &&
            candidate.mesh.HasTriangleIndices()) {
            geometry::TriangleMesh legacy = candidate.mesh.ToLegacy();
            const std::vector<Eigen::Vector3d> surface_pts =
                    cluster.ToLegacy().points_;
            const RegionVoidReport void_before = ComputeLocalVoidFromPoints(
                    sanitize_bounds, surface_pts, legacy.vertices_,
                    config.voxel_size, {}, 0.0);
            // Interior extract: keep default long-edge clamp (no seam boost).
            const RegionMeshSanitizeStats sanitize_stats =
                    SanitizeRegionTriangleMesh(
                            legacy, sanitize_bounds, max_edge_length,
                            bounds_margin, config.max_triangle_aspect_ratio,
                            1.10);
            const RegionVoidReport void_after = ComputeLocalVoidFromPoints(
                    sanitize_bounds, surface_pts, legacy.vertices_,
                    config.voxel_size, {}, 0.0);
            candidate.void_report.void_ratio_before_sanitize =
                    void_before.void_ratio;
            candidate.void_report.void_ratio_after_sanitize =
                    void_after.void_ratio;
            if (legacy.triangles_.empty()) {
                candidate.mesh = t::geometry::TriangleMesh(core::Device("CPU:0"));
            } else {
                candidate.mesh =
                        t::geometry::TriangleMesh::FromLegacy(legacy);
            }
            if (sanitize_stats.removed_long_edge > 0) {
                utility::LogInfo(
                        "Region candidate {} sanitize removed {} long-edge "
                        "tris ({} -> {}), void {:.2f}->{:.2f}.",
                        candidate.id, sanitize_stats.removed_long_edge,
                        sanitize_stats.tris_before, sanitize_stats.tris_after,
                        void_before.void_ratio, void_after.void_ratio);
            }
        }
    } else {
        candidate.mesh =
                CreatePrimitiveMesh(candidate.type, cluster, mesh_device);
        candidate.mesh = candidate.mesh.To(core::Device("CPU:0"));
        if (candidate.mesh.HasVertexPositions() &&
            candidate.mesh.HasTriangleIndices()) {
            geometry::TriangleMesh legacy = candidate.mesh.ToLegacy();
            SanitizeRegionTriangleMesh(legacy, sanitize_bounds, max_edge_length,
                                       bounds_margin,
                                       config.max_triangle_aspect_ratio, 1.10);
            if (legacy.triangles_.empty()) {
                candidate.mesh = t::geometry::TriangleMesh(core::Device("CPU:0"));
            } else {
                candidate.mesh =
                        t::geometry::TriangleMesh::FromLegacy(legacy);
            }
        }
    }
    candidate.bounds = cluster_bounds;
    candidate.source_cluster = cluster;

    if (!IsValidRegionMesh(candidate.mesh)) {
        candidate.mesh = t::geometry::TriangleMesh(core::Device("CPU:0"));
        utility::LogInfo(
                "Region mesh not ready (weight/sanitize), will retry "
                "(candidate {}).",
                candidate.id);
    }
}

inline RegionReadinessReport ComputeRegionReadiness(
        const FrozenObjectCandidate& candidate,
        const t::geometry::PointCloud& cluster,
        t::pipelines::slam::Model& model,
        float weight_threshold,
        const SegmentationConfig& config) {
    RegionReadinessReport report;
    report.occupancy = ComputeClusterOccupancy(cluster, config.voxel_size);

    if (candidate.mesh.HasVertexPositions()) {
        report.mesh_vertex_count =
                candidate.mesh.GetVertexPositions().GetLength();
    }
    if (!IsEmptyBlockKeys(candidate.block_keys)) {
        try {
            t::geometry::PointCloud surface =
                    model.voxel_grid_.ExtractPointCloudIncluding(
                            weight_threshold, -1, candidate.block_keys);
            surface = surface.To(core::Device("CPU:0"));
            if (surface.HasPointPositions()) {
                report.surface_point_count =
                        surface.GetPointPositions().GetLength();
            }
        } catch (const std::exception&) {
            report.surface_point_count = 0;
        }
    }
    report.yield_ratio =
            static_cast<double>(report.mesh_vertex_count) /
            static_cast<double>(std::max<int64_t>(
                    report.surface_point_count, 1));

    if (IsValidRegionMesh(candidate.mesh)) {
        report.boundary_ratio =
                ComputeMeshBoundaryRatio(candidate.mesh.ToLegacy());
    }

    constexpr double kTargetYield = 0.12;
    constexpr double kTargetOccupancy = 0.30;
    constexpr double kMaxBoundary = 0.35;
    const auto clamp01 = [](double value) {
        return std::max(0.0, std::min(1.0, value));
    };
    double boundary_weight = 0.30;
    if (candidate.type == ObjectType::kWall) {
        boundary_weight = 0.15;
    }

    report.score = static_cast<float>(
            0.40 * clamp01(report.yield_ratio / kTargetYield) +
            0.30 * clamp01(report.occupancy / kTargetOccupancy) +
            boundary_weight *
                    clamp01(1.0 - report.boundary_ratio / kMaxBoundary));
    return report;
}

inline std::vector<Eigen::Vector3d> BlockKeysWorldCenters(
        const core::Tensor& block_keys,
        float voxel_size,
        int block_resolution) {
    std::vector<Eigen::Vector3d> centers;
    if (IsEmptyBlockKeys(block_keys)) {
        return centers;
    }
    core::Tensor cpu = block_keys.To(core::Device("CPU:0")).Contiguous();
    const int32_t* data = cpu.GetDataPtr<int32_t>();
    const int64_t n = BlockKeyCount(cpu);
    const double extent =
            static_cast<double>(voxel_size) * block_resolution;
    centers.reserve(static_cast<size_t>(n));
    for (int64_t i = 0; i < n; ++i) {
        centers.emplace_back((data[i * 3 + 0] + 0.5) * extent,
                             (data[i * 3 + 1] + 0.5) * extent,
                             (data[i * 3 + 2] + 0.5) * extent);
    }
    return centers;
}

inline RegionVoidReport ComputeRegionVoidReport(
        const FrozenObjectCandidate& candidate,
        const t::geometry::PointCloud& cluster,
        t::pipelines::slam::Model& model,
        float weight_threshold,
        const SegmentationConfig& config) {
    RegionVoidReport report;
    geometry::AxisAlignedBoundingBox aabb = candidate.bounds;
    if (aabb.IsEmpty() && cluster.HasPointPositions()) {
        aabb = cluster.GetAxisAlignedBoundingBox().ToLegacy();
    }
    if (aabb.IsEmpty()) {
        return report;
    }

    std::vector<Eigen::Vector3d> surface_points;
    if (!IsEmptyBlockKeys(candidate.block_keys)) {
        try {
            t::geometry::PointCloud surface =
                    model.voxel_grid_.ExtractPointCloudIncluding(
                            weight_threshold, -1, candidate.block_keys);
            surface = surface.To(core::Device("CPU:0"));
            if (surface.HasPointPositions()) {
                surface_points = surface.ToLegacy().points_;
            }
        } catch (const std::exception&) {
        }
    }
    if (surface_points.empty() && cluster.HasPointPositions()) {
        surface_points = cluster.ToLegacy().points_;
    }

    std::vector<Eigen::Vector3d> mesh_vertices;
    if (candidate.mesh.HasVertexPositions()) {
        mesh_vertices = candidate.mesh.ToLegacy().vertices_;
    }

    std::vector<Eigen::Vector3d> boundary_centers;
    const core::Tensor frozen = model.GetFrozenBlockKeys();
    if (!IsEmptyBlockKeys(frozen) && !IsEmptyBlockKeys(candidate.block_keys)) {
        const int seam = ComputeRegionSeamBlockRadius(config);
        core::Tensor neighbor = CollectNeighborBlockKeys(
                frozen, candidate.block_keys, seam);
        boundary_centers = BlockKeysWorldCenters(
                neighbor, config.voxel_size, config.block_resolution);
    }

    const double block_extent =
            static_cast<double>(config.voxel_size) * config.block_resolution;
    report = ComputeLocalVoidFromPoints(
            aabb, surface_points, mesh_vertices, config.voxel_size,
            boundary_centers, block_extent * 1.5);
    // Preserve sanitize before/after captured during mesh extract when present.
    const double before_sanitize =
            candidate.void_report.void_ratio_before_sanitize;
    report.void_ratio_after_sanitize = report.void_ratio;
    report.void_ratio_before_sanitize =
            (before_sanitize > 0.0) ? before_sanitize : report.void_ratio;
    return report;
}

inline float RegionExtractWeightThreshold(const FrozenObjectCandidate& candidate,
                                          const SegmentationConfig& config) {
    float weight_threshold = config.mesh_weight_threshold;
    if (candidate.freeze_attempt > 0) {
        weight_threshold = std::max(
                1.0f, config.mesh_weight_threshold -
                              0.5f * static_cast<float>(candidate.freeze_attempt));
    }
    return weight_threshold;
}

inline void CommitRegionFreeze(FrozenObjectCandidate& candidate,
                               t::pipelines::slam::Model& model) {
    if (IsEmptyBlockKeys(candidate.block_keys)) {
        return;
    }
    model.FreezeBlocks(candidate.block_keys);
    const int64_t triangle_count =
            candidate.mesh.GetTriangleIndices().GetLength();
    utility::LogInfo(
            "Region candidate {} committed (triangles={}, blocks={}, "
            "readiness={:.2f}, void={:.2f}->{:.2f}, boundary_void={:.2f}).",
            candidate.id, triangle_count, BlockKeyCount(candidate.block_keys),
            candidate.readiness.score,
            candidate.void_report.void_ratio_before_sanitize,
            candidate.void_report.void_ratio_after_sanitize,
            candidate.void_report.boundary_void_ratio);
    candidate.source_cluster = t::geometry::PointCloud();
}

struct RegionProcessResult {
    std::vector<FrozenObjectCandidate> committed;
    std::vector<FrozenObjectCandidate> seam_updates;
    int deferred_holey = 0;
    int deferred_void = 0;
    int deferred_stationary = 0;
    int failed_empty = 0;
};

inline bool BlocksNearEachOther(const core::Tensor& a,
                                const core::Tensor& b,
                                int radius_blocks) {
    if (IsEmptyBlockKeys(a) || IsEmptyBlockKeys(b) || radius_blocks < 0) {
        return false;
    }
    core::Tensor a_cpu = a.To(core::Device("CPU:0")).Contiguous();
    core::Tensor b_cpu = b.To(core::Device("CPU:0")).Contiguous();
    const int32_t* a_data = a_cpu.GetDataPtr<int32_t>();
    const int32_t* b_data = b_cpu.GetDataPtr<int32_t>();
    const int64_t na = BlockKeyCount(a_cpu);
    const int64_t nb = BlockKeyCount(b_cpu);
    const int32_t r = static_cast<int32_t>(radius_blocks);
    for (int64_t i = 0; i < na; ++i) {
        for (int64_t j = 0; j < nb; ++j) {
            const int32_t dx = std::abs(a_data[i * 3] - b_data[j * 3]);
            const int32_t dy =
                    std::abs(a_data[i * 3 + 1] - b_data[j * 3 + 1]);
            const int32_t dz =
                    std::abs(a_data[i * 3 + 2] - b_data[j * 3 + 2]);
            if (dx <= r && dy <= r && dz <= r) {
                return true;
            }
        }
    }
    return false;
}

inline FrozenObjectCandidate RemeshCommittedRegion(
        int id,
        const core::Tensor& block_keys,
        const geometry::AxisAlignedBoundingBox& bounds,
        t::pipelines::slam::Model& model,
        const SegmentationConfig& config) {
    FrozenObjectCandidate updated;
    updated.id = id;
    updated.block_keys = block_keys;
    updated.bounds = bounds;
    if (IsEmptyBlockKeys(block_keys)) {
        return updated;
    }
    // Re-expand a 1-block halo so marching cubes inverse_index_map stays valid
    // even if the map grew after the original commit.
    const core::Tensor remesh_keys = ExpandBlockKeysForMeshExtract(block_keys);
    try {
        utility::LogInfo(
                "Region seam remesh start (id {}, blocks {} -> {}).", id,
                BlockKeyCount(block_keys), BlockKeyCount(remesh_keys));
        updated.mesh = model.ExtractTriangleMeshIncluding(
                config.mesh_weight_threshold, -1, remesh_keys);
    } catch (const std::exception&) {
        return updated;
    }
    updated.mesh = updated.mesh.To(core::Device("CPU:0"));
    if (!IsValidRegionMesh(updated.mesh)) {
        return updated;
    }
    geometry::TriangleMesh legacy = updated.mesh.ToLegacy();
    const double max_edge_length = std::min(
            static_cast<double>(config.voxel_size) * config.block_resolution *
                    config.max_triangle_edge_multiplier,
            std::max(0.05, bounds.GetMaxExtent() * 0.35));
    const double block_extent =
            static_cast<double>(config.voxel_size) * config.block_resolution;
    const double bounds_margin =
            std::max(static_cast<double>(config.voxel_size) *
                             config.trunc_multiplier,
                     block_extent);
    SanitizeRegionTriangleMesh(legacy, bounds, max_edge_length, bounds_margin,
                               // Seam remesh: allow slightly longer edges.
                               config.max_triangle_aspect_ratio, 1.20);
    if (!legacy.triangles_.empty()) {
        updated.mesh = t::geometry::TriangleMesh::FromLegacy(legacy);
    }
    return updated;
}

inline RegionProcessResult ProcessPendingRegionCandidates(
        std::vector<FrozenObjectCandidate>& pending,
        t::pipelines::slam::Model& model,
        ObjectFreezeTracker& tracker,
        const SegmentationConfig& config,
        bool session_stopping,
        const std::vector<FrozenObjectCandidate>& previously_committed =
                {}) {
    RegionProcessResult result;
    if (pending.empty()) {
        return result;
    }

    std::sort(pending.begin(), pending.end(),
              [&](const FrozenObjectCandidate& lhs,
                  const FrozenObjectCandidate& rhs) {
                  const double lhs_occ = ComputeClusterOccupancy(
                          lhs.source_cluster, config.voxel_size);
                  const double rhs_occ = ComputeClusterOccupancy(
                          rhs.source_cluster, config.voxel_size);
                  return lhs_occ > rhs_occ;
              });

    struct ScoredCandidate {
        FrozenObjectCandidate candidate;
        RegionReadinessReport readiness;
        RegionVoidReport void_report;
        bool valid_mesh = false;
    };
    std::vector<ScoredCandidate> scored;
    scored.reserve(pending.size());

    for (auto& candidate : pending) {
        ScoredCandidate item;
        item.candidate = std::move(candidate);

        // Skip CUDA mesh extract while camera is stationary (non-flush).
        // Gates used to run after extract, causing repeated illegal-access risk.
        if (!session_stopping && config.require_camera_motion &&
            !config.camera_moved_since_last_check) {
            tracker.MarkFreezeDeferredStationary(item.candidate.id);
            ++result.deferred_stationary;
            continue;
        }

        try {
            ApplyFreezeAndExtractMesh(item.candidate, model, config);
            item.valid_mesh = IsValidRegionMesh(item.candidate.mesh);
            if (item.valid_mesh) {
                const float weight_threshold =
                        RegionExtractWeightThreshold(item.candidate, config);
                item.readiness = ComputeRegionReadiness(
                        item.candidate, item.candidate.source_cluster, model,
                        weight_threshold, config);
                item.void_report = ComputeRegionVoidReport(
                        item.candidate, item.candidate.source_cluster, model,
                        weight_threshold, config);
                item.candidate.readiness = item.readiness;
                item.candidate.void_report = item.void_report;
            }
        } catch (const std::exception&) {
            item.valid_mesh = false;
        }
        scored.push_back(std::move(item));
    }

    std::sort(scored.begin(), scored.end(),
              [](const ScoredCandidate& lhs, const ScoredCandidate& rhs) {
                  return lhs.readiness.score > rhs.readiness.score;
              });

    for (auto& item : scored) {
        FrozenObjectCandidate& candidate = item.candidate;
        if (!item.valid_mesh) {
            tracker.MarkFreezeFailed(candidate.id);
            ++result.failed_empty;
            continue;
        }

        const int hole_defer_attempts =
                tracker.GetHoleDeferAttempts(candidate.id);
        const bool flush =
                tracker.ShouldFlushHoley(candidate.id, session_stopping);
        const float min_readiness =
                EffectiveMinReadiness(config, hole_defer_attempts, flush);

        if (!flush && config.require_camera_motion &&
            !config.camera_moved_since_last_check) {
            // Already counted in the pre-extract gate; keep as safety net.
            tracker.MarkFreezeDeferredStationary(candidate.id);
            ++result.deferred_stationary;
            continue;
        }

        if (!PassesVoidGate(item.void_report, config, flush)) {
            tracker.MarkFreezeDeferredVoid(candidate.id, item.void_report,
                                           item.readiness.score);
            ++result.deferred_void;
            continue;
        }

        if (config.defer_holey_regions &&
            item.readiness.score < min_readiness && !flush) {
            tracker.MarkFreezeDeferred(
                    candidate.id, item.readiness.score,
                    item.readiness.yield_ratio, item.readiness.occupancy,
                    item.readiness.boundary_ratio);
            ++result.deferred_holey;
            continue;
        }

        CommitRegionFreeze(candidate, model);
        if (flush && (config.defer_holey_regions ||
                      item.void_report.void_ratio > config.max_void_ratio)) {
            utility::LogInfo(
                    "Region candidate {} flushed on exit/stop: readiness={:.2f} "
                    "void={:.2f} -> committed.",
                    candidate.id, item.readiness.score,
                    item.void_report.void_ratio);
        }
        tracker.MarkFreezeCommitted(candidate.id);

        const int seam = ComputeRegionSeamBlockRadius(config);
        for (const auto& prev : previously_committed) {
            if (prev.id == candidate.id ||
                !IsValidRegionMesh(prev.mesh) ||
                IsEmptyBlockKeys(prev.block_keys)) {
                continue;
            }
            if (!BlocksNearEachOther(prev.block_keys, candidate.block_keys,
                                     seam)) {
                continue;
            }
            FrozenObjectCandidate seam_mesh = RemeshCommittedRegion(
                    prev.id, prev.block_keys, prev.bounds, model, config);
            if (IsValidRegionMesh(seam_mesh.mesh)) {
                utility::LogInfo(
                        "Updated region {} seam strip after neighbor {} "
                        "commit.",
                        prev.id, candidate.id);
                result.seam_updates.push_back(std::move(seam_mesh));
            }
        }

        result.committed.push_back(std::move(candidate));
    }

    if (result.deferred_holey > 0 || result.deferred_void > 0 ||
        result.deferred_stationary > 0 || !result.committed.empty()) {
        utility::LogInfo(
                "Region worker: committed {}, deferred {} holey / {} void / "
                "{} stationary, failed {} empty, seam updates {}.",
                static_cast<int>(result.committed.size()),
                result.deferred_holey, result.deferred_void,
                result.deferred_stationary, result.failed_empty,
                static_cast<int>(result.seam_updates.size()));
    }
    return result;
}

/// Keep points whose Euclidean distance to the camera origin
/// (T_frame_to_model translation) lies in [depth_min_m, depth_max_m].
inline t::geometry::PointCloud FilterPointCloudByCameraDistance(
        const t::geometry::PointCloud& pcd,
        const core::Tensor& T_frame_to_model,
        double depth_min_m,
        double depth_max_m) {
    if (!pcd.HasPointPositions() || T_frame_to_model.NumElements() == 0) {
        return pcd;
    }
    if (!(depth_max_m >= depth_min_m) || depth_min_m < 0.0) {
        return t::geometry::PointCloud(core::Device("CPU:0"));
    }

    t::geometry::PointCloud cpu = pcd.To(core::Device("CPU:0"));
    const Eigen::Matrix4d T =
            core::eigen_converter::TensorToEigenMatrixXd(T_frame_to_model);
    const Eigen::Vector3d cam = T.block<3, 1>(0, 3);
    const double min2 = depth_min_m * depth_min_m;
    const double max2 = depth_max_m * depth_max_m;

    core::Tensor positions = cpu.GetPointPositions().Contiguous();
    const int64_t n = positions.GetLength();
    std::vector<int64_t> keep;
    keep.reserve(static_cast<size_t>(n));

    if (positions.GetDtype() == core::Float32) {
        const float* ptr = positions.GetDataPtr<float>();
        for (int64_t i = 0; i < n; ++i) {
            const double dx = static_cast<double>(ptr[3 * i]) - cam.x();
            const double dy = static_cast<double>(ptr[3 * i + 1]) - cam.y();
            const double dz = static_cast<double>(ptr[3 * i + 2]) - cam.z();
            const double d2 = dx * dx + dy * dy + dz * dz;
            if (d2 >= min2 && d2 <= max2) {
                keep.push_back(i);
            }
        }
    } else if (positions.GetDtype() == core::Float64) {
        const double* ptr = positions.GetDataPtr<double>();
        for (int64_t i = 0; i < n; ++i) {
            const double dx = ptr[3 * i] - cam.x();
            const double dy = ptr[3 * i + 1] - cam.y();
            const double dz = ptr[3 * i + 2] - cam.z();
            const double d2 = dx * dx + dy * dy + dz * dz;
            if (d2 >= min2 && d2 <= max2) {
                keep.push_back(i);
            }
        }
    } else {
        utility::LogWarning(
                "FilterPointCloudByCameraDistance: unsupported dtype, "
                "returning unfiltered cloud.");
        return cpu;
    }

    if (keep.empty()) {
        return t::geometry::PointCloud(core::Device("CPU:0"));
    }
    core::Tensor indices(
            keep, {static_cast<int64_t>(keep.size())}, core::Int64,
            core::Device("CPU:0"));
    return cpu.SelectByIndex(indices);
}

/// Split an oversized cluster into AABB tiles of side tile_size_m.
/// Empty / undersized tiles are dropped. Logs tile counts.
inline std::vector<t::geometry::PointCloud> SplitClusterByExtentGrid(
        const t::geometry::PointCloud& cluster,
        double tile_size_m,
        int64_t min_cluster_points) {
    std::vector<t::geometry::PointCloud> tiles;
    if (!cluster.HasPointPositions() || tile_size_m <= 0.0) {
        return tiles;
    }

    t::geometry::PointCloud cpu = cluster.To(core::Device("CPU:0"));
    const geometry::AxisAlignedBoundingBox aabb =
            cpu.ToLegacy().GetAxisAlignedBoundingBox();
    const Eigen::Vector3d origin = aabb.min_bound_;
    const double extent = aabb.GetMaxExtent();

    core::Tensor positions = cpu.GetPointPositions().Contiguous();
    const int64_t n = positions.GetLength();
    std::unordered_map<int64_t, std::vector<int64_t>> buckets;

    auto pack_tile = [](int ix, int iy, int iz) -> int64_t {
        constexpr int64_t kMask = 0x1FFFFF;
        return (static_cast<int64_t>(ix) & kMask) |
               ((static_cast<int64_t>(iy) & kMask) << 21) |
               ((static_cast<int64_t>(iz) & kMask) << 42);
    };

    auto bucket_point = [&](double x, double y, double z, int64_t i) {
        const int ix =
                static_cast<int>(std::floor((x - origin.x()) / tile_size_m));
        const int iy =
                static_cast<int>(std::floor((y - origin.y()) / tile_size_m));
        const int iz =
                static_cast<int>(std::floor((z - origin.z()) / tile_size_m));
        buckets[pack_tile(ix, iy, iz)].push_back(i);
    };

    if (positions.GetDtype() == core::Float32) {
        const float* ptr = positions.GetDataPtr<float>();
        for (int64_t i = 0; i < n; ++i) {
            bucket_point(ptr[3 * i], ptr[3 * i + 1], ptr[3 * i + 2], i);
        }
    } else if (positions.GetDtype() == core::Float64) {
        const double* ptr = positions.GetDataPtr<double>();
        for (int64_t i = 0; i < n; ++i) {
            bucket_point(ptr[3 * i], ptr[3 * i + 1], ptr[3 * i + 2], i);
        }
    } else {
        utility::LogWarning(
                "SplitClusterByExtentGrid: unsupported dtype, skipping split.");
        return tiles;
    }

    tiles.reserve(buckets.size());
    for (auto& kv : buckets) {
        if (static_cast<int64_t>(kv.second.size()) < min_cluster_points) {
            continue;
        }
        core::Tensor indices(
                kv.second, {static_cast<int64_t>(kv.second.size())},
                core::Int64, core::Device("CPU:0"));
        tiles.push_back(cpu.SelectByIndex(indices));
    }

    utility::LogInfo(
            "Region cluster split: extent={:.2f} m -> {} tiles (kept {}).",
            extent, buckets.size(), tiles.size());
    return tiles;
}

inline std::vector<FrozenObjectCandidate> ProcessExtractedSurface(
        const t::geometry::PointCloud& surface_pcd,
        t::pipelines::slam::Model& model,
        ObjectFreezeTracker& tracker,
        const SegmentationConfig& config,
        int& next_object_id) {
    std::vector<FrozenObjectCandidate> frozen_now;
    if (!surface_pcd.HasPointPositions() ||
        surface_pcd.GetPointPositions().GetLength() <
                config.min_cluster_points) {
        return frozen_now;
    }

    t::geometry::PointCloud pcd_cpu = surface_pcd.To(core::Device("CPU:0"));
    core::Tensor labels =
            pcd_cpu.ClusterDBSCAN(config.dbscan_eps, config.dbscan_min_points,
                                  false);
    const int64_t num_points = labels.GetLength();
    int max_label = -1;
    const int32_t* label_data = labels.GetDataPtr<int32_t>();
    for (int64_t i = 0; i < num_points; ++i) {
        max_label = std::max(max_label, label_data[i]);
    }

    std::vector<ClusterSignature> signatures;
    std::vector<t::geometry::PointCloud> clusters;
    std::vector<ObjectType> types;
    signatures.reserve(static_cast<size_t>(max_label + 1));
    clusters.reserve(static_cast<size_t>(max_label + 1));
    types.reserve(static_cast<size_t>(max_label + 1));

    auto accept_cluster = [&](t::geometry::PointCloud cluster) {
        ObjectType type = ClassifyCluster(cluster);
        signatures.push_back(BuildClusterSignature(cluster, type));
        clusters.push_back(std::move(cluster));
        types.push_back(type);
    };

    for (int cluster_id = 0; cluster_id <= max_label; ++cluster_id) {
        t::geometry::PointCloud cluster =
                SelectCluster(pcd_cpu, labels, cluster_id);
        if (cluster.GetPointPositions().GetLength() <
            config.min_cluster_points) {
            continue;
        }
        const geometry::AxisAlignedBoundingBox cluster_aabb =
                cluster.ToLegacy().GetAxisAlignedBoundingBox();
        if (cluster_aabb.GetMaxExtent() > config.max_cluster_extent_m) {
            std::vector<t::geometry::PointCloud> parts =
                    SplitClusterByExtentGrid(cluster,
                                             config.max_cluster_extent_m,
                                             config.min_cluster_points);
            for (auto& part : parts) {
                accept_cluster(std::move(part));
            }
            continue;
        }
        accept_cluster(std::move(cluster));
    }

    auto build_candidate =
            [&](int id, ObjectType type,
                const ClusterSignature& signature) -> FrozenObjectCandidate {
        FrozenObjectCandidate candidate;
        candidate.id = id;
        candidate.type = type;
        candidate.signature = signature;

        t::geometry::PointCloud matched_cluster;
        for (size_t i = 0; i < signatures.size(); ++i) {
            if (signatures[i].Matches(signature, config.centroid_match_eps,
                                      config.extent_iou_min) &&
                types[i] == type) {
                matched_cluster = clusters[i];
                break;
            }
        }
        if (!matched_cluster.HasPointPositions()) {
            return candidate;
        }

        if (config.defer_model_ops) {
            candidate.source_cluster = std::move(matched_cluster);
            candidate.bounds =
                    candidate.source_cluster.GetAxisAlignedBoundingBox()
                            .ToLegacy();
            return candidate;
        }

        candidate.source_cluster = matched_cluster;
        ApplyFreezeAndExtractMesh(candidate, model, config);
        return candidate;
    };

    std::vector<FrozenObjectCandidate> ready =
            tracker.Update(signatures, next_object_id, build_candidate);
    for (auto& candidate : ready) {
        if (config.defer_model_ops) {
            if (candidate.source_cluster.HasPointPositions()) {
                frozen_now.push_back(std::move(candidate));
            }
        } else if (IsValidRegionMesh(candidate.mesh)) {
            if (config.require_camera_motion &&
                !config.camera_moved_since_last_check) {
                tracker.MarkFreezeDeferredStationary(candidate.id);
                continue;
            }
            const float weight_threshold =
                    RegionExtractWeightThreshold(candidate, config);
            candidate.readiness = ComputeRegionReadiness(
                    candidate, candidate.source_cluster, model,
                    weight_threshold, config);
            candidate.void_report = ComputeRegionVoidReport(
                    candidate, candidate.source_cluster, model,
                    weight_threshold, config);
            if (!PassesVoidGate(candidate.void_report, config, false)) {
                tracker.MarkFreezeDeferredVoid(candidate.id,
                                               candidate.void_report,
                                               candidate.readiness.score);
            } else if (!config.defer_holey_regions ||
                       candidate.readiness.score >=
                               config.min_region_readiness) {
                CommitRegionFreeze(candidate, model);
                tracker.MarkFreezeCommitted(candidate.id);
                frozen_now.push_back(std::move(candidate));
            } else {
                tracker.MarkFreezeDeferred(
                        candidate.id, candidate.readiness.score,
                        candidate.readiness.yield_ratio,
                        candidate.readiness.occupancy,
                        candidate.readiness.boundary_ratio);
            }
        } else {
            tracker.MarkFreezeFailed(candidate.id);
        }
    }
    return frozen_now;
}

inline t::geometry::PointCloud DownsamplePointCloudIfNeeded(
        const t::geometry::PointCloud& pcd, int64_t max_points,
        float voxel_size_hint = 0.0f) {
    if (!pcd.HasPointPositions()) {
        return pcd;
    }
    const int64_t count = pcd.GetPointPositions().GetLength();
    if (count <= max_points) {
        return pcd;
    }
    // Voxel downsample is deterministic and keeps cluster signatures stable
    // across region checks (RandomDownSample breaks DBSCAN tracking).
    float voxel = voxel_size_hint > 0.0f ? voxel_size_hint : 0.01f;
    t::geometry::PointCloud down = pcd.VoxelDownSample(voxel);
    int64_t down_count = down.GetPointPositions().GetLength();
    constexpr int kMaxVoxelIterations = 12;
    for (int i = 0; i < kMaxVoxelIterations && down_count > max_points; ++i) {
        voxel *= 1.25f;
        down = pcd.VoxelDownSample(voxel);
        down_count = down.GetPointPositions().GetLength();
    }
    if (down_count > max_points) {
        down = down.RandomDownSample(static_cast<double>(max_points) /
                                     static_cast<double>(down_count));
    }
    return down;
}

struct RegionParams {
    bool enabled = false;
    int min_points = 5000;
    int stability_frames = 5;
    int interval = 60;
    std::string output_dir = "regions";
    float min_readiness = 0.80f;
    int max_holey_defer = 40;
    bool defer_holey = true;
    bool flush_holey_on_exit = true;
    bool flush_when_only_holey_left = true;
    bool require_camera_motion = true;
    double min_motion_translation_m = 0.05;
    double min_motion_rotation_deg = 8.0;
    int min_hash_delta_blocks = 2;
    int region_motion_warmup_frames = 45;
    double max_void_ratio = 0.08;
    int max_void_blob_cells = 32;
    double max_boundary_void_ratio = 0.10;
    /// TSDF weight for region surface extract (display extract stays separate).
    float extract_weight = 3.0f;
    /// Camera-distance band for region input (meters from camera origin).
    double depth_min_m = 0.3;
    /// Cap for region depth band; callers may set min(2.5, slam depth_max).
    double depth_max_m = 2.5;
    /// Max AABB extent before grid-split (also tile side length).
    double max_cluster_extent_m = 1.2;
};

struct RegionRecord {
    int id = -1;
    ObjectType type = ObjectType::kGeneric;
    geometry::AxisAlignedBoundingBox bounds;
    int block_count = 0;
    int vertex_count = 0;
    int frame_id = 0;
    std::string timestamp;
    // Saved mesh basename, e.g. region_20260713_154530.ply
    std::string mesh_file;
};

inline SegmentationConfig BuildLiveRegionSegmentationConfig(
        float voxel_size,
        float trunc_multiplier,
        float mesh_weight_threshold,
        int min_cluster_points,
        int stability_frames,
        const RegionParams& region_params,
        double dbscan_eps_multiplier = 4.0) {
    SegmentationConfig config;
    config.auto_freeze = true;
    config.tsdf_mesh_only = true;
    config.defer_model_ops = true;
    config.min_cluster_points = min_cluster_points;
    config.stability_frames = stability_frames;
    config.voxel_size = voxel_size;
    config.trunc_multiplier = trunc_multiplier;
    config.mesh_weight_threshold = mesh_weight_threshold;
    config.dbscan_eps = dbscan_eps_multiplier * voxel_size;
    config.dbscan_min_points = std::max(10, min_cluster_points / 100);
    config.centroid_match_eps =
            std::max(0.25, 10.0 * static_cast<double>(voxel_size));
    config.extent_iou_min = 0.45;
    config.min_region_readiness = region_params.min_readiness;
    config.max_holey_defer_attempts = region_params.max_holey_defer;
    config.defer_holey_regions = region_params.defer_holey;
    config.flush_holey_on_exit = region_params.flush_holey_on_exit;
    config.flush_when_only_holey_left = region_params.flush_when_only_holey_left;
    config.require_camera_motion = region_params.require_camera_motion;
    config.min_motion_translation_m = region_params.min_motion_translation_m;
    config.min_motion_rotation_deg = region_params.min_motion_rotation_deg;
    config.min_hash_delta_blocks = region_params.min_hash_delta_blocks;
    config.region_motion_warmup_frames =
            region_params.region_motion_warmup_frames;
    config.max_void_ratio = region_params.max_void_ratio;
    config.max_void_blob_cells = region_params.max_void_blob_cells;
    config.max_boundary_void_ratio = region_params.max_boundary_void_ratio;
    config.max_cluster_extent_m = region_params.max_cluster_extent_m;
    return config;
}

inline void ClampVertexColors(geometry::TriangleMesh& mesh) {
    for (auto& c : mesh.vertex_colors_) {
        c = c.cwiseMax(Eigen::Vector3d::Zero())
                    .cwiseMin(Eigen::Vector3d::Ones());
    }
}

inline std::tm LocalTimeNow() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif
    return tm_buf;
}

inline std::string CurrentTimestampIso8601() {
    const std::tm tm_buf = LocalTimeNow();
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &tm_buf);
    return std::string(buffer);
}

// Compact local timestamp for region mesh filenames: YYYYMMDD_hhmmss
inline std::string CurrentTimestampCompact() {
    const std::tm tm_buf = LocalTimeNow();
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &tm_buf);
    return std::string(buffer);
}

// Build a unique region_YYYYMMDD_hhmmss[.N].ply name under output_dir.
inline std::string MakeRegionMeshFileName(const std::string& output_dir) {
    const std::string base = "region_" + CurrentTimestampCompact();
    std::string mesh_file = base + ".ply";
    std::string mesh_path = output_dir + "/" + mesh_file;
    int suffix = 1;
    while (utility::filesystem::FileExists(mesh_path)) {
        mesh_file = base + "_" + std::to_string(suffix++) + ".ply";
        mesh_path = output_dir + "/" + mesh_file;
    }
    return mesh_file;
}

inline void WriteRegionsJson(const std::string& output_dir,
                             const std::vector<RegionRecord>& records) {
    const std::string json_path = output_dir + "/regions.json";
    std::ofstream out(json_path, std::ios::binary);
    if (!out) {
        utility::LogWarning("Failed to open {} for writing.", json_path);
        return;
    }

    out << "{\n";
    out << "  \"region_count\": " << records.size() << ",\n";
    out << "  \"regions\": [\n";
    for (size_t i = 0; i < records.size(); ++i) {
        const auto& record = records[i];
        out << "    {\n";
        out << "      \"id\": " << record.id << ",\n";
        out << "      \"type\": \"" << ObjectTypeName(record.type) << "\",\n";
        out << "      \"frame_id\": " << record.frame_id << ",\n";
        out << "      \"timestamp\": \"" << record.timestamp << "\",\n";
        out << "      \"block_count\": " << record.block_count << ",\n";
        out << "      \"vertex_count\": " << record.vertex_count << ",\n";
        out << "      \"aabb\": {\n";
        out << "        \"min\": [" << record.bounds.min_bound_.x() << ", "
            << record.bounds.min_bound_.y() << ", "
            << record.bounds.min_bound_.z() << "],\n";
        out << "        \"max\": [" << record.bounds.max_bound_.x() << ", "
            << record.bounds.max_bound_.y() << ", "
            << record.bounds.max_bound_.z() << "]\n";
        out << "      },\n";
        const std::string mesh_file = record.mesh_file.empty()
                                              ? ("region_" +
                                                 std::to_string(record.id) +
                                                 ".ply")
                                              : record.mesh_file;
        out << "      \"mesh_file\": \"" << mesh_file << "\"\n";
        out << "    }";
        if (i + 1 < records.size()) {
            out << ",";
        }
        out << "\n";
    }
    out << "  ]\n";
    out << "}\n";
}

inline bool SaveFrozenRegion(const RegionParams& region_params,
                             const FrozenObjectCandidate& candidate,
                             int frame_id,
                             RegionRecord& record_out) {
    if (!IsValidRegionMesh(candidate.mesh)) {
        return false;
    }

    utility::filesystem::MakeDirectoryHierarchy(region_params.output_dir);

    const std::string mesh_file =
            MakeRegionMeshFileName(region_params.output_dir);
    const std::string mesh_path =
            region_params.output_dir + "/" + mesh_file;
    auto legacy_mesh =
            std::make_shared<geometry::TriangleMesh>(candidate.mesh.ToLegacy());
    ClampVertexColors(*legacy_mesh);
    if (!io::WriteTriangleMesh(mesh_path, *legacy_mesh)) {
        utility::LogWarning("Failed to save region mesh: {}", mesh_path);
        return false;
    }

    record_out.id = candidate.id;
    record_out.type = candidate.type;
    record_out.bounds = candidate.bounds;
    record_out.block_count = static_cast<int>(BlockKeyCount(candidate.block_keys));
    record_out.vertex_count =
            static_cast<int>(legacy_mesh->vertices_.size());
    record_out.frame_id = frame_id;
    record_out.timestamp = CurrentTimestampIso8601();
    record_out.mesh_file = mesh_file;

    utility::LogInfo(
            "Saved region {} ({}, {} blocks, {} vertices) -> {}",
            record_out.id, ObjectTypeName(record_out.type),
            record_out.block_count, record_out.vertex_count, mesh_path);
    return true;
}

}  // namespace object_mesh
}  // namespace examples
}  // namespace open3d
