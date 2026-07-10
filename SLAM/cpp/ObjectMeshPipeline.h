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
};

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
        double max_aspect_ratio) {
    RegionMeshSanitizeStats stats;
    if (mesh.vertices_.empty() || mesh.triangles_.empty()) {
        return stats;
    }
    stats.tris_before = static_cast<int>(mesh.triangles_.size());

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
            max_edge > max_edge_length * 0.25) {
            remove_mask[static_cast<size_t>(i)] = true;
            ++stats.removed_needle;
            continue;
        }
        if (e01 > max_edge_length || e12 > max_edge_length ||
            e20 > max_edge_length) {
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
};

struct TrackedCluster {
    ClusterSignature signature;
    int stable_frames = 0;
    bool frozen = false;
};

class ObjectFreezeTracker {
public:
    explicit ObjectFreezeTracker(const SegmentationConfig& config)
        : config_(config) {}

    void SetConfig(const SegmentationConfig& config) { config_ = config; }

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
        for (auto& tracked : tracked_clusters_) {
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
                tracked.signature = cluster_signatures[best_idx];
                ++tracked.stable_frames;
                if (tracked.stable_frames >= config_.stability_frames) {
                    tracked.frozen = true;
                    ready_to_freeze.push_back(
                            build_candidate(next_object_id++,
                                            tracked.signature.type,
                                            tracked.signature));
                }
            } else {
                tracked.stable_frames = 0;
            }
        }

        tracked_clusters_.erase(
                std::remove_if(tracked_clusters_.begin(),
                               tracked_clusters_.end(),
                               [](const TrackedCluster& cluster) {
                                   return !cluster.frozen &&
                                          cluster.stable_frames == 0;
                               }),
                tracked_clusters_.end());

        for (size_t i = 0; i < cluster_signatures.size(); ++i) {
            if (matched[i]) {
                continue;
            }
            TrackedCluster cluster;
            cluster.signature = cluster_signatures[i];
            cluster.stable_frames = 1;
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

/// Marching-cubes mesh extract needs a 1-block halo; partial block sets crash
/// CUDA when neighbor indices are missing from inverse_index_map.
inline core::Tensor ExpandBlockKeysForMeshExtract(
        const core::Tensor& block_keys) {
    if (block_keys.NumElements() == 0) {
        return block_keys;
    }
    core::Tensor keys_cpu =
            block_keys.To(core::Device("CPU:0")).Contiguous();
    const int32_t* data = keys_cpu.GetDataPtr<int32_t>();
    const int64_t n = keys_cpu.GetLength();
    std::unordered_set<BlockKey3, BlockKey3Hash> expanded;
    expanded.reserve(static_cast<size_t>(n) * 27);
    for (int64_t i = 0; i < n; ++i) {
        const int32_t x = data[i * 3 + 0];
        const int32_t y = data[i * 3 + 1];
        const int32_t z = data[i * 3 + 2];
        for (int32_t dx = -1; dx <= 1; ++dx) {
            for (int32_t dy = -1; dy <= 1; ++dy) {
                for (int32_t dz = -1; dz <= 1; ++dz) {
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
        return core::Tensor({}, core::Int32, core::Device("CPU:0"));
    }
    if (block_coords.GetLength() == 0) {
        return core::Tensor({}, core::Int32, core::Device("CPU:0"));
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
    const double bounds_margin =
            static_cast<double>(config.voxel_size) * config.trunc_multiplier;

    candidate.block_keys = ExpandBlockKeysForMeshExtract(
            CollectBlockKeys(model.voxel_grid_, cluster, config.trunc_multiplier));
    const core::Device mesh_device =
            model.voxel_grid_.GetHashMap().GetDevice();
    if (candidate.block_keys.NumElements() == 0) {
        candidate.bounds = cluster_bounds;
        return;
    }

    model.FreezeBlocks(candidate.block_keys);
    if (config.tsdf_mesh_only || candidate.type == ObjectType::kGeneric) {
        try {
            candidate.mesh = model.ExtractTriangleMeshIncluding(
                    config.mesh_weight_threshold, -1, candidate.block_keys);
        } catch (const std::exception&) {
            candidate.mesh = t::geometry::TriangleMesh(core::Device("CPU:0"));
        }
        candidate.mesh = candidate.mesh.To(core::Device("CPU:0"));
        if (candidate.mesh.HasVertexPositions() &&
            candidate.mesh.HasTriangleIndices()) {
            geometry::TriangleMesh legacy = candidate.mesh.ToLegacy();
            SanitizeRegionTriangleMesh(legacy, cluster_bounds, max_edge_length,
                                       bounds_margin,
                                       config.max_triangle_aspect_ratio);
            if (legacy.triangles_.empty()) {
                candidate.mesh = t::geometry::TriangleMesh(core::Device("CPU:0"));
            } else {
                candidate.mesh =
                        t::geometry::TriangleMesh::FromLegacy(legacy);
            }
        }
    } else {
        candidate.mesh =
                CreatePrimitiveMesh(candidate.type, cluster, mesh_device);
        candidate.mesh = candidate.mesh.To(core::Device("CPU:0"));
        if (candidate.mesh.HasVertexPositions() &&
            candidate.mesh.HasTriangleIndices()) {
            geometry::TriangleMesh legacy = candidate.mesh.ToLegacy();
            SanitizeRegionTriangleMesh(legacy, cluster_bounds, max_edge_length,
                                       bounds_margin,
                                       config.max_triangle_aspect_ratio);
            if (legacy.triangles_.empty()) {
                candidate.mesh = t::geometry::TriangleMesh(core::Device("CPU:0"));
            } else {
                candidate.mesh =
                        t::geometry::TriangleMesh::FromLegacy(legacy);
            }
        }
    }
    candidate.bounds = cluster_bounds;
    candidate.source_cluster = t::geometry::PointCloud();
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
            continue;
        }
        ObjectType type = ClassifyCluster(cluster);
        signatures.push_back(BuildClusterSignature(cluster, type));
        clusters.push_back(std::move(cluster));
        types.push_back(type);
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
        if (candidate.mesh.HasVertexPositions() ||
            candidate.source_cluster.HasPointPositions()) {
            frozen_now.push_back(std::move(candidate));
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
};

struct RegionRecord {
    int id = -1;
    ObjectType type = ObjectType::kGeneric;
    geometry::AxisAlignedBoundingBox bounds;
    int block_count = 0;
    int vertex_count = 0;
    int frame_id = 0;
    std::string timestamp;
};

inline SegmentationConfig BuildLiveRegionSegmentationConfig(
        float voxel_size,
        float trunc_multiplier,
        float mesh_weight_threshold,
        int min_cluster_points,
        int stability_frames,
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
    return config;
}

inline void ClampVertexColors(geometry::TriangleMesh& mesh) {
    for (auto& c : mesh.vertex_colors_) {
        c = c.cwiseMax(Eigen::Vector3d::Zero())
                    .cwiseMin(Eigen::Vector3d::Ones());
    }
}

inline std::string CurrentTimestampIso8601() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &tm_buf);
    return std::string(buffer);
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
        out << "      \"mesh_file\": \"region_" << record.id << ".ply\"\n";
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
    if (!candidate.mesh.HasVertexPositions()) {
        return false;
    }

    utility::filesystem::MakeDirectoryHierarchy(region_params.output_dir);

    const std::string mesh_path = region_params.output_dir + "/region_" +
                                  std::to_string(candidate.id) + ".ply";
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
    record_out.block_count =
            static_cast<int>(candidate.block_keys.NumElements() / 3);
    record_out.vertex_count =
            static_cast<int>(legacy_mesh->vertices_.size());
    record_out.frame_id = frame_id;
    record_out.timestamp = CurrentTimestampIso8601();

    utility::LogInfo(
            "Saved region {} ({}, {} blocks, {} vertices) -> {}",
            record_out.id, ObjectTypeName(record_out.type),
            record_out.block_count, record_out.vertex_count, mesh_path);
    return true;
}

}  // namespace object_mesh
}  // namespace examples
}  // namespace open3d
