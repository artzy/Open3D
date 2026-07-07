// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>
#include <unordered_map>
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
    int min_cluster_points = 5000;
    int stability_frames = 5;
    bool auto_freeze = true;
    float voxel_size = 3.0f / 512.0f;
    float trunc_multiplier = 8.0f;
    float mesh_weight_threshold = 3.0f;
    double centroid_match_eps = 0.15;
    double extent_iou_min = 0.7;
};

struct FrozenObjectCandidate {
    int id = -1;
    ObjectType type = ObjectType::kGeneric;
    t::geometry::TriangleMesh mesh;
    core::Tensor block_keys;
    geometry::AxisAlignedBoundingBox bounds;
    ClusterSignature signature;
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

private:
    SegmentationConfig config_;
    std::vector<TrackedCluster> tracked_clusters_;
};

inline core::Tensor CollectBlockKeys(
        t::geometry::VoxelBlockGrid& vbg,
        const t::geometry::PointCloud& cluster,
        float trunc_multiplier) {
    core::Tensor block_coords =
            vbg.GetUniqueBlockCoordinates(cluster, trunc_multiplier);
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

    const core::Device mesh_device =
            model.voxel_grid_.GetHashMap().GetDevice();

    t::geometry::PointCloud pcd_cpu = surface_pcd.To(core::Device("CPU:0"));
    core::Tensor labels =
            pcd_cpu.ClusterDBSCAN(config.dbscan_eps, config.min_cluster_points,
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

        candidate.block_keys = CollectBlockKeys(model.voxel_grid_,
                                                matched_cluster,
                                                config.trunc_multiplier);
        if (candidate.block_keys.NumElements() > 0) {
            model.FreezeBlocks(candidate.block_keys);
            if (type == ObjectType::kGeneric) {
                candidate.mesh = model.ExtractTriangleMeshIncluding(
                        config.mesh_weight_threshold, -1,
                        candidate.block_keys);
                if (!candidate.mesh.HasVertexPositions()) {
                    candidate.mesh = CreatePrimitiveMesh(
                            ObjectType::kBox, matched_cluster, mesh_device);
                }
            } else {
                candidate.mesh = CreatePrimitiveMesh(
                        type, matched_cluster, mesh_device);
            }
        }
        candidate.bounds = matched_cluster.GetAxisAlignedBoundingBox().ToLegacy();
        return candidate;
    };

    std::vector<FrozenObjectCandidate> ready =
            tracker.Update(signatures, next_object_id, build_candidate);
    for (auto& candidate : ready) {
        if (candidate.mesh.HasVertexPositions()) {
            frozen_now.push_back(std::move(candidate));
        }
    }
    return frozen_now;
}

}  // namespace object_mesh
}  // namespace examples
}  // namespace open3d
