// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include "open3d/t/pipelines/slam/Model.h"

#include <unordered_set>

#include "open3d/core/Tensor.h"
#include "open3d/t/geometry/Image.h"
#include "open3d/t/geometry/RGBDImage.h"
#include "open3d/t/geometry/Utility.h"
#include "open3d/t/geometry/VoxelBlockGrid.h"
#include "open3d/t/pipelines/odometry/RGBDOdometry.h"
#include "open3d/t/pipelines/slam/Frame.h"

namespace open3d {
namespace t {
namespace pipelines {
namespace slam {

namespace {

struct BlockKey {
    int32_t x;
    int32_t y;
    int32_t z;
    bool operator==(const BlockKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct BlockKeyHash {
    size_t operator()(const BlockKey& key) const {
        return (static_cast<size_t>(static_cast<uint32_t>(key.x)) * 73856093u) ^
               (static_cast<size_t>(static_cast<uint32_t>(key.y)) * 19349663u) ^
               (static_cast<size_t>(static_cast<uint32_t>(key.z)) * 83492791u);
    }
};

std::unordered_set<BlockKey, BlockKeyHash> BuildBlockKeySet(
        const core::Tensor& block_keys) {
    std::unordered_set<BlockKey, BlockKeyHash> key_set;
    if (block_keys.NumElements() == 0) {
        return key_set;
    }
    core::Tensor keys_cpu = block_keys.To(core::Device("CPU:0")).Contiguous();
    const int64_t n = keys_cpu.GetLength();
    const int32_t* data = keys_cpu.GetDataPtr<int32_t>();
    key_set.reserve(static_cast<size_t>(n));
    for (int64_t i = 0; i < n; ++i) {
        key_set.insert({data[i * 3 + 0], data[i * 3 + 1], data[i * 3 + 2]});
    }
    return key_set;
}

core::Tensor MergeBlockKeys(const core::Tensor& existing_keys,
                            const core::Tensor& new_keys) {
    std::unordered_set<BlockKey, BlockKeyHash> merged =
            BuildBlockKeySet(existing_keys);
    const std::unordered_set<BlockKey, BlockKeyHash> incoming =
            BuildBlockKeySet(new_keys);
    merged.insert(incoming.begin(), incoming.end());

    std::vector<int32_t> flat;
    flat.reserve(merged.size() * 3);
    for (const auto& key : merged) {
        flat.push_back(key.x);
        flat.push_back(key.y);
        flat.push_back(key.z);
    }
    return core::Tensor(flat,
                        {static_cast<int64_t>(merged.size()), 3},
                        core::Int32, core::Device("CPU:0"));
}

}  // namespace

Model::Model(float voxel_size,
             int block_resolution,
             int est_block_count,
             const core::Tensor& T_init,
             const core::Device& device)
    : voxel_grid_(std::vector<std::string>({"tsdf", "weight", "color"}),
                  std::vector<core::Dtype>(
                          {core::Float32, core::UInt16, core::UInt16}),
                  std::vector<core::SizeVector>({{1}, {1}, {3}}),
                  voxel_size,
                  block_resolution,
                  est_block_count,
                  device),
      T_frame_to_world_(T_init.To(core::Device("CPU:0"))) {}

void Model::SynthesizeModelFrame(Frame& raycast_frame,
                                 float depth_scale,
                                 float depth_min,
                                 float depth_max,
                                 float trunc_voxel_multiplier,
                                 bool enable_color,
                                 float weight_threshold) {
    if (weight_threshold < 0) {
        weight_threshold = std::min(frame_id_ * 1.0f, 3.0f);
    }

    auto result = voxel_grid_.RayCast(
            frustum_block_coords_, raycast_frame.GetIntrinsics(),
            t::geometry::InverseTransformation(GetCurrentFramePose()),
            raycast_frame.GetWidth(), raycast_frame.GetHeight(),
            {"depth", "color"}, depth_scale, depth_min, depth_max,
            weight_threshold, trunc_voxel_multiplier);
    raycast_frame.SetData("depth", result["depth"]);

    if (enable_color) {
        raycast_frame.SetData("color", result["color"]);
    } else if (raycast_frame.GetData("color").NumElements() == 0) {
        // Put a dummy RGB frame to enable RGBD odometry in TrackFrameToModel
        raycast_frame.SetData("color",
                              core::Tensor({raycast_frame.GetHeight(),
                                            raycast_frame.GetWidth(), 3},
                                           core::Float32, core::Device()));
    }
}

odometry::OdometryResult Model::TrackFrameToModel(
        const Frame& input_frame,
        const Frame& raycast_frame,
        float depth_scale,
        float depth_max,
        float depth_diff,
        const odometry::Method method,
        const std::vector<odometry::OdometryConvergenceCriteria>& criteria) {
    // TODO: Expose init_source_to_target as param, and make the input sequence
    // consistent with RGBDOdometryMultiScale.
    const static core::Tensor init_source_to_target =
            core::Tensor::Eye(4, core::Float64, core::Device("CPU:0"));

    return odometry::RGBDOdometryMultiScale(
            t::geometry::RGBDImage(input_frame.GetDataAsImage("color"),
                                   input_frame.GetDataAsImage("depth")),
            t::geometry::RGBDImage(raycast_frame.GetDataAsImage("color"),
                                   raycast_frame.GetDataAsImage("depth")),
            raycast_frame.GetIntrinsics(), init_source_to_target, depth_scale,
            depth_max, criteria, method,
            odometry::OdometryLossParams(depth_diff));
}

void Model::Integrate(const Frame& input_frame,
                      float depth_scale,
                      float depth_max,
                      float trunc_voxel_multiplier) {
    t::geometry::Image depth = input_frame.GetDataAsImage("depth");
    t::geometry::Image color = input_frame.GetDataAsImage("color");
    core::Tensor intrinsic = input_frame.GetIntrinsics();
    core::Tensor extrinsic =
            t::geometry::InverseTransformation(GetCurrentFramePose());
    frustum_block_coords_ = voxel_grid_.GetUniqueBlockCoordinates(
            depth, intrinsic, extrinsic, depth_scale, depth_max,
            trunc_voxel_multiplier);
    voxel_grid_.Integrate(frustum_block_coords_, depth, color, intrinsic,
                          extrinsic, depth_scale, depth_max,
                          trunc_voxel_multiplier);
}

t::geometry::PointCloud Model::ExtractPointCloud(float weight_threshold,
                                                 int estimated_number) {
    return voxel_grid_.ExtractPointCloud(weight_threshold, estimated_number);
}

t::geometry::TriangleMesh Model::ExtractTriangleMesh(float weight_threshold,
                                                     int estimated_number) {
    return voxel_grid_.ExtractTriangleMesh(weight_threshold, estimated_number);
}

void Model::FreezeBlocks(const core::Tensor& block_keys) {
    if (block_keys.NumElements() == 0) {
        return;
    }
    frozen_block_keys_ = MergeBlockKeys(frozen_block_keys_, block_keys);
}

core::Tensor Model::GetFrozenBlockKeys() const { return frozen_block_keys_; }

t::geometry::PointCloud Model::ExtractPointCloudExcludingFrozen(
        float weight_threshold,
        int estimated_number) {
    if (frozen_block_keys_.NumElements() == 0) {
        return ExtractPointCloud(weight_threshold, estimated_number);
    }
    return voxel_grid_.ExtractPointCloudExcluding(weight_threshold,
                                                  estimated_number,
                                                  frozen_block_keys_);
}

t::geometry::TriangleMesh Model::ExtractTriangleMeshIncluding(
        float weight_threshold,
        int estimated_number,
        const core::Tensor& include_block_keys) {
    return voxel_grid_.ExtractTriangleMeshIncluding(weight_threshold,
                                                    estimated_number,
                                                    include_block_keys);
}

core::HashMap Model::GetHashMap() { return voxel_grid_.GetHashMap(); }

}  // namespace slam
}  // namespace pipelines
}  // namespace t
}  // namespace open3d
