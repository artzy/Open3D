// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------
//
// Real-time RGB-D SLAM with Intel RealSense (live camera or .bag playback).
//
// Combines OfflineSLAM-style frame-to-model tracking with RealSense capture.
// Optimized defaults for D415 (640x480, RS400 HIGH_ACCURACY preset).
//
// Build: BUILD_LIBREALSENSE=ON, BUILD_CUDA_MODULE=ON (recommended)
//
// Examples:
//   RealTimeSLAMRealSense -l
//   RealTimeSLAMRealSense -c examples/test_data/rs_d415_slam.json
//   RealTimeSLAMRealSense -c rs_d415_slam.json --device CUDA:0 --profile medium
//   RealTimeSLAMRealSense --use_bag_file capture.bag --profile low
//

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <functional>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include "ObjectMeshPipeline.h"
#include "RealTimeSLAMUtil.h"
#include "open3d/Open3D.h"

namespace {

using namespace open3d;
namespace tio = open3d::t::io;
namespace object_mesh = open3d::examples::object_mesh;
namespace realtime_slam = open3d::examples::realtime_slam;
using DisplayState = realtime_slam::RealTimeSLAMWindow::DisplayState;

struct RegionParams {
    bool enabled = false;
    int min_points = 5000;
    int stability_frames = 5;
    int interval = 60;
    std::string output_dir = "regions";
};

struct RegionRecord {
    int id = -1;
    object_mesh::ObjectType type = object_mesh::ObjectType::kGeneric;
    geometry::AxisAlignedBoundingBox bounds;
    int block_count = 0;
    int vertex_count = 0;
    int frame_id = 0;
    std::string timestamp;
};

struct SlamParams {
    float voxel_size = 3.f / 512.f;
    float trunc_multiplier = 8.f;
    int block_count = 40000;
    int estimated_points = 6000000;
    float depth_max = 5.f;
    float depth_diff = 0.07f;
    float depth_scale = 1000.f;
    int update_interval = 15;
    int odom_iter_coarse = 6;
    int odom_iter_mid = 3;
    int odom_iter_fine = 2;
    RegionParams regions;
};

SlamParams GetProfile(const std::string& profile) {
    SlamParams p;
    if (profile == "low") {
        p.voxel_size = 0.008f;
        p.block_count = 16384;
        p.estimated_points = 2500000;
        p.depth_max = 2.f;
        p.update_interval = 20;
        return p;
    }
    if (profile == "high") {
        p.block_count = 50000;
        p.estimated_points = 10000000;
        p.depth_max = 8.f;
        p.update_interval = 10;
        p.odom_iter_coarse = 8;
        p.odom_iter_mid = 4;
        p.odom_iter_fine = 2;
        return p;
    }
    return p;
}

void PrintHelp() {
    PrintOpen3DVersion();
    utility::LogInfo("Usage:");
    utility::LogInfo("    > RealTimeSLAMRealSense [options]");
    utility::LogInfo("");
    utility::LogInfo("RealSense input:");
    utility::LogInfo("    [-l|--list-devices]     List connected RealSense devices.");
    utility::LogInfo("    [-c|--config PATH]       RealSense JSON config.");
    utility::LogInfo("                             Default: examples/test_data/rs_d415_slam.json");
    utility::LogInfo("    [--use_bag_file PATH]   Play back a RealSense .bag file.");
    utility::LogInfo("    [--record PATH.bag]     Record live capture to bag (optional).");
    utility::LogInfo("    [--align]               Align depth to color (default).");
    utility::LogInfo("    [--no-align]            Skip alignment (~33 ms saved at 30 fps).");
    utility::LogInfo("                             Use --no-align if RealSense CUDA align crashes.");
    utility::LogInfo("");
    utility::LogInfo("SLAM:");
    utility::LogInfo("    [--device CUDA:0|CPU:0] Compute device (default: CUDA:0).");
    utility::LogInfo("    [--profile low|medium|high]  Memory/quality preset (default: medium).");
    utility::LogInfo("    [--update_interval N]   Refresh 3D view every N frames (default: profile).");
    utility::LogInfo("    [--depth_max M]         Depth truncation in meters (default: profile).");
    utility::LogInfo("    [--voxel_size M]        TSDF voxel size in meters.");
    utility::LogInfo("    [--estimated_points N]  Extract buffer size (auto-scales with hash).");
    utility::LogInfo("    [--block_count N]       TSDF hash block capacity (default: profile).");
    utility::LogInfo("");
    utility::LogInfo("Region freeze (--regions):");
    utility::LogInfo("    [--regions]             Detect stable scan regions, freeze TSDF blocks,");
    utility::LogInfo("                             extract triangle meshes, and save to --region_dir.");
    utility::LogInfo("    [--region_min_points N] Min cluster points (default: 5000).");
    utility::LogInfo("    [--region_stability N]  Stable frames before freeze (default: 5).");
    utility::LogInfo("    [--region_interval N]   Region check every N frames (default: 60).");
    utility::LogInfo("    [--region_dir PATH]     Output directory (default: regions).");
    utility::LogInfo("");
    utility::LogInfo("GUI controls (left panel):");
    utility::LogInfo("    Cloud capture ON/OFF    Pause/resume RGB-D capture and SLAM integration.");
    utility::LogInfo("    Polygon ON/OFF          Show/hide frozen region triangle meshes.");
    utility::LogInfo("    Point cloud ON/OFF      Show/hide live scan + frozen region point clouds.");
    utility::LogInfo("");
    utility::LogInfo("Indoor room scan tips:");
    utility::LogInfo("    Walk slowly (~0.3 m/s), keep 30%% overlap between views.");
    utility::LogInfo("    Use --profile high --depth_max 8 for large rooms.");
    utility::LogInfo("    scene_mesh.ply is the watertight mesh; scene.ply is points.");
    utility::LogInfo("");
    utility::LogInfo("Output (on window close / ESC):");
    utility::LogInfo("    scene.ply        Reconstructed point cloud");
    utility::LogInfo("    scene_mesh.ply   Reconstructed triangle mesh");
    utility::LogInfo("    trajectory.log   Camera trajectory (strong tracking only)");
    utility::LogInfo("    regions/         Frozen region meshes (--regions only)");
    utility::LogInfo("");
    utility::LogInfo("Note: sudden fast motion may pause integration; move slowly");
    utility::LogInfo("      back to the scanned area to resume reconstruction.");
    utility::LogInfo("");
}


std::string DefaultConfigPath() {
    const std::vector<std::string> candidates = {
            "examples/test_data/rs_d415_slam.json",
            "../examples/test_data/rs_d415_slam.json",
            "../../examples/test_data/rs_d415_slam.json",
            "../../../examples/test_data/rs_d415_slam.json",
            "../../../../examples/test_data/rs_d415_slam.json"};
    for (const auto& path : candidates) {
        if (utility::filesystem::FileExists(path)) {
            return path;
        }
    }
    return "";
}

// Tracking tiers: strict pose+integrate vs reject outlier jumps.
// Be conservative here: accepting one bad pose integrates the same surface into
// a second location, which looks like a duplicated object in the TSDF map.
static constexpr double kPoseFitnessMin = 0.15;
static constexpr double kPoseTranslationMax = 0.12;
static constexpr double kPoseRotationMaxDeg = 10.0;
static constexpr double kWeakFitnessMin = 0.08;
static constexpr double kWeakTranslationMax = 0.25;
static constexpr double kWeakRotationMaxDeg = 20.0;
static constexpr double kOutlierTranslation = 0.40;
static constexpr double kOutlierRotationDeg = 30.0;
static constexpr float kOdometryHuberDelta = 0.05f;
static constexpr int kLostTrackingThreshold = 5;
static constexpr int kStrongStreakAfterLost = 5;
static constexpr double kRecoveryFitnessMin = 0.20;
static constexpr double kRecoveryTranslationMax = 0.06;
static constexpr double kRecoveryRotationMaxDeg = 5.0;
static constexpr double kF2FBridgeFitnessMin = 0.20;
static constexpr double kF2FBridgeTranslationMax = 0.18;
static constexpr double kF2FBridgeRotationMaxDeg = 20.0;
static constexpr int kModelRetryIntervalWhenLost = 10;
// Skip integrate/tracking when hash is nearly full (avoids rehash OOM + bad odometry).
static constexpr double kHashIntegrateFillRatio = 0.92;
static constexpr int kMaxEmptyCaptureRetries = 30;

bool IsHashNearFull(int64_t hash_size, int64_t hash_capacity) {
    return hash_capacity > 0 &&
           hash_size >
                   static_cast<int64_t>(hash_capacity * kHashIntegrateFillRatio);
}

static constexpr int kMaxExtractPoints = 12000000;
static constexpr int kPointsPerHashBlock = 200;
static constexpr int kMaxRegionSegmentationPoints = 200000;
static constexpr double kDbscanEpsMultiplier = 2.0;

int GetExtractPointBudget(int estimated_points, int64_t hash_size) {
    const int64_t hash_based = std::min(
            hash_size * static_cast<int64_t>(kPointsPerHashBlock),
            static_cast<int64_t>(kMaxExtractPoints));
    return static_cast<int>(std::max(static_cast<int64_t>(estimated_points),
                                     hash_based));
}

void ClampVertexColors(geometry::TriangleMesh& mesh) {
    for (auto& c : mesh.vertex_colors_) {
        c = c.cwiseMax(Eigen::Vector3d::Zero())
                    .cwiseMin(Eigen::Vector3d::Ones());
    }
}

void ClampPointColors(geometry::PointCloud& pcd) {
    for (auto& c : pcd.colors_) {
        c = c.cwiseMax(Eigen::Vector3d::Zero())
                    .cwiseMin(Eigen::Vector3d::Ones());
    }
}

bool IsOdometrySingularError(const std::exception& e) {
    const std::string msg = e.what();
    return msg.find("Singular 6x6") != std::string::npos ||
           msg.find("singular condition") != std::string::npos;
}

enum class TrackingTier { kInit, kStrong, kWeak, kOutlier, kFail };

float SafeOdometryDepthDiff(float depth_diff) {
    return std::max(kOdometryHuberDelta + 0.001f, depth_diff);
}

double TranslationNorm(const core::Tensor& transformation) {
    core::Tensor translation =
            transformation.Slice(0, 0, 3).Slice(1, 3, 4);
    return std::sqrt(
            (translation * translation).Sum({0, 1}).Item<double>());
}

double RotationAngleDeg(const core::Tensor& transformation) {
    Eigen::Matrix4d mat =
            core::eigen_converter::TensorToEigenMatrixXd(transformation);
    const double trace = mat.block<3, 3>(0, 0).trace();
    const double cos_angle = std::max(-1.0, std::min(1.0, (trace - 1.0) * 0.5));
    return std::acos(cos_angle) * 180.0 / 3.14159265358979323846;
}

TrackingTier ClassifyTracking(double fitness,
                              double translation,
                              double rotation_deg) {
    if (translation >= kOutlierTranslation ||
        rotation_deg >= kOutlierRotationDeg) {
        return TrackingTier::kOutlier;
    }
    if (fitness >= kPoseFitnessMin && translation < kPoseTranslationMax &&
        rotation_deg < kPoseRotationMaxDeg) {
        return TrackingTier::kStrong;
    }
    if (fitness >= kWeakFitnessMin && translation < kWeakTranslationMax &&
        rotation_deg < kWeakRotationMaxDeg) {
        return TrackingTier::kWeak;
    }
    return TrackingTier::kFail;
}

bool IsRecoveryPoseStable(double fitness,
                          double translation,
                          double rotation_deg) {
    return fitness >= kRecoveryFitnessMin &&
           translation < kRecoveryTranslationMax &&
           rotation_deg < kRecoveryRotationMaxDeg;
}

bool IsFrameToFrameBridgeStable(double fitness,
                                double translation,
                                double rotation_deg) {
    return fitness >= kF2FBridgeFitnessMin &&
           translation < kF2FBridgeTranslationMax &&
           rotation_deg < kF2FBridgeRotationMaxDeg;
}

std::shared_ptr<geometry::LineSet> CreateCameraMarker(
        int width,
        int height,
        const Eigen::Matrix3d& intrinsic,
        const core::Tensor& T_frame_to_model,
        double scale = 0.25) {
    const Eigen::Matrix4d camera_to_world =
            core::eigen_converter::TensorToEigenMatrixXd(T_frame_to_model);
    auto marker = geometry::LineSet::CreateCameraVisualization(
            width, height, intrinsic, camera_to_world.inverse(), scale);
    // Tango orange: distinct from the live point cloud and easy to relocate.
    marker->PaintUniformColor(Eigen::Vector3d(0.961, 0.475, 0.000));
    return marker;
}

std::string BuildPoseDiffText(const core::Tensor& stable_T_frame_to_model,
                              const core::Tensor& current_T_frame_to_model) {
    const Eigen::Matrix4d stable =
            core::eigen_converter::TensorToEigenMatrixXd(
                    stable_T_frame_to_model);
    const Eigen::Matrix4d current =
            core::eigen_converter::TensorToEigenMatrixXd(
                    current_T_frame_to_model);
    const Eigen::Matrix4d diff = stable.inverse() * current;
    const Eigen::Vector3d delta = diff.block<3, 1>(0, 3);
    const Eigen::Vector3d euler_deg =
            diff.block<3, 3>(0, 0).eulerAngles(0, 1, 2) * 180.0 /
            3.14159265358979323846;
    const double trace = diff.block<3, 3>(0, 0).trace();
    const double cos_angle = std::max(-1.0, std::min(1.0, (trace - 1.0) * 0.5));
    const double angle_deg =
            std::acos(cos_angle) * 180.0 / 3.14159265358979323846;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "Return toward orange camera\n";
    ss << "dX " << delta.x() << " m\n";
    ss << "dY " << delta.y() << " m\n";
    ss << "dZ " << delta.z() << " m\n";
    ss << "dist " << delta.norm() << " m\n";
    ss << std::setprecision(1);
    ss << "rot " << angle_deg << " deg\n";
    ss << "rX " << euler_deg.x() << " deg\n";
    ss << "rY " << euler_deg.y() << " deg\n";
    ss << "rZ " << euler_deg.z() << " deg";
    return ss.str();
}

const char* TrackingTierName(TrackingTier tier) {
    switch (tier) {
        case TrackingTier::kInit:
            return "init";
        case TrackingTier::kStrong:
            return "strong";
        case TrackingTier::kWeak:
            return "weak";
        case TrackingTier::kOutlier:
            return "outlier";
        case TrackingTier::kFail:
            return "fail";
    }
    return "unknown";
}

// TSDF voxel weight grows per integration; threshold 3.0 yields zero points on
// early frames and leaves the visualizer with an empty bounding box.
float ExtractWeightThreshold(int frame_id) {
    return std::max(1.0f, std::min(static_cast<float>(frame_id), 3.0f));
}

bool ShouldRefreshDisplay(int frame_id, int update_interval) {
    return frame_id == 0 || frame_id == 3 ||
           (frame_id > 0 && frame_id % update_interval == 0);
}

bool ShouldCheckRegions(int frame_id, int region_interval) {
    return frame_id > 0 && frame_id % region_interval == 0;
}

std::string CurrentTimestampIso8601() {
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

t::geometry::PointCloud DownsamplePointCloudIfNeeded(
        const t::geometry::PointCloud& pcd, int64_t max_points) {
    if (!pcd.HasPointPositions()) {
        return pcd;
    }
    const int64_t count = pcd.GetPointPositions().GetLength();
    if (count <= max_points) {
        return pcd;
    }
    return pcd.RandomDownSample(static_cast<double>(max_points) /
                                static_cast<double>(count));
}

object_mesh::SegmentationConfig BuildRegionSegmentationConfig(
        const SlamParams& params, float extract_weight) {
    object_mesh::SegmentationConfig config;
    config.auto_freeze = true;
    config.tsdf_mesh_only = true;
    config.defer_model_ops = true;
    config.min_cluster_points = params.regions.min_points;
    config.stability_frames = params.regions.stability_frames;
    config.voxel_size = params.voxel_size;
    config.trunc_multiplier = params.trunc_multiplier;
    config.mesh_weight_threshold = extract_weight;
    config.dbscan_eps = kDbscanEpsMultiplier * params.voxel_size;
    return config;
}

struct SlamRuntime {
    std::mutex model_mutex;
    t::pipelines::slam::Model* model = nullptr;
    std::atomic<bool> model_ready{false};

    std::mutex region_mutex;
    std::condition_variable region_cv;
    std::atomic<bool> region_requested{false};
    std::atomic<bool> region_stop{false};
    t::geometry::PointCloud pending_region_pcd;
    float pending_extract_weight = 3.0f;
    int pending_frame_id = 0;

    object_mesh::ObjectFreezeTracker freeze_tracker{
            object_mesh::SegmentationConfig{}};
    int next_object_id = 0;
    std::mutex records_mutex;
    std::vector<RegionRecord> region_records;
};

void WriteRegionsJson(const std::string& output_dir,
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
        out << "      \"type\": \"" << object_mesh::ObjectTypeName(record.type)
            << "\",\n";
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
    out.close();
}

bool SaveFrozenRegion(const RegionParams& region_params,
                      const object_mesh::FrozenObjectCandidate& candidate,
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
            record_out.id, object_mesh::ObjectTypeName(record_out.type),
            record_out.block_count, record_out.vertex_count, mesh_path);
    return true;
}

void RegionWorker(SlamRuntime& runtime,
                  const SlamParams& params,
                  DisplayState& state) {
    while (!runtime.region_stop.load() && !state.request_stop.load()) {
        t::geometry::PointCloud surface_pcd;
        float extract_weight = 3.0f;
        int frame_id = 0;
        {
            std::unique_lock<std::mutex> lock(runtime.region_mutex);
            runtime.region_cv.wait(lock, [&]() {
                return runtime.region_requested.load() ||
                       runtime.region_stop.load() || state.request_stop.load();
            });
            if (runtime.region_stop.load() || state.request_stop.load()) {
                break;
            }
            runtime.region_requested.store(false);
            surface_pcd = std::move(runtime.pending_region_pcd);
            extract_weight = runtime.pending_extract_weight;
            frame_id = runtime.pending_frame_id;
        }

        if (!runtime.model_ready.load() || !runtime.model ||
            surface_pcd.IsEmpty()) {
            continue;
        }

        try {
            surface_pcd = DownsamplePointCloudIfNeeded(
                    surface_pcd, kMaxRegionSegmentationPoints);
            object_mesh::SegmentationConfig config =
                    BuildRegionSegmentationConfig(params, extract_weight);
            runtime.freeze_tracker.SetConfig(config);

            std::vector<object_mesh::FrozenObjectCandidate> pending;
            pending = object_mesh::ProcessExtractedSurface(
                    surface_pcd, *runtime.model, runtime.freeze_tracker,
                    config, runtime.next_object_id);

            std::vector<object_mesh::FrozenObjectCandidate> frozen_now;
            {
                std::lock_guard<std::mutex> model_lock(runtime.model_mutex);
                for (auto& candidate : pending) {
                    const t::geometry::PointCloud region_cluster =
                            candidate.source_cluster;
                    object_mesh::ApplyFreezeAndExtractMesh(
                            candidate, *runtime.model, config);
                    if (candidate.mesh.HasVertexPositions()) {
                        candidate.source_cluster = region_cluster;
                        frozen_now.push_back(std::move(candidate));
                    }
                }
            }

            if (frozen_now.empty()) {
                continue;
            }

            int total_regions = 0;
            {
                std::lock_guard<std::mutex> lock(runtime.records_mutex);
                for (auto& candidate : frozen_now) {
                    RegionRecord record;
                    if (!SaveFrozenRegion(params.regions, candidate, frame_id,
                                          record)) {
                        continue;
                    }
                    runtime.region_records.push_back(record);

                    DisplayState::RegionPair pair;
                    pair.id = record.id;
                    pair.mesh = std::make_shared<geometry::TriangleMesh>(
                            candidate.mesh.ToLegacy());
                    ClampVertexColors(*pair.mesh);
                    if (candidate.source_cluster.HasPointPositions()) {
                        pair.pcd = std::make_shared<geometry::PointCloud>(
                                candidate.source_cluster.ToLegacy());
                        ClampPointColors(*pair.pcd);
                    }
                    state.PushRegionPair(std::move(pair));
                }
                WriteRegionsJson(params.regions.output_dir,
                                 runtime.region_records);
                total_regions = static_cast<int>(runtime.region_records.size());
            }
            state.SetRegionCount(total_regions);
            utility::LogInfo("Frozen {} region(s). Total regions: {}.",
                             frozen_now.size(), total_regions);
        } catch (const std::exception& e) {
            utility::LogWarning("Region worker failed at frame {}: {}",
                                frame_id, e.what());
        }
    }
}

void SlamWorker(std::function<t::geometry::RGBDImage()> capture_frame,
                const core::Tensor& intrinsic,
                const camera::PinholeCameraIntrinsic& cam_intrinsic,
                const core::Device& device,
                SlamParams params,
                SlamRuntime& runtime,
                DisplayState& state) {
    using t::pipelines::slam::Frame;
    using t::pipelines::slam::Model;

    t::geometry::RGBDImage first;
    for (int attempt = 0; attempt < kMaxEmptyCaptureRetries; ++attempt) {
        first = capture_frame();
        if (!first.IsEmpty()) {
            if (attempt > 0) {
                utility::LogInfo(
                        "First RGB-D frame captured after {} warmup attempt(s).",
                        attempt);
            }
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (first.IsEmpty()) {
        utility::LogWarning(
                "Failed to capture the first RGB-D frame after {} attempts. "
                "Check that the camera is connected and not in use by another "
                "application.",
                kMaxEmptyCaptureRetries);
        state.slam_finished.store(true);
        return;
    }

    const int height = first.depth_.GetRows();
    const int width = first.depth_.GetCols();

    core::Tensor T_frame_to_model =
            core::Tensor::Eye(4, core::Dtype::Float64, core::Device("CPU:0"));
    const int init_hash_capacity =
            std::min(params.block_count, 40000);
    Model model(params.voxel_size, 16, init_hash_capacity, T_frame_to_model,
                device);
    runtime.model = &model;
    runtime.model_ready.store(true);
    if (params.regions.enabled) {
        utility::filesystem::MakeDirectoryHierarchy(params.regions.output_dir);
        utility::LogInfo(
                "Region freeze enabled: min_points={}, stability={}, "
                "interval={}, dir={}",
                params.regions.min_points, params.regions.stability_frames,
                params.regions.interval, params.regions.output_dir);
    }
    if (params.block_count > init_hash_capacity) {
        try {
            model.GetHashMap().Reserve(params.block_count);
            utility::LogInfo("Voxel hash capacity: {} blocks.",
                             model.GetHashMap().GetCapacity());
        } catch (const std::exception& e) {
            utility::LogWarning(
                    "Could not reserve {} hash blocks ({}). Using {}.",
                    params.block_count, e.what(),
                    model.GetHashMap().GetCapacity());
        }
    }

    Frame input_frame(height, width, intrinsic, device);
    Frame raycast_frame(height, width, intrinsic, device);

    auto trajectory = std::make_shared<camera::PinholeCameraTrajectory>();
    const std::vector<t::pipelines::odometry::OdometryConvergenceCriteria>
            odom_criteria = {params.odom_iter_coarse, params.odom_iter_mid,
                             params.odom_iter_fine};
    const core::Tensor identity_pose =
            core::Tensor::Eye(4, core::Dtype::Float64, core::Device("CPU:0"));
    const Eigen::Matrix3d intrinsic_eigen =
            core::eigen_converter::TensorToEigenMatrixXd(intrinsic);
    core::Tensor last_stable_T_frame_to_model =
            T_frame_to_model.Contiguous();
    bool lost_camera_marker_visible = false;

    t::geometry::RGBDImage prev_rgbd;
    int consecutive_tracking_failures = 0;
    int consecutive_strong = 0;
    int consecutive_f2f_bridges = 0;
    int rejected_pose_updates = 0;
    int f2f_bridge_successes = 0;
    int integrated_frames = 0;
    bool tracking_was_unstable = false;
    int empty_capture_retries = 0;
    bool hash_full_warned = false;

    int frame_id = 0;
    while (!state.request_stop.load()) {
        if (!state.capture_enabled.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
            continue;
        }

        t::geometry::RGBDImage rgbd = (frame_id == 0) ? first : capture_frame();
        if (rgbd.IsEmpty()) {
            if (frame_id == 0) {
                utility::LogError("Empty RGB-D frame at startup.");
            }
            ++empty_capture_retries;
            if (empty_capture_retries >= kMaxEmptyCaptureRetries) {
                utility::LogWarning(
                        "Capture failed {} times; stopping SLAM loop.",
                        empty_capture_retries);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        empty_capture_retries = 0;

        rgbd = rgbd.To(device);
        input_frame.SetDataFromImage("depth", rgbd.depth_);
        input_frame.SetDataFromImage("color", rgbd.color_);

        const int64_t hash_size_before = model.GetHashMap().Size();
        const int64_t hash_capacity = model.GetHashMap().GetCapacity();
        const bool hash_near_full =
                IsHashNearFull(hash_size_before, hash_capacity);
        if (hash_near_full && !hash_full_warned) {
            utility::LogWarning(
                    "Voxel hash nearly full ({}/{}). Stopping map growth but "
                    "continuing pose tracking. Press ESC to finish and save, "
                    "or restart with --profile high / --block_count.",
                    hash_size_before, hash_capacity);
            hash_full_warned = true;
        }

        bool integrate = (frame_id == 0) && !hash_near_full;
        TrackingTier tracking_tier = TrackingTier::kInit;
        const bool prefer_f2f_bridge =
                tracking_was_unstable &&
                consecutive_tracking_failures > kLostTrackingThreshold &&
                (frame_id % kModelRetryIntervalWhenLost) != 0;

        if (frame_id > 0 && !prefer_f2f_bridge) {
            tracking_tier = TrackingTier::kFail;
            try {
                auto run_model_tracking =
                        [&](float depth_diff) {
                            return model.TrackFrameToModel(
                                    input_frame, raycast_frame,
                                    params.depth_scale, params.depth_max,
                                    SafeOdometryDepthDiff(depth_diff),
                                    t::pipelines::odometry::Method::PointToPlane,
                                    odom_criteria);
                        };

                core::Tensor track_transform;
                double track_fitness = 0.0;
                double track_translation = 0.0;
                double track_rotation = 0.0;
                {
                    std::lock_guard<std::mutex> model_lock(runtime.model_mutex);
                    auto result = run_model_tracking(params.depth_diff);
                    track_fitness = result.fitness_;
                    track_transform = result.transformation_;
                    track_translation =
                            TranslationNorm(result.transformation_);
                    track_rotation = RotationAngleDeg(result.transformation_);
                    tracking_tier =
                            ClassifyTracking(track_fitness, track_translation,
                                             track_rotation);

                    if (tracking_tier == TrackingTier::kFail &&
                        track_fitness < kWeakFitnessMin) {
                        result = run_model_tracking(params.depth_diff * 2.0f);
                        track_fitness = result.fitness_;
                        track_transform = result.transformation_;
                        track_translation =
                                TranslationNorm(result.transformation_);
                        track_rotation =
                                RotationAngleDeg(result.transformation_);
                        tracking_tier =
                                ClassifyTracking(track_fitness,
                                                 track_translation,
                                                 track_rotation);
                    }
                }

                if (tracking_tier == TrackingTier::kStrong) {
                    core::Tensor candidate_T_frame_to_model =
                            T_frame_to_model.Matmul(track_transform);
                    ++consecutive_strong;
                    consecutive_tracking_failures = 0;
                    if (!tracking_was_unstable) {
                        T_frame_to_model = candidate_T_frame_to_model;
                        integrate = !hash_near_full;
                    } else if (tracking_was_unstable &&
                               consecutive_strong >= kStrongStreakAfterLost &&
                               IsRecoveryPoseStable(track_fitness,
                                                    track_translation,
                                                    track_rotation)) {
                        T_frame_to_model = candidate_T_frame_to_model;
                        integrate = !hash_near_full;
                        tracking_was_unstable = false;
                        consecutive_f2f_bridges = 0;
                        if (lost_camera_marker_visible) {
                            state.SetLostCameraMarker(nullptr, false);
                            state.SetPoseDiffText("", false);
                            lost_camera_marker_visible = false;
                        }
                        utility::LogInfo(
                                "Tracking restabilized at frame {} — "
                                "resuming integration.",
                                frame_id);
                    } else if (tracking_was_unstable) {
                        utility::LogWarning(
                                "Recovery candidate frame {} held from "
                                "integration (strong streak {}, fitness {:.3f}, "
                                "translation {:.3f}, rotation {:.2f} deg).",
                                frame_id, consecutive_strong, track_fitness,
                                track_translation, track_rotation);
                        ++rejected_pose_updates;
                    }
                } else {
                    consecutive_strong = 0;
                    consecutive_f2f_bridges = 0;
                    tracking_was_unstable = true;
                    ++consecutive_tracking_failures;
                    const char* tier_name = TrackingTierName(tracking_tier);
                    utility::LogWarning(
                            "Tracking {} for frame {}, fitness: {:.3f}, "
                            "translation: {:.3f}, rotation: {:.2f} deg. "
                            "Skipping integration.",
                            tier_name, frame_id, track_fitness,
                            track_translation, track_rotation);
                    ++rejected_pose_updates;
                }
            } catch (const std::exception& e) {
                tracking_tier = TrackingTier::kFail;
                consecutive_strong = 0;
                consecutive_f2f_bridges = 0;
                tracking_was_unstable = true;
                ++consecutive_tracking_failures;
                if (IsOdometrySingularError(e)) {
                    utility::LogWarning(
                            "Odometry singular at frame {} (low overlap or "
                            "featureless view).",
                            frame_id);
                } else {
                    utility::LogWarning("Tracking exception for frame {}: {}",
                                        frame_id, e.what());
                }
            }
        } else if (frame_id == 0) {
            consecutive_strong = 1;
        } else if (prefer_f2f_bridge) {
            tracking_tier = TrackingTier::kWeak;
        }

        if (frame_id > 0 && tracking_was_unstable && !prev_rgbd.IsEmpty() &&
            tracking_tier != TrackingTier::kStrong) {
            try {
                auto f2f_result =
                        t::pipelines::odometry::RGBDOdometryMultiScale(
                                rgbd, prev_rgbd, intrinsic, identity_pose,
                                params.depth_scale, params.depth_max,
                                odom_criteria,
                                t::pipelines::odometry::Method::PointToPlane,
                                t::pipelines::odometry::OdometryLossParams(
                                        SafeOdometryDepthDiff(
                                                params.depth_diff)));
                const double f2f_fitness = f2f_result.fitness_;
                const double f2f_translation =
                        TranslationNorm(f2f_result.transformation_);
                const double f2f_rotation =
                        RotationAngleDeg(f2f_result.transformation_);
                if (IsFrameToFrameBridgeStable(f2f_fitness, f2f_translation,
                                               f2f_rotation)) {
                    T_frame_to_model =
                            T_frame_to_model.Matmul(f2f_result.transformation_);
                    ++consecutive_f2f_bridges;
                    ++f2f_bridge_successes;
                    // Pose bridge keeps raycasting near the current camera view,
                    // but integration stays disabled until model tracking
                    // restabilizes against the TSDF.
                    utility::LogInfo(
                            "Frame-to-frame bridge frame {} accepted "
                            "(streak {}, fitness {:.3f}, translation {:.3f}, "
                            "rotation {:.2f} deg).",
                            frame_id, consecutive_f2f_bridges, f2f_fitness,
                            f2f_translation, f2f_rotation);
                } else if (!prefer_f2f_bridge) {
                    utility::LogWarning(
                            "Frame-to-frame bridge frame {} rejected "
                            "(fitness {:.3f}, translation {:.3f}, rotation "
                            "{:.2f} deg).",
                            frame_id, f2f_fitness, f2f_translation,
                            f2f_rotation);
                }
            } catch (const std::exception& f2f_e) {
                if (!IsOdometrySingularError(f2f_e) && !prefer_f2f_bridge) {
                    utility::LogWarning(
                            "Frame-to-frame bridge failed at frame {}: {}",
                            frame_id, f2f_e.what());
                }
            }
        }

        model.UpdateFramePose(frame_id, T_frame_to_model);
        {
            std::lock_guard<std::mutex> model_lock(runtime.model_mutex);
            if (integrate && !hash_near_full) {
                model.Integrate(input_frame, params.depth_scale, params.depth_max,
                                params.trunc_multiplier);
                ++integrated_frames;
            }
            model.SynthesizeModelFrame(raycast_frame, params.depth_scale, 0.1f,
                                       params.depth_max, params.trunc_multiplier,
                                       false);
        }

        if (integrate) {
            last_stable_T_frame_to_model = T_frame_to_model.Contiguous();
            if (lost_camera_marker_visible) {
                state.SetLostCameraMarker(nullptr, false);
                state.SetPoseDiffText("", false);
                lost_camera_marker_visible = false;
            }
            camera::PinholeCameraParameters cam_params;
            cam_params.intrinsic_ = cam_intrinsic;
            cam_params.extrinsic_ = core::eigen_converter::TensorToEigenMatrixXd(
                    T_frame_to_model);
            trajectory->parameters_.push_back(cam_params);
        }

        if (!lost_camera_marker_visible &&
            consecutive_tracking_failures > kLostTrackingThreshold) {
            state.SetLostCameraMarker(
                    CreateCameraMarker(width, height, intrinsic_eigen,
                                       last_stable_T_frame_to_model),
                    true);
            lost_camera_marker_visible = true;
            utility::LogInfo(
                    "Tracking lost. Showing orange last-stable camera marker; "
                    "return near this camera frustum to relocalize.");
        }
        if (consecutive_tracking_failures > kLostTrackingThreshold) {
            state.SetPoseDiffText(
                    BuildPoseDiffText(last_stable_T_frame_to_model,
                                      T_frame_to_model),
                    true);
        }

        if (ShouldRefreshDisplay(frame_id, params.update_interval)) {
            const float weight = ExtractWeightThreshold(frame_id);
            const int extract_budget =
                    GetExtractPointBudget(params.estimated_points,
                                          hash_size_before);
            try {
                t::geometry::PointCloud pcd_t;
                t::geometry::PointCloud region_pcd;
                {
                    std::lock_guard<std::mutex> model_lock(runtime.model_mutex);
                    if (params.regions.enabled) {
                        pcd_t = model.ExtractPointCloudExcludingFrozen(
                                weight, extract_budget);
                    } else {
                        pcd_t = model.ExtractPointCloud(weight, extract_budget);
                    }
                    if (params.regions.enabled &&
                        ShouldCheckRegions(frame_id, params.regions.interval)) {
                        region_pcd = pcd_t.To(core::Device("CPU:0"));
                    }
                }
                auto pcd =
                        std::make_shared<geometry::PointCloud>(pcd_t.ToLegacy());
                if (!pcd->IsEmpty()) {
                    state.SetPointCloud(pcd);
                }
                if (params.regions.enabled &&
                    region_pcd.HasPointPositions()) {
                    {
                        std::lock_guard<std::mutex> lock(runtime.region_mutex);
                        runtime.pending_region_pcd = std::move(region_pcd);
                        runtime.pending_extract_weight = weight;
                        runtime.pending_frame_id = frame_id;
                        runtime.region_requested.store(true);
                    }
                    runtime.region_cv.notify_one();
                }
            } catch (const std::exception& e) {
                utility::LogWarning("Live extract skipped at frame {}: {}",
                                    frame_id, e.what());
            }
        }

        std::string status = "Frame " + std::to_string(frame_id) + " | blocks " +
                             std::to_string(hash_size_before) + "/" +
                             std::to_string(hash_capacity);
        if (params.regions.enabled) {
            status += " | regions " + std::to_string(state.RegionCount());
        }
        if (hash_near_full) {
            status += " | HASH FULL (tracking only)";
        } else if (consecutive_tracking_failures > kLostTrackingThreshold) {
            status += " | RELOCALIZING lost " +
                      std::to_string(consecutive_tracking_failures) +
                      " | f2f " + std::to_string(consecutive_f2f_bridges) +
                      " | return to orange camera";
        } else if (frame_id > 0 && tracking_tier != TrackingTier::kStrong) {
            status += " | tracking " + std::string(TrackingTierName(tracking_tier));
        }
        state.SetStatus(status);

        utility::LogInfo(
                "SLAM frame {} | hash blocks {}/{} | tier {} | f2f bridge {}",
                frame_id, hash_size_before, hash_capacity,
                TrackingTierName(tracking_tier), consecutive_f2f_bridges);

        prev_rgbd = rgbd;
        ++frame_id;
    }

    runtime.model_ready.store(false);
    runtime.region_stop.store(true);
    runtime.region_cv.notify_all();

    const int64_t final_hash_size = model.GetHashMap().Size();
    utility::LogInfo(
            "Extracting final scene (2-pass point count, hash blocks {})...",
            final_hash_size);

    try {
        t::geometry::PointCloud final_pcd_t;
        t::geometry::TriangleMesh final_mesh_t;
        {
            std::lock_guard<std::mutex> model_lock(runtime.model_mutex);
            // estimated_number=-1: two-pass extract, no truncation warning.
            final_pcd_t = model.ExtractPointCloud(3.f, -1);
            final_mesh_t = model.ExtractTriangleMesh(3.f, -1);
        }
        auto final_pcd =
                std::make_shared<geometry::PointCloud>(final_pcd_t.ToLegacy());
        ClampPointColors(*final_pcd);
        state.SetPointCloud(final_pcd);
        io::WritePointCloud("scene.ply", *final_pcd);
        utility::LogInfo("Saved scene.ply ({} points).", final_pcd->points_.size());

        auto final_mesh =
                std::make_shared<geometry::TriangleMesh>(final_mesh_t.ToLegacy());
        ClampVertexColors(*final_mesh);
        io::WriteTriangleMesh("scene_mesh.ply", *final_mesh);
        utility::LogInfo("Saved scene_mesh.ply ({} vertices).",
                         final_mesh->vertices_.size());
    } catch (const std::exception& e) {
        utility::LogError("Failed to extract/save scene: {}", e.what());
    }

    if (params.regions.enabled) {
        std::lock_guard<std::mutex> lock(runtime.records_mutex);
        WriteRegionsJson(params.regions.output_dir, runtime.region_records);
        utility::LogInfo("Saved {} region record(s) to {}/regions.json.",
                         runtime.region_records.size(),
                         params.regions.output_dir);
    }

    io::WritePinholeCameraTrajectory("trajectory.log", *trajectory);

    const double integrate_ratio =
            frame_id > 0 ? 100.0 * integrated_frames / frame_id : 100.0;
    utility::LogInfo(
            "Saved scene.ply, scene_mesh.ply, trajectory.log "
            "({} SLAM frames, {} integrated ({:.1f}%), {} strong poses, {} "
            "f2f bridges, {} rejected pose updates).",
            frame_id, integrated_frames, integrate_ratio,
            trajectory->parameters_.size(), f2f_bridge_successes,
            rejected_pose_updates);
    if (integrate_ratio < 50.0 && frame_id > 30) {
        utility::LogWarning(
                "Less than half of frames were integrated. Rescan slowly with "
                "more overlap, or use --profile high --depth_max 8.");
    }

    state.slam_finished.store(true);
}

}  // namespace

int main(int argc, char* argv[]) {
    utility::SetVerbosityLevel(utility::VerbosityLevel::Info);

    if (utility::ProgramOptionExistsAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }
    if (argc <= 1) {
        utility::LogInfo(
                "No options given — starting live SLAM with default settings "
                "(D415 config, CUDA:0, medium profile). Use --help for "
                "options.");
    }

    if (utility::ProgramOptionExists(argc, argv, "-l") ||
        utility::ProgramOptionExists(argc, argv, "--list-devices")) {
        tio::RealSenseSensor::ListDevices();
        return 0;
    }

    if (utility::ProgramOptionExists(argc, argv, "-V")) {
        utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    }

    const std::string bag_file =
            utility::GetProgramOptionAsString(argc, argv, "--use_bag_file", "");
    const bool use_bag = !bag_file.empty();

    std::string config_file;
    if (utility::ProgramOptionExists(argc, argv, "-c")) {
        config_file = utility::GetProgramOptionAsString(argc, argv, "-c");
    } else if (utility::ProgramOptionExists(argc, argv, "--config")) {
        config_file =
                utility::GetProgramOptionAsString(argc, argv, "--config");
    } else if (!use_bag) {
        config_file = DefaultConfigPath();
        if (!config_file.empty()) {
            utility::LogInfo("Using default D415 config: {}", config_file);
        } else {
            utility::LogError(
                    "Default RealSense config not found. Run from the "
                    "repository root/SLAM output directory or pass "
                    "-c d:\\study\\Open3D\\examples\\test_data\\rs_d415_slam.json.");
            return 1;
        }
    }

    bool align_streams = true;
    if (utility::ProgramOptionExists(argc, argv, "--no-align")) {
        align_streams = false;
    } else if (utility::ProgramOptionExists(argc, argv, "--align")) {
        align_streams = true;
    }

    const std::string record_bag =
            utility::GetProgramOptionAsString(argc, argv, "--record", "");

    const std::string profile =
            utility::GetProgramOptionAsString(argc, argv, "--profile", "medium");
    if (profile != "low" && profile != "medium" && profile != "high") {
        utility::LogError("Unknown --profile '{}'. Use low, medium, or high.",
                          profile);
        return 1;
    }

    SlamParams params = GetProfile(profile);
    if (utility::ProgramOptionExists(argc, argv, "--update_interval")) {
        params.update_interval = utility::GetProgramOptionAsInt(
                argc, argv, "--update_interval", params.update_interval);
    }
    if (utility::ProgramOptionExists(argc, argv, "--depth_max")) {
        params.depth_max = static_cast<float>(utility::GetProgramOptionAsDouble(
                argc, argv, "--depth_max", params.depth_max));
    }
    if (utility::ProgramOptionExists(argc, argv, "--voxel_size")) {
        params.voxel_size = static_cast<float>(utility::GetProgramOptionAsDouble(
                argc, argv, "--voxel_size", params.voxel_size));
    }
    if (utility::ProgramOptionExists(argc, argv, "--block_count")) {
        params.block_count = utility::GetProgramOptionAsInt(
                argc, argv, "--block_count", params.block_count);
    }
    if (utility::ProgramOptionExists(argc, argv, "--estimated_points")) {
        params.estimated_points = utility::GetProgramOptionAsInt(
                argc, argv, "--estimated_points", params.estimated_points);
    }

    if (utility::ProgramOptionExists(argc, argv, "--regions")) {
        params.regions.enabled = true;
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_min_points")) {
        params.regions.min_points = utility::GetProgramOptionAsInt(
                argc, argv, "--region_min_points", params.regions.min_points);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_stability")) {
        params.regions.stability_frames = utility::GetProgramOptionAsInt(
                argc, argv, "--region_stability",
                params.regions.stability_frames);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_interval")) {
        params.regions.interval = utility::GetProgramOptionAsInt(
                argc, argv, "--region_interval", params.regions.interval);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_dir")) {
        params.regions.output_dir =
                utility::GetProgramOptionAsString(argc, argv, "--region_dir",
                                                  params.regions.output_dir);
    }

    const std::string device_code =
            utility::GetProgramOptionAsString(argc, argv, "--device", "CUDA:0");
    const core::Device device(device_code);
    utility::LogInfo("Compute device: {}", device.ToString());
    utility::LogInfo("SLAM profile: {} (voxel {:.4f} m, depth_max {:.1f} m, "
                     "blocks {}, est. points {})",
                     profile, params.voxel_size, params.depth_max,
                     params.block_count, params.estimated_points);

    tio::RealSenseSensor rs;
    tio::RSBagReader bag_reader;
    std::function<t::geometry::RGBDImage()> capture_frame;
    core::Tensor intrinsic;
    camera::PinholeCameraIntrinsic cam_intrinsic;
    float depth_scale = params.depth_scale;

    if (use_bag) {
        bag_reader.Open(bag_file);
        if (!bag_reader.IsOpened()) {
            utility::LogError("Unable to open bag file: {}", bag_file);
            return 1;
        }
        const auto meta = bag_reader.GetMetadata();
        utility::LogInfo("{}", meta.ToString());
        depth_scale = static_cast<float>(meta.depth_scale_);
        params.depth_scale = depth_scale;
        cam_intrinsic = meta.intrinsics_;
        intrinsic = core::eigen_converter::EigenMatrixToTensor(
                meta.intrinsics_.intrinsic_matrix_);

        capture_frame = [&bag_reader]() -> t::geometry::RGBDImage {
            if (bag_reader.IsEOF()) {
                return t::geometry::RGBDImage();
            }
            return bag_reader.NextFrame();
        };
    } else {
        tio::RealSenseSensorConfig rs_cfg;
        if (!config_file.empty()) {
            if (!io::ReadIJsonConvertible(config_file, rs_cfg)) {
                utility::LogError("Failed to read RealSense config: {}",
                                  config_file);
                return 1;
            }
        }

        try {
            if (!rs.InitSensor(rs_cfg, 0, record_bag)) {
                utility::LogWarning(
                        "RealSense sensor initialization failed. Check that "
                        "the camera is connected and not used by another "
                        "application.");
                return 1;
            }
            utility::LogInfo("{}", rs.GetMetadata().ToString());
            depth_scale = static_cast<float>(rs.GetMetadata().depth_scale_);
            params.depth_scale = depth_scale;
            cam_intrinsic = rs.GetMetadata().intrinsics_;
            intrinsic = core::eigen_converter::EigenMatrixToTensor(
                    rs.GetMetadata().intrinsics_.intrinsic_matrix_);
            if (!rs.StartCapture(!record_bag.empty())) {
                utility::LogWarning(
                        "RealSense capture failed to start. Check the camera "
                        "connection and close other RealSense applications.");
                return 1;
            }
        } catch (const std::exception& e) {
            utility::LogWarning("RealSense startup failed: {}", e.what());
            return 1;
        }
        // RealSense pipelines need a short warmup before the first frame.
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        capture_frame = [&rs, align_streams]() -> t::geometry::RGBDImage {
            return rs.CaptureFrame(true, align_streams);
        };
    }

    DisplayState display_state;
    SlamRuntime runtime;
    std::thread region_thread;
    if (params.regions.enabled) {
        region_thread = std::thread(RegionWorker, std::ref(runtime),
                                    std::cref(params), std::ref(display_state));
    }
    std::thread slam_thread(SlamWorker, capture_frame, intrinsic, cam_intrinsic,
                            device, params, std::ref(runtime),
                            std::ref(display_state));

    auto& app = visualization::gui::Application::GetInstance();
    app.Initialize();
    const auto mono = app.AddFont(
            visualization::gui::FontDescription(
                    visualization::gui::FontDescription::MONOSPACE));
    app.AddWindow(std::make_shared<realtime_slam::RealTimeSLAMWindow>(
            display_state, mono));
    app.Run();

    display_state.request_stop.store(true);
    runtime.region_stop.store(true);
    runtime.region_cv.notify_all();
    slam_thread.join();
    if (params.regions.enabled && region_thread.joinable()) {
        region_thread.join();
    }

    if (!use_bag) {
        rs.StopCapture();
    } else {
        bag_reader.Close();
    }

    return 0;
}
