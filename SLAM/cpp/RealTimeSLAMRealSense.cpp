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
#include "Relocalization.h"
#include "open3d/Open3D.h"

namespace {

using namespace open3d;

int64_t MeshVertexCount(const t::geometry::TriangleMesh& mesh) {
    if (!mesh.HasVertexPositions()) {
        return 0;
    }
    return mesh.GetVertexPositions().GetLength();
}

int64_t MeshTriangleCount(const t::geometry::TriangleMesh& mesh) {
    if (!mesh.HasTriangleIndices()) {
        return 0;
    }
    return mesh.GetTriangleIndices().GetLength();
}

namespace tio = open3d::t::io;
namespace object_mesh = open3d::examples::object_mesh;
namespace realtime_slam = open3d::examples::realtime_slam;
namespace relocalization = open3d::examples::relocalization;
using DisplayState = realtime_slam::RealTimeSLAMWindow::DisplayState;

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
    object_mesh::RegionParams regions;
};

SlamParams GetProfile(const std::string& profile) {
    SlamParams p;
    if (profile == "low") {
        p.voxel_size = 0.008f;
        p.block_count = 16384;
        p.estimated_points = 2500000;
        p.depth_max = 2.f;
        p.update_interval = 20;
        p.regions.min_points = 2000;
        p.regions.stability_frames = 3;
        p.regions.interval = 30;
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
        p.regions.min_points = 5000;
        p.regions.stability_frames = 5;
        p.regions.interval = 60;
        return p;
    }
    // Default medium: balance quality and VRAM for indoor region mesh.
    p.regions.min_points = 3000;
    p.regions.stability_frames = 5;
    p.regions.interval = 45;
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
    utility::LogInfo("    [--region_min_readiness F] Min mesh readiness 0-1 (default: 0.80).");
    utility::LogInfo("    [--region_max_holey_defer N] Max hole-defer cycles (default: 40).");
    utility::LogInfo("    [--region_no_holey_defer] Disable holey-region mesh deferral.");
    utility::LogInfo("    [--region_max_void_ratio R] Max local void ratio (default: 0.08).");
    utility::LogInfo("    [--region_max_void_blob N] Max connected void cells (default: 32).");
    utility::LogInfo("    [--region_max_boundary_void R] Max boundary void ratio (default: 0.10).");
    utility::LogInfo("    [--region_extract_weight W] Region TSDF extract weight (default: 3.0).");
    utility::LogInfo("    [--region_depth_min M] Camera-distance band min meters (default: 0.3).");
    utility::LogInfo("    [--region_depth_max M] Camera-distance band max (default: min(2.5, depth_max)).");
    utility::LogInfo("    [--region_max_extent M] Max cluster extent / split tile size m (default: 1.2).");
    utility::LogInfo("    [--region_min_motion_m M] Net translation (m) vs anchor (default: 0.05).");
    utility::LogInfo("    [--region_min_motion_deg D] Net rotation (deg) vs anchor (default: 8).");
    utility::LogInfo("    [--region_min_hash_delta N] Min hash-block growth with motion (default: 2).");
    utility::LogInfo("    [--region_motion_warmup N] Skip region mesh for first N frames (default: 45).");
    utility::LogInfo("    [--region_allow_stationary_mesh] Allow mesh commit while camera is still.");
    utility::LogInfo("    [--global_reloc 0|1]    Enable keyframe global relocalization (default: 1).");
    utility::LogInfo("    [--keyframe_interval N] Min frames between keyframes (default: 30).");
    utility::LogInfo("    [--keyframe_max N]      Max stored keyframes (default: 48).");
    utility::LogInfo("    [--reloc_method NAME]   Global registration: ransac or fgr.");
    utility::LogInfo("    [--reloc_retry_interval N] Global reloc attempt interval when lost.");
    utility::LogInfo("    [--reloc_candidate_radius M] Spatial search radius for keyframes.");
    utility::LogInfo("    [--reloc_min_fitness F]   Minimum ICP fitness to accept global reloc.");
    utility::LogInfo("    [--reloc_max_pose_jump M] Max jump vs last_stable (m, default: 2.0).");
    utility::LogInfo("    [--reloc_ransac_iter N]   RANSAC max iterations (default: 40000).");
    utility::LogInfo("    [--reloc_self_test]       Auto lost/global-reloc self test (exits).");
    utility::LogInfo("");
    utility::LogInfo("GUI controls (left panel):");
    utility::LogInfo("    Cloud capture ON/OFF    Pause/resume RGB-D capture and SLAM integration.");
    utility::LogInfo("    Polygon ON/OFF          Show/hide frozen region triangle meshes.");
    utility::LogInfo("    Point cloud ON/OFF      Show/hide live scan + frozen region point clouds.");
    utility::LogInfo("");
    utility::LogInfo("Indoor room scan tips:");
    utility::LogInfo("    Walk slowly (~0.3 m/s), keep 30%% overlap between views.");
    utility::LogInfo("    Prefer --profile medium for region mesh quality (holes).");
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
// Surface crossings can greatly exceed sparse occupancy; under-estimating the
// one-pass CUDA extract buffer can abort the process (0xc0000409).
static constexpr int kPointsPerHashBlock = 1600;
static constexpr int kMaxRegionSegmentationPoints = 60000;
static constexpr double kDbscanEpsMultiplier = 4.0;

int GetExtractPointBudget(int estimated_points, int64_t hash_size) {
    const int64_t hash_based = std::min(
            hash_size * static_cast<int64_t>(kPointsPerHashBlock),
            static_cast<int64_t>(kMaxExtractPoints));
    // Extra headroom: one-pass extract must not be smaller than the true count.
    const int64_t with_margin = std::min(
            hash_based + hash_based / 2,
            static_cast<int64_t>(kMaxExtractPoints));
    return static_cast<int>(std::max(static_cast<int64_t>(estimated_points),
                                     with_margin));
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
        const Eigen::Vector3d& color,
        double scale = 0.25) {
    const Eigen::Matrix4d camera_to_world =
            core::eigen_converter::TensorToEigenMatrixXd(T_frame_to_model);
    auto marker = geometry::LineSet::CreateCameraVisualization(
            width, height, intrinsic, camera_to_world.inverse(), scale);
    marker->PaintUniformColor(color);
    return marker;
}

/// Last-stable pose: Tango orange (return target).
const Eigen::Vector3d kLastStableCameraColor(0.961, 0.475, 0.000);
/// Current estimated pose while lost: cyan/blue (where you are now).
const Eigen::Vector3d kCurrentCameraColor(0.15, 0.55, 1.00);

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
    ss << "Orange: last stable  |  Blue: current\n";
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

object_mesh::SegmentationConfig BuildRegionSegmentationConfig(
        const SlamParams& params, float extract_weight) {
    return object_mesh::BuildLiveRegionSegmentationConfig(
            params.voxel_size, params.trunc_multiplier, extract_weight,
            params.regions.min_points, params.regions.stability_frames,
            params.regions, kDbscanEpsMultiplier);
}

struct SlamRuntime {
    std::mutex model_mutex;
    t::pipelines::slam::Model* model = nullptr;
    std::atomic<bool> model_ready{false};

    std::mutex region_mutex;
    std::condition_variable region_cv;
    std::atomic<bool> region_requested{false};
    std::atomic<bool> region_worker_busy{false};
    std::atomic<bool> region_stop{false};
    t::geometry::PointCloud pending_region_pcd;
    float pending_extract_weight = 3.0f;
    int pending_frame_id = 0;
    t::geometry::PointCloud last_region_surface_pcd;
    float last_region_extract_weight = 3.0f;
    int last_region_frame_id = 0;
    bool have_last_region_surface = false;

    double motion_window_translation_m = 0.0;
    double motion_window_rotation_deg = 0.0;
    bool camera_moved_for_regions = false;
    core::Tensor motion_anchor_T;
    int64_t motion_anchor_hash = 0;
    bool have_motion_anchor = false;
    std::vector<object_mesh::FrozenObjectCandidate> committed_region_meshes;

    object_mesh::ObjectFreezeTracker freeze_tracker{
            object_mesh::SegmentationConfig{}};
    int next_object_id = 0;
    std::mutex records_mutex;
    std::vector<object_mesh::RegionRecord> region_records;

    relocalization::RelocalizationConfig reloc_config;

    // Async global relocalization (does not block the SLAM integrate loop).
    std::mutex reloc_mutex;
    std::condition_variable reloc_cv;
    std::atomic<bool> reloc_requested{false};
    std::atomic<bool> reloc_worker_busy{false};
    std::atomic<bool> reloc_stop{false};
    std::atomic<bool> reloc_result_ready{false};
    t::geometry::RGBDImage pending_reloc_rgbd;
    core::Tensor pending_reloc_intrinsic;
    core::Tensor pending_reloc_last_stable;
    std::vector<int> pending_reloc_candidate_ids;
    int pending_reloc_frame_id = 0;
    int pending_reloc_request_id = 0;
    float pending_reloc_depth_scale = 1000.f;
    float pending_reloc_depth_max = 3.f;
    bool pending_reloc_used_fallback = false;
    int next_reloc_request_id = 1;
    relocalization::RelocalizationAttempt reloc_result;
    int reloc_result_frame_id = 0;
    int reloc_result_request_id = 0;
    double reloc_result_elapsed_ms = 0.0;
    bool reloc_result_used_fallback = false;
    int reloc_result_candidate_count = 0;
    // Shared with RelocWorker; set by SlamWorker before starting the thread.
    relocalization::KeyframeDatabase* reloc_keyframe_db = nullptr;
    core::Device reloc_device{"CPU:0"};

    struct RelocSelfTestParams {
        bool enabled = false;
        int warmup_frames = 35;
        int min_keyframes = 1;
        double drift_translation_m = 0.20;
        double drift_rotation_deg = 15.0;
        int recovery_timeout_frames = 8;
        double max_pose_error_m = 0.15;
        double max_pose_error_deg = 12.0;
    } reloc_self_test;
    std::atomic<bool> reloc_self_test_passed{false};
    std::atomic<bool> reloc_self_test_finished{false};
};

core::Tensor ApplyPoseDrift(const core::Tensor& T,
                            double translation_m,
                            double rotation_deg) {
    Eigen::Matrix4d pose =
            core::eigen_converter::TensorToEigenMatrixXd(T);
    Eigen::Matrix4d drift = Eigen::Matrix4d::Identity();
    drift(0, 3) = translation_m;
    const double rad = rotation_deg * 3.14159265358979323846 / 180.0;
    drift.block<3, 3>(0, 0) =
            Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitY()).toRotationMatrix();
    return core::eigen_converter::EigenMatrixToTensor(pose * drift);
}

void RegionWorker(SlamRuntime& runtime,
                  const SlamParams& params,
                  DisplayState& state) {
    bool exit_flush_done = false;
    while (true) {
        t::geometry::PointCloud surface_pcd;
        float extract_weight = 3.0f;
        int frame_id = 0;
        bool process_request = false;
        bool session_stopping = false;
        {
            std::unique_lock<std::mutex> lock(runtime.region_mutex);
            runtime.region_cv.wait(lock, [&]() {
                return runtime.region_requested.load() ||
                       runtime.region_stop.load() || state.request_stop.load();
            });
            session_stopping =
                    runtime.region_stop.load() || state.request_stop.load();

            if (runtime.region_requested.load()) {
                runtime.region_requested.store(false);
                surface_pcd = std::move(runtime.pending_region_pcd);
                extract_weight = runtime.pending_extract_weight;
                frame_id = runtime.pending_frame_id;
                process_request = true;
            } else if (session_stopping && !exit_flush_done &&
                       params.regions.flush_holey_on_exit &&
                       runtime.have_last_region_surface) {
                surface_pcd = runtime.last_region_surface_pcd;
                extract_weight = runtime.last_region_extract_weight;
                frame_id = runtime.last_region_frame_id;
                process_request = true;
                exit_flush_done = true;
            } else {
                break;
            }
        }

        if (!process_request || !runtime.model_ready.load() || !runtime.model ||
            surface_pcd.IsEmpty()) {
            if (session_stopping) {
                break;
            }
            continue;
        }

        runtime.region_worker_busy.store(true);
        const auto region_t0 = std::chrono::steady_clock::now();

        try {
            const int64_t before_down =
                    surface_pcd.HasPointPositions()
                            ? surface_pcd.GetPointPositions().GetLength()
                            : 0;
            float downsample_voxel = params.voxel_size;
            surface_pcd = object_mesh::DownsamplePointCloudIfNeeded(
                    surface_pcd, kMaxRegionSegmentationPoints,
                    params.voxel_size, &downsample_voxel);
            object_mesh::SegmentationConfig config =
                    BuildRegionSegmentationConfig(params, extract_weight);
            // DBSCAN eps must track the *actual* downsample spacing. When the
            // map is huge, voxel grows above params.voxel_size and a fixed eps
            // (4*voxel_size) leaves every point as noise → tracked 0 forever.
            config.dbscan_eps = std::max(
                    config.dbscan_eps,
                    4.0 * static_cast<double>(downsample_voxel));
            config.camera_moved_since_last_check =
                    runtime.camera_moved_for_regions ||
                    !params.regions.require_camera_motion;
            runtime.freeze_tracker.SetConfig(config);

            const int64_t after_down =
                    surface_pcd.HasPointPositions()
                            ? surface_pcd.GetPointPositions().GetLength()
                            : 0;
            utility::LogInfo(
                    "Region downsample: {} -> {} points (voxel={:.4f} m, "
                    "dbscan_eps={:.4f} m).",
                    before_down, after_down, downsample_voxel,
                    config.dbscan_eps);

            std::vector<object_mesh::FrozenObjectCandidate> pending;
            pending = object_mesh::ProcessExtractedSurface(
                    surface_pcd, *runtime.model, runtime.freeze_tracker,
                    config, runtime.next_object_id);

            if (pending.empty()) {
                utility::LogInfo(
                        "Region check frame {}: surface segmented, waiting for "
                        "stable cluster (tracked {}, pending {}, holey {}, "
                        "frozen {}).",
                        frame_id, runtime.freeze_tracker.TrackedCount(),
                        runtime.freeze_tracker.PendingCount(),
                        runtime.freeze_tracker.PendingHoleyCount(),
                        runtime.freeze_tracker.FrozenCount());
            }

            const bool session_stopping =
                    runtime.region_stop.load() || state.request_stop.load();
            object_mesh::RegionProcessResult processed;
            {
                std::lock_guard<std::mutex> model_lock(runtime.model_mutex);
                processed = object_mesh::ProcessPendingRegionCandidates(
                        pending, *runtime.model, runtime.freeze_tracker, config,
                        session_stopping, runtime.committed_region_meshes);
            }

            for (auto& seam : processed.seam_updates) {
                DisplayState::RegionPair pair;
                pair.id = seam.id;
                if (object_mesh::IsValidRegionMesh(seam.mesh)) {
                    pair.mesh = std::make_shared<geometry::TriangleMesh>(
                            seam.mesh.ToLegacy());
                    object_mesh::ClampVertexColors(*pair.mesh);
                    state.PushRegionPair(std::move(pair));
                }
                for (auto& stored : runtime.committed_region_meshes) {
                    if (stored.id == seam.id) {
                        stored.mesh = std::move(seam.mesh);
                        break;
                    }
                }
            }

            if (processed.committed.empty()) {
                utility::LogInfo(
                        "Region check frame {}: {} candidate(s) tracked, none "
                        "ready to freeze yet (deferred holey {} void {} "
                        "stationary {}).",
                        frame_id, static_cast<int>(pending.size()),
                        processed.deferred_holey, processed.deferred_void,
                        processed.deferred_stationary);
            } else {
                int total_regions = 0;
                {
                    std::lock_guard<std::mutex> lock(runtime.records_mutex);
                    for (auto& candidate : processed.committed) {
                        object_mesh::RegionRecord record;
                        if (!object_mesh::SaveFrozenRegion(params.regions, candidate,
                                                           frame_id, record)) {
                            continue;
                        }
                        runtime.region_records.push_back(record);

                        DisplayState::RegionPair pair;
                        pair.id = record.id;
                        if (object_mesh::IsValidRegionMesh(candidate.mesh)) {
                            pair.mesh = std::make_shared<geometry::TriangleMesh>(
                                    candidate.mesh.ToLegacy());
                            object_mesh::ClampVertexColors(*pair.mesh);
                        }
                        object_mesh::FrozenObjectCandidate stored = candidate;
                        runtime.committed_region_meshes.push_back(
                                std::move(stored));
                        state.PushRegionPair(std::move(pair));
                    }
                    object_mesh::WriteRegionsJson(params.regions.output_dir,
                                     runtime.region_records);
                    total_regions = static_cast<int>(runtime.region_records.size());
                }
                state.SetRegionCount(total_regions);
                utility::LogInfo("Frozen {} region(s). Total regions: {}.",
                                 processed.committed.size(), total_regions);
            }
        } catch (const std::exception& e) {
            utility::LogWarning("Region worker failed at frame {}: {}",
                                frame_id, e.what());
        }
        const auto region_ms = std::chrono::duration_cast<
                std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - region_t0)
                                       .count();
        if (region_ms > 500) {
            utility::LogInfo("Region worker frame {} took {} ms.", frame_id,
                             region_ms);
        }
        runtime.region_worker_busy.store(false);
        if (session_stopping && exit_flush_done) {
            break;
        }
    }
    if (runtime.freeze_tracker.PendingHoleyCount() > 0) {
        utility::LogWarning(
                "Region worker exiting with {} pending holey region(s).",
                runtime.freeze_tracker.PendingHoleyCount());
    }
}

/// Runs global relocalization off the SLAM thread so tracking stays responsive.
void RelocWorker(SlamRuntime& runtime) {
    while (true) {
        t::geometry::RGBDImage rgbd;
        core::Tensor intrinsic;
        core::Tensor last_stable;
        std::vector<int> candidate_ids;
        int frame_id = 0;
        int request_id = 0;
        float depth_scale = 1000.f;
        float depth_max = 3.f;
        bool used_fallback = false;
        {
            std::unique_lock<std::mutex> lock(runtime.reloc_mutex);
            runtime.reloc_cv.wait(lock, [&]() {
                return runtime.reloc_requested.load() ||
                       runtime.reloc_stop.load();
            });
            if (runtime.reloc_stop.load() && !runtime.reloc_requested.load()) {
                break;
            }
            runtime.reloc_requested.store(false);
            rgbd = runtime.pending_reloc_rgbd;
            intrinsic = runtime.pending_reloc_intrinsic;
            last_stable = runtime.pending_reloc_last_stable;
            candidate_ids = runtime.pending_reloc_candidate_ids;
            frame_id = runtime.pending_reloc_frame_id;
            request_id = runtime.pending_reloc_request_id;
            depth_scale = runtime.pending_reloc_depth_scale;
            depth_max = runtime.pending_reloc_depth_max;
            used_fallback = runtime.pending_reloc_used_fallback;
        }

        if (!runtime.reloc_keyframe_db) {
            continue;
        }

        runtime.reloc_worker_busy.store(true);
        const auto t0 = std::chrono::steady_clock::now();
        relocalization::RelocalizationAttempt attempt;
        try {
            attempt = relocalization::Relocalize(
                    rgbd, intrinsic, *runtime.reloc_keyframe_db, candidate_ids,
                    last_stable, depth_scale, depth_max, runtime.reloc_config,
                    runtime.reloc_device);
        } catch (const std::exception& e) {
            attempt = {};
            attempt.accepted = false;
            attempt.reject_reason = std::string("exception: ") + e.what();
            utility::LogWarning("Global relocalization threw at frame {}: {}",
                                frame_id, e.what());
        }
        const double elapsed_ms =
                std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t0)
                        .count();

        {
            std::lock_guard<std::mutex> lock(runtime.reloc_mutex);
            runtime.reloc_result = attempt;
            runtime.reloc_result_frame_id = frame_id;
            runtime.reloc_result_request_id = request_id;
            runtime.reloc_result_elapsed_ms = elapsed_ms;
            runtime.reloc_result_used_fallback = used_fallback;
            runtime.reloc_result_candidate_count =
                    static_cast<int>(candidate_ids.size());
            runtime.reloc_result_ready.store(true);
        }
        runtime.reloc_worker_busy.store(false);

        if (attempt.accepted) {
            utility::LogInfo(
                    "Global reloc finished in {:.0f} ms (frame {}, KF#{}, "
                    "fitness {:.3f}, candidates {}, fallback={}).",
                    elapsed_ms, frame_id, attempt.keyframe_id,
                    attempt.icp_fitness, candidate_ids.size(),
                    used_fallback ? 1 : 0);
        } else {
            utility::LogWarning(
                    "Global reloc finished in {:.0f} ms (frame {}, rejected: "
                    "{}, candidates {}, fallback={}).",
                    elapsed_ms, frame_id,
                    attempt.reject_reason.empty() ? "unknown"
                                                  : attempt.reject_reason,
                    candidate_ids.size(), used_fallback ? 1 : 0);
        }
    }
}

std::shared_ptr<geometry::LineSet> CreateRelocGuideLine(
        const core::Tensor& last_stable_T,
        const core::Tensor& current_T) {
    const Eigen::Vector3d a = relocalization::PoseTranslation(last_stable_T);
    const Eigen::Vector3d b = relocalization::PoseTranslation(current_T);
    auto line = std::make_shared<geometry::LineSet>();
    line->points_ = {a, b};
    line->lines_ = {Eigen::Vector2i(0, 1)};
    line->colors_ = {Eigen::Vector3d(0.961, 0.475, 0.000),
                     Eigen::Vector3d(0.961, 0.475, 0.000)};
    line->PaintUniformColor(Eigen::Vector3d(0.961, 0.475, 0.000));
    return line;
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

    relocalization::KeyframeDatabase keyframe_db(runtime.reloc_config);
    relocalization::MultiHypothesisTracker hypothesis_tracker(
            runtime.reloc_config);
    int last_global_reloc_frame = -1000;
    bool global_reloc_attempt_frame = false;
    bool global_reloc_recovering = false;
    bool hypotheses_initialized = false;
    relocalization::RelocalizationAttempt last_reloc_attempt;
    int last_keyframe_marker_publish = -1000;
    double last_reloc_elapsed_ms = 0.0;
    bool last_reloc_used_fallback = false;
    int last_reloc_candidate_count = 0;
    int applied_reloc_request_id = 0;

    runtime.reloc_keyframe_db = &keyframe_db;
    runtime.reloc_device = device;
    runtime.reloc_stop.store(false);
    runtime.reloc_requested.store(false);
    runtime.reloc_result_ready.store(false);
    std::thread reloc_thread;
    if (runtime.reloc_config.enabled) {
        reloc_thread = std::thread(RelocWorker, std::ref(runtime));
    }

    bool reloc_self_test_injected = false;
    bool reloc_self_test_global_ok = false;
    int reloc_self_test_inject_frame = -1;

    int frame_id = 0;
    while (!state.request_stop.load()) {
        if (!state.capture_enabled.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
            continue;
        }

        try {
        t::geometry::RGBDImage rgbd = (frame_id == 0) ? first : capture_frame();
        if (rgbd.IsEmpty()) {
            if (frame_id == 0) {
                // LogError throws/aborts the worker thread (0xc0000409).
                utility::LogWarning(
                        "Empty RGB-D frame at startup; stopping SLAM loop.");
                break;
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

        if (runtime.reloc_self_test.enabled && !reloc_self_test_injected &&
            !runtime.reloc_self_test_finished.load() &&
            frame_id >= runtime.reloc_self_test.warmup_frames &&
            keyframe_db.Size() >= runtime.reloc_self_test.min_keyframes) {
            reloc_self_test_injected = true;
            last_stable_T_frame_to_model = T_frame_to_model.Contiguous();
            T_frame_to_model = ApplyPoseDrift(
                    T_frame_to_model,
                    runtime.reloc_self_test.drift_translation_m,
                    runtime.reloc_self_test.drift_rotation_deg);
            tracking_was_unstable = true;
            hypotheses_initialized = false;
            consecutive_strong = 0;
            consecutive_tracking_failures = kLostTrackingThreshold + 1;
            last_global_reloc_frame = -100000;
            utility::LogInfo(
                    "RELOC_SELF_TEST: injected lost at frame {} (keyframes {}, "
                    "drift {:.2f} m, {:.1f} deg).",
                    frame_id, keyframe_db.Size(),
                    runtime.reloc_self_test.drift_translation_m,
                    runtime.reloc_self_test.drift_rotation_deg);
        }

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
        global_reloc_attempt_frame = false;

        // Apply finished async reloc results first (stale-safe via request id).
        if (runtime.reloc_config.enabled && runtime.reloc_result_ready.load()) {
            relocalization::RelocalizationAttempt attempt;
            int result_frame = 0;
            int result_request = 0;
            double elapsed_ms = 0.0;
            bool used_fallback = false;
            int candidate_count = 0;
            {
                std::lock_guard<std::mutex> lock(runtime.reloc_mutex);
                if (runtime.reloc_result_ready.load()) {
                    attempt = runtime.reloc_result;
                    result_frame = runtime.reloc_result_frame_id;
                    result_request = runtime.reloc_result_request_id;
                    elapsed_ms = runtime.reloc_result_elapsed_ms;
                    used_fallback = runtime.reloc_result_used_fallback;
                    candidate_count = runtime.reloc_result_candidate_count;
                    runtime.reloc_result_ready.store(false);
                }
            }
            if (result_request > applied_reloc_request_id) {
                applied_reloc_request_id = result_request;
                last_reloc_attempt = attempt;
                last_reloc_elapsed_ms = elapsed_ms;
                last_reloc_used_fallback = used_fallback;
                last_reloc_candidate_count = candidate_count;

                if (attempt.accepted) {
                    const double jump = relocalization::TranslationDistance(
                            attempt.T_live_to_world,
                            last_stable_T_frame_to_model);
                    if (jump > runtime.reloc_config.max_pose_jump_m) {
                        last_reloc_attempt.accepted = false;
                        last_reloc_attempt.reject_reason =
                                "pose jump too large (apply)";
                        utility::LogWarning(
                                "Global reloc result dropped at frame {} "
                                "(stale jump {:.2f} m).",
                                frame_id, jump);
                    } else {
                        T_frame_to_model =
                                attempt.T_live_to_world.Contiguous();
                        global_reloc_recovering = true;
                        consecutive_strong = 0;
                        {
                            std::lock_guard<std::mutex> model_lock(
                                    runtime.model_mutex);
                            model.UpdateFramePose(frame_id, T_frame_to_model);
                            model.SynthesizeModelFrame(
                                    raycast_frame, params.depth_scale, 0.1f,
                                    params.depth_max, params.trunc_multiplier,
                                    false);
                        }
                        if (!hypotheses_initialized) {
                            hypothesis_tracker.ResetOnLost(
                                    last_stable_T_frame_to_model,
                                    T_frame_to_model);
                            hypotheses_initialized = true;
                        }
                        hypothesis_tracker.AddOrReplace(
                                "global", T_frame_to_model, attempt.keyframe_id);
                        utility::LogInfo(
                                "Global relocalization accepted at frame {} "
                                "(from request frame {}, KF#{}, icp "
                                "{:.3f}, info {:.3f}, {:.0f} ms).",
                                frame_id, result_frame, attempt.keyframe_id,
                                attempt.icp_fitness, attempt.information_ratio,
                                elapsed_ms);
                        if (runtime.reloc_self_test.enabled &&
                            reloc_self_test_injected &&
                            !runtime.reloc_self_test_finished.load()) {
                            reloc_self_test_global_ok = true;
                            if (reloc_self_test_inject_frame < 0) {
                                reloc_self_test_inject_frame = result_frame;
                            }
                            utility::LogInfo(
                                    "RELOC_SELF_TEST: global reloc OK "
                                    "(keyframe {}, icp fitness {:.3f}).",
                                    attempt.keyframe_id, attempt.icp_fitness);
                        }
                    }
                } else if (runtime.reloc_self_test.enabled &&
                           reloc_self_test_injected &&
                           !runtime.reloc_self_test_finished.load()) {
                    // Async: keep waiting until timeout unless hard reject.
                    utility::LogWarning(
                            "RELOC_SELF_TEST: reloc rejected ({}); waiting "
                            "for retry/timeout.",
                            attempt.reject_reason.empty()
                                    ? "unknown"
                                    : attempt.reject_reason);
                }
            }
        }

        // Queue async reloc (query from last_stable, not drifted pose).
        if (runtime.reloc_config.enabled && tracking_was_unstable &&
            !global_reloc_recovering &&
            consecutive_tracking_failures > kLostTrackingThreshold &&
            keyframe_db.Size() > 0 &&
            frame_id - last_global_reloc_frame >=
                    runtime.reloc_config.retry_interval_frames &&
            !runtime.reloc_worker_busy.load() &&
            !runtime.reloc_requested.load()) {
            global_reloc_attempt_frame = true;
            last_global_reloc_frame = frame_id;
            const Eigen::Vector3d query_position =
                    relocalization::PoseTranslation(
                            last_stable_T_frame_to_model);
            const std::vector<double> live_histogram =
                    relocalization::ComputeDepthHistogram(
                            rgbd, params.depth_scale, params.depth_max);
            const relocalization::CandidateSelection selection =
                    keyframe_db.SelectCandidates(query_position, live_histogram,
                                                 runtime.reloc_config);
            utility::LogInfo(
                    "reloc candidates: {} (query=last_stable, fallback={}).",
                    selection.ids.size(), selection.used_fallback ? 1 : 0);
            if (selection.ids.empty()) {
                last_reloc_attempt = {};
                last_reloc_attempt.reject_reason = "no candidates";
                last_reloc_candidate_count = 0;
                last_reloc_used_fallback = false;
            } else {
                const int request_id = runtime.next_reloc_request_id++;
                {
                    std::lock_guard<std::mutex> lock(runtime.reloc_mutex);
                    runtime.pending_reloc_rgbd = rgbd.To(core::Device("CPU:0"));
                    runtime.pending_reloc_intrinsic = intrinsic;
                    runtime.pending_reloc_last_stable =
                            last_stable_T_frame_to_model.Contiguous();
                    runtime.pending_reloc_candidate_ids = selection.ids;
                    runtime.pending_reloc_frame_id = frame_id;
                    runtime.pending_reloc_request_id = request_id;
                    runtime.pending_reloc_depth_scale = params.depth_scale;
                    runtime.pending_reloc_depth_max = params.depth_max;
                    runtime.pending_reloc_used_fallback =
                            selection.used_fallback;
                    runtime.reloc_requested.store(true);
                }
                runtime.reloc_cv.notify_one();
                if (runtime.reloc_self_test.enabled &&
                    reloc_self_test_injected &&
                    reloc_self_test_inject_frame < 0) {
                    reloc_self_test_inject_frame = frame_id;
                }
            }
        }

        if (tracking_was_unstable &&
            consecutive_tracking_failures > kLostTrackingThreshold &&
            !hypotheses_initialized) {
            hypothesis_tracker.ResetOnLost(last_stable_T_frame_to_model,
                                           T_frame_to_model);
            hypotheses_initialized = true;
        }

        const bool prefer_f2f_bridge =
                tracking_was_unstable &&
                consecutive_tracking_failures > kLostTrackingThreshold &&
                (frame_id % kModelRetryIntervalWhenLost) != 0 &&
                !global_reloc_attempt_frame;

        if (frame_id > 0 && tracking_was_unstable && !prefer_f2f_bridge &&
            hypothesis_tracker.HasHypotheses()) {
            core::Tensor pose_before_hypotheses;
            {
                std::lock_guard<std::mutex> model_lock(runtime.model_mutex);
                pose_before_hypotheses = model.GetCurrentFramePose();
            }
            auto track_from_pose =
                    [&](const core::Tensor& seed_pose)
                    -> relocalization::MultiHypothesisTracker::TrackProbeResult {
                        relocalization::MultiHypothesisTracker::TrackProbeResult
                                probe;
                        try {
                            std::lock_guard<std::mutex> model_lock(
                                    runtime.model_mutex);
                            model.UpdateFramePose(frame_id, seed_pose);
                            model.SynthesizeModelFrame(
                                    raycast_frame, params.depth_scale, 0.1f,
                                    params.depth_max, params.trunc_multiplier,
                                    false);
                            auto result = model.TrackFrameToModel(
                                    input_frame, raycast_frame,
                                    params.depth_scale, params.depth_max,
                                    SafeOdometryDepthDiff(params.depth_diff),
                                    t::pipelines::odometry::Method::PointToPlane,
                                    odom_criteria);
                            probe.fitness = result.fitness_;
                            probe.transformation = result.transformation_;
                            model.UpdateFramePose(frame_id,
                                                  pose_before_hypotheses);
                        } catch (const std::exception& e) {
                            probe.fitness = 0.0;
                            probe.transformation = identity_pose;
                            try {
                                std::lock_guard<std::mutex> model_lock(
                                        runtime.model_mutex);
                                model.UpdateFramePose(frame_id,
                                                      pose_before_hypotheses);
                            } catch (...) {
                            }
                            if (!IsOdometrySingularError(e)) {
                                utility::LogWarning(
                                        "Hypothesis probe failed at frame {}: "
                                        "{}",
                                        frame_id, e.what());
                            }
                        }
                        return probe;
                    };
            auto eval = hypothesis_tracker.EvaluateAndPickBest(
                    last_stable_T_frame_to_model, track_from_pose);
            if (eval.updated && eval.best_fitness >= kWeakFitnessMin) {
                T_frame_to_model = eval.best_T.Contiguous();
                utility::LogInfo(
                        "Hypothesis '{}' selected at frame {} (fitness {:.3f}).",
                        eval.best_label, frame_id, eval.best_fitness);
            }
        }

        const int recovery_streak_required =
                tracking_was_unstable
                        ? (global_reloc_recovering
                                   ? runtime.reloc_config.verify_strong_streak
                                   : kStrongStreakAfterLost)
                        : 1;

        if (frame_id > 0 && !prefer_f2f_bridge) {
            {
                std::lock_guard<std::mutex> model_lock(runtime.model_mutex);
                model.UpdateFramePose(frame_id, T_frame_to_model);
                model.SynthesizeModelFrame(
                        raycast_frame, params.depth_scale, 0.1f,
                        params.depth_max, params.trunc_multiplier, false);
            }
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
                // Motion gate uses net pose vs anchor + hash growth (not
                // summed frame-to-frame jitter).

                if (tracking_tier == TrackingTier::kStrong) {
                    core::Tensor candidate_T_frame_to_model =
                            T_frame_to_model.Matmul(track_transform);
                    ++consecutive_strong;
                    consecutive_tracking_failures = 0;
                    if (!tracking_was_unstable) {
                        T_frame_to_model = candidate_T_frame_to_model;
                        integrate = !hash_near_full;
                    } else if (tracking_was_unstable &&
                               consecutive_strong >= recovery_streak_required &&
                               IsRecoveryPoseStable(track_fitness,
                                                    track_translation,
                                                    track_rotation)) {
                        T_frame_to_model = candidate_T_frame_to_model;
                        integrate = !hash_near_full;
                        tracking_was_unstable = false;
                        global_reloc_recovering = false;
                        hypotheses_initialized = false;
                        consecutive_f2f_bridges = 0;
                        if (lost_camera_marker_visible) {
                            state.SetLostCameraMarker(nullptr, false);
                            state.SetCurrentCameraMarker(nullptr, false);
                            state.SetRelocGuideLine(nullptr, false);
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

        {
            std::lock_guard<std::mutex> model_lock(runtime.model_mutex);
            try {
                model.UpdateFramePose(frame_id, T_frame_to_model);
                if (integrate && !hash_near_full) {
                    model.Integrate(input_frame, params.depth_scale,
                                    params.depth_max, params.trunc_multiplier);
                    ++integrated_frames;
                }
                model.SynthesizeModelFrame(
                        raycast_frame, params.depth_scale, 0.1f,
                        params.depth_max, params.trunc_multiplier, false);
            } catch (const std::exception& e) {
                integrate = false;
                utility::LogWarning(
                        "Integrate/raycast failed at frame {}: {}", frame_id,
                        e.what());
            }
        }

        if (integrate) {
            last_stable_T_frame_to_model = T_frame_to_model.Contiguous();
            if (runtime.reloc_config.enabled) {
                keyframe_db.MaybeAddKeyframe(
                        rgbd, intrinsic, T_frame_to_model, frame_id,
                        params.depth_scale, params.depth_max, device);
            }
            if (lost_camera_marker_visible) {
                state.SetLostCameraMarker(nullptr, false);
                state.SetCurrentCameraMarker(nullptr, false);
                state.SetRelocGuideLine(nullptr, false);
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
                                       last_stable_T_frame_to_model,
                                       kLastStableCameraColor),
                    true);
            lost_camera_marker_visible = true;
            // Hide keyframe spheres — they clutter the view and do not mark
            // the return target; orange last-stable + blue current do.
            state.SetShowKeyframeMarkers(false);
            state.SetKeyframeMarkers({});
            utility::LogInfo(
                    "Tracking lost. Orange=last stable, blue=current pose; "
                    "return toward the orange camera frustum.");
        }
        if (consecutive_tracking_failures > kLostTrackingThreshold) {
            state.SetLostCameraMarker(
                    CreateCameraMarker(width, height, intrinsic_eigen,
                                       last_stable_T_frame_to_model,
                                       kLastStableCameraColor),
                    true);
            state.SetCurrentCameraMarker(
                    CreateCameraMarker(width, height, intrinsic_eigen,
                                       T_frame_to_model, kCurrentCameraColor),
                    true);
            state.SetPoseDiffText(
                    BuildPoseDiffText(last_stable_T_frame_to_model,
                                      T_frame_to_model),
                    true);
            state.SetRelocGuideLine(
                    CreateRelocGuideLine(last_stable_T_frame_to_model,
                                         T_frame_to_model),
                    true);
        } else if (lost_camera_marker_visible) {
            state.SetCurrentCameraMarker(nullptr, false);
            state.SetRelocGuideLine(nullptr, false);
        }

        if (params.regions.enabled &&
            ShouldCheckRegions(frame_id, params.regions.interval) &&
            !runtime.region_worker_busy.load() &&
            consecutive_tracking_failures <= kLostTrackingThreshold) {
            if (!runtime.have_motion_anchor) {
                runtime.motion_anchor_T = T_frame_to_model.Contiguous();
                runtime.motion_anchor_hash = hash_size_before;
                runtime.have_motion_anchor = true;
            }

            object_mesh::CameraMotionSample motion_sample;
            object_mesh::RelativePoseMetrics(
                    runtime.motion_anchor_T, T_frame_to_model,
                    motion_sample.net_translation_m,
                    motion_sample.net_rotation_deg);
            motion_sample.hash_delta =
                    hash_size_before - runtime.motion_anchor_hash;
            if (motion_sample.hash_delta < 0) {
                motion_sample.hash_delta = 0;
            }
            motion_sample.frame_id = frame_id;

            object_mesh::SegmentationConfig motion_cfg;
            motion_cfg.require_camera_motion =
                    params.regions.require_camera_motion;
            motion_cfg.min_motion_translation_m =
                    params.regions.min_motion_translation_m;
            motion_cfg.min_motion_rotation_deg =
                    params.regions.min_motion_rotation_deg;
            motion_cfg.min_hash_delta_blocks =
                    params.regions.min_hash_delta_blocks;
            motion_cfg.region_motion_warmup_frames =
                    params.regions.region_motion_warmup_frames;

            const bool camera_moved =
                    object_mesh::CameraMovedEnough(motion_cfg, motion_sample);
            runtime.camera_moved_for_regions = camera_moved;
            runtime.motion_window_translation_m =
                    motion_sample.net_translation_m;
            runtime.motion_window_rotation_deg = motion_sample.net_rotation_deg;

            if (!camera_moved) {
                if (frame_id < params.regions.region_motion_warmup_frames) {
                    utility::LogInfo(
                            "Region check skipped: motion warmup "
                            "(frame {}/{}, net t={:.4f} m, r={:.2f} deg, "
                            "dhash={}).",
                            frame_id, params.regions.region_motion_warmup_frames,
                            motion_sample.net_translation_m,
                            motion_sample.net_rotation_deg,
                            motion_sample.hash_delta);
                } else {
                    utility::LogInfo(
                            "Region check skipped: camera nearly stationary "
                            "(net t={:.4f} m, r={:.2f} deg, dhash={}).",
                            motion_sample.net_translation_m,
                            motion_sample.net_rotation_deg,
                            motion_sample.hash_delta);
                }
            } else {
            const float region_weight = params.regions.extract_weight;
            try {
                t::geometry::PointCloud region_pcd;
                {
                    std::lock_guard<std::mutex> model_lock(runtime.model_mutex);
                    // 2-pass extract (-1) avoids CUDA buffer overrun when the map
                    // exceeds kMaxRegionSegmentationPoints; downsample in worker.
                    region_pcd = model.ExtractPointCloudExcludingFrozen(
                            region_weight, -1);
                    region_pcd = region_pcd.To(core::Device("CPU:0"));
                }
                const int64_t before_filter =
                        region_pcd.HasPointPositions()
                                ? region_pcd.GetPointPositions().GetLength()
                                : 0;
                if (before_filter > 0) {
                    region_pcd = object_mesh::FilterPointCloudByCameraDistance(
                            region_pcd, T_frame_to_model,
                            params.regions.depth_min_m,
                            params.regions.depth_max_m);
                }
                const int64_t after_filter =
                        region_pcd.HasPointPositions()
                                ? region_pcd.GetPointPositions().GetLength()
                                : 0;
                utility::LogInfo(
                        "Region surface filtered: {} -> {} points "
                        "(weight={}, depth=[{:.2f},{:.2f}] m).",
                        before_filter, after_filter, region_weight,
                        params.regions.depth_min_m,
                        params.regions.depth_max_m);
                if (after_filter > 0) {
                    {
                        std::lock_guard<std::mutex> lock(runtime.region_mutex);
                        runtime.pending_region_pcd = std::move(region_pcd);
                        runtime.pending_extract_weight = region_weight;
                        runtime.pending_frame_id = frame_id;
                        runtime.last_region_surface_pcd =
                                runtime.pending_region_pcd;
                        runtime.last_region_extract_weight = region_weight;
                        runtime.last_region_frame_id = frame_id;
                        runtime.have_last_region_surface = true;
                        runtime.region_requested.store(true);
                    }
                    runtime.region_cv.notify_one();
                    utility::LogInfo(
                            "Region segmentation queued at frame {} ({} points, "
                            "net t={:.4f} m, r={:.2f} deg, dhash={}).",
                            frame_id, after_filter,
                            motion_sample.net_translation_m,
                            motion_sample.net_rotation_deg,
                            motion_sample.hash_delta);
                    runtime.motion_anchor_T = T_frame_to_model.Contiguous();
                    runtime.motion_anchor_hash = hash_size_before;
                    runtime.have_motion_anchor = true;
                    runtime.motion_window_translation_m = 0.0;
                    runtime.motion_window_rotation_deg = 0.0;
                }
            } catch (const std::exception& e) {
                utility::LogWarning("Region extract skipped at frame {}: {}",
                                    frame_id, e.what());
            }
            }  // camera_moved
        }

        if (ShouldRefreshDisplay(frame_id, params.update_interval)) {
            const float weight = ExtractWeightThreshold(frame_id);
            const int extract_budget =
                    GetExtractPointBudget(params.estimated_points,
                                          hash_size_before);
            try {
                t::geometry::PointCloud pcd_t;
                {
                    std::lock_guard<std::mutex> model_lock(runtime.model_mutex);
                    // Always 2-pass (-1) with regions: frozen filters + one-pass
                    // under-estimate can abort via CUDA extract fail-fast.
                    const int extract_arg =
                            (params.regions.enabled || hash_size_before > 2000)
                                    ? -1
                                    : extract_budget;
                    if (params.regions.enabled) {
                        pcd_t = model.ExtractPointCloudExcludingFrozen(
                                weight, extract_arg);
                    } else {
                        pcd_t = model.ExtractPointCloud(weight, extract_arg);
                    }
                }
                auto pcd =
                        std::make_shared<geometry::PointCloud>(pcd_t.ToLegacy());
                if (!pcd->IsEmpty()) {
                    state.SetPointCloud(pcd);
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
                      " | f2f " + std::to_string(consecutive_f2f_bridges);
            if (runtime.reloc_config.enabled) {
                status += " | KF " + std::to_string(keyframe_db.Size()) + "/" +
                          std::to_string(keyframe_db.Capacity());
            }
            if (global_reloc_recovering) {
                status += " | GLOBAL RECOVERING";
            }
        } else if (frame_id > 0 && tracking_tier != TrackingTier::kStrong) {
            status += " | tracking " + std::string(TrackingTierName(tracking_tier));
        }
        state.SetStatus(status);

        std::string reloc_detail;
        if (runtime.reloc_config.enabled) {
            const double d_last = relocalization::TranslationDistance(
                    T_frame_to_model, last_stable_T_frame_to_model);
            std::ostringstream detail_ss;
            detail_ss << std::fixed << std::setprecision(2);
            if (consecutive_tracking_failures > kLostTrackingThreshold) {
                detail_ss << "Lost | d_last_stable=" << d_last << "m | ";
            }
            detail_ss << "KF " << keyframe_db.Size() << "/"
                      << keyframe_db.Capacity();
            if (last_reloc_candidate_count > 0 ||
                !last_reloc_attempt.reject_reason.empty() ||
                last_reloc_attempt.accepted) {
                detail_ss << " | last: ";
                if (last_reloc_attempt.accepted) {
                    detail_ss << "accepted KF#"
                              << last_reloc_attempt.keyframe_id;
                } else {
                    detail_ss << "rejected("
                              << (last_reloc_attempt.reject_reason.empty()
                                          ? "pending"
                                          : last_reloc_attempt.reject_reason)
                              << ")";
                }
                if (last_reloc_elapsed_ms > 0.0) {
                    detail_ss << " " << static_cast<int>(last_reloc_elapsed_ms)
                              << "ms";
                }
                detail_ss << " cand=" << last_reloc_candidate_count;
                if (last_reloc_used_fallback) {
                    detail_ss << " fallback";
                }
            }
            if (global_reloc_recovering) {
                detail_ss << " | confirming";
            }
            if (runtime.reloc_worker_busy.load()) {
                detail_ss << " | reloc busy";
            }
            reloc_detail = detail_ss.str();
        }
        state.SetRelocDetail(reloc_detail);

        // Keyframe spheres are debug-only (toggle). Reloc guidance uses
        // orange last-stable + blue current camera frustums instead.
        if (runtime.reloc_config.enabled && state.ShowKeyframeMarkers() &&
            frame_id - last_keyframe_marker_publish >= params.update_interval) {
            std::vector<Eigen::Vector3d> markers;
            for (const auto& entry : keyframe_db.SnapshotEntries()) {
                markers.push_back(entry.capture_position);
            }
            state.SetKeyframeMarkers(std::move(markers));
            last_keyframe_marker_publish = frame_id;
        }

        utility::LogInfo(
                "SLAM frame {} | hash blocks {}/{} | tier {} | f2f bridge {}",
                frame_id, hash_size_before, hash_capacity,
                TrackingTierName(tracking_tier), consecutive_f2f_bridges);

        if (runtime.reloc_self_test.enabled && reloc_self_test_global_ok &&
            !runtime.reloc_self_test_finished.load()) {
            const double pose_err_m = relocalization::TranslationDistance(
                    T_frame_to_model, last_stable_T_frame_to_model);
            const core::Tensor pose_delta = T_frame_to_model.Matmul(
                    last_stable_T_frame_to_model.Inverse());
            const double pose_err_deg =
                    relocalization::PoseRotationAngleDeg(pose_delta);

            const bool pose_ok =
                    pose_err_m <= runtime.reloc_self_test.max_pose_error_m &&
                    pose_err_deg <= runtime.reloc_self_test.max_pose_error_deg;
            // Async reloc: accept pose-corrected recovery even while strong
            // streak is still confirming (global_ok already proves reloc).
            const bool recovered =
                    !tracking_was_unstable ||
                    (global_reloc_recovering && pose_ok &&
                     consecutive_strong >= 1);
            const bool timed_out =
                    reloc_self_test_inject_frame >= 0 &&
                    frame_id >= reloc_self_test_inject_frame +
                                         runtime.reloc_self_test
                                                 .recovery_timeout_frames;

            if (recovered && pose_ok) {
                runtime.reloc_self_test_passed.store(true);
                runtime.reloc_self_test_finished.store(true);
                state.reloc_self_test_finished.store(true);
                utility::LogInfo(
                        "RELOC_SELF_TEST: PASS (recovered at frame {}, pose err "
                        "{:.3f} m, {:.1f} deg).",
                        frame_id, pose_err_m, pose_err_deg);
                state.request_stop.store(true);
            } else if (timed_out) {
                runtime.reloc_self_test_passed.store(false);
                runtime.reloc_self_test_finished.store(true);
                state.reloc_self_test_finished.store(true);
                utility::LogWarning(
                        "RELOC_SELF_TEST: FAIL (recovery timeout at frame {}, "
                        "pose err {:.3f} m, unstable={}, global_ok={}).",
                        frame_id, pose_err_m, tracking_was_unstable,
                        reloc_self_test_global_ok);
                state.request_stop.store(true);
            }
        }

        if (runtime.reloc_self_test_finished.load()) {
            break;
        }

        prev_rgbd = rgbd;
        ++frame_id;
        } catch (const std::exception& e) {
            utility::LogWarning("SLAM worker exception at frame {}: {}",
                                frame_id, e.what());
            ++frame_id;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } catch (...) {
            utility::LogWarning("SLAM worker unknown exception at frame {}.",
                                frame_id);
            ++frame_id;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    runtime.model_ready.store(false);
    runtime.region_stop.store(true);
    runtime.region_cv.notify_all();
    runtime.reloc_stop.store(true);
    runtime.reloc_cv.notify_all();
    if (reloc_thread.joinable()) {
        reloc_thread.join();
    }
    runtime.reloc_keyframe_db = nullptr;

    if (runtime.reloc_self_test.enabled) {
        state.slam_finished.store(true);
        return;
    }

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
        object_mesh::ClampVertexColors(*final_mesh);
        io::WriteTriangleMesh("scene_mesh.ply", *final_mesh);
        utility::LogInfo("Saved scene_mesh.ply ({} vertices).",
                         final_mesh->vertices_.size());
    } catch (const std::exception& e) {
        utility::LogWarning("Failed to extract/save scene: {}", e.what());
    }

    if (params.regions.enabled) {
        std::lock_guard<std::mutex> lock(runtime.records_mutex);
        object_mesh::WriteRegionsJson(params.regions.output_dir, runtime.region_records);
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
    if (utility::ProgramOptionExists(argc, argv, "--region_min_readiness")) {
        params.regions.min_readiness = static_cast<float>(
                utility::GetProgramOptionAsDouble(
                        argc, argv, "--region_min_readiness",
                        params.regions.min_readiness));
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_max_holey_defer")) {
        params.regions.max_holey_defer = utility::GetProgramOptionAsInt(
                argc, argv, "--region_max_holey_defer",
                params.regions.max_holey_defer);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_no_holey_defer")) {
        params.regions.defer_holey = false;
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_max_void_ratio")) {
        params.regions.max_void_ratio = utility::GetProgramOptionAsDouble(
                argc, argv, "--region_max_void_ratio",
                params.regions.max_void_ratio);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_max_void_blob")) {
        params.regions.max_void_blob_cells = utility::GetProgramOptionAsInt(
                argc, argv, "--region_max_void_blob",
                params.regions.max_void_blob_cells);
    }
    if (utility::ProgramOptionExists(argc, argv,
                                     "--region_max_boundary_void")) {
        params.regions.max_boundary_void_ratio =
                utility::GetProgramOptionAsDouble(
                        argc, argv, "--region_max_boundary_void",
                        params.regions.max_boundary_void_ratio);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_extract_weight")) {
        params.regions.extract_weight = static_cast<float>(
                utility::GetProgramOptionAsDouble(
                        argc, argv, "--region_extract_weight",
                        params.regions.extract_weight));
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_depth_min")) {
        params.regions.depth_min_m = utility::GetProgramOptionAsDouble(
                argc, argv, "--region_depth_min", params.regions.depth_min_m);
    }
    bool region_depth_max_set = false;
    if (utility::ProgramOptionExists(argc, argv, "--region_depth_max")) {
        params.regions.depth_max_m = utility::GetProgramOptionAsDouble(
                argc, argv, "--region_depth_max", params.regions.depth_max_m);
        region_depth_max_set = true;
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_max_extent")) {
        params.regions.max_cluster_extent_m =
                utility::GetProgramOptionAsDouble(
                        argc, argv, "--region_max_extent",
                        params.regions.max_cluster_extent_m);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_min_motion_m")) {
        params.regions.min_motion_translation_m =
                utility::GetProgramOptionAsDouble(
                        argc, argv, "--region_min_motion_m",
                        params.regions.min_motion_translation_m);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_min_motion_deg")) {
        params.regions.min_motion_rotation_deg =
                utility::GetProgramOptionAsDouble(
                        argc, argv, "--region_min_motion_deg",
                        params.regions.min_motion_rotation_deg);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_min_hash_delta")) {
        params.regions.min_hash_delta_blocks = utility::GetProgramOptionAsInt(
                argc, argv, "--region_min_hash_delta",
                params.regions.min_hash_delta_blocks);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_motion_warmup")) {
        params.regions.region_motion_warmup_frames =
                utility::GetProgramOptionAsInt(
                        argc, argv, "--region_motion_warmup",
                        params.regions.region_motion_warmup_frames);
    }
    if (utility::ProgramOptionExists(argc, argv,
                                     "--region_allow_stationary_mesh")) {
        params.regions.require_camera_motion = false;
    }
    if (!region_depth_max_set) {
        params.regions.depth_max_m =
                std::min(2.5, static_cast<double>(params.depth_max));
    }

    SlamRuntime runtime;
    runtime.reloc_config =
            relocalization::RelocalizationConfigForProfile(profile);
    if (utility::ProgramOptionExists(argc, argv, "--global_reloc")) {
        runtime.reloc_config.enabled = utility::GetProgramOptionAsInt(
                argc, argv, "--global_reloc", 1) != 0;
    }
    if (utility::ProgramOptionExists(argc, argv, "--keyframe_interval")) {
        runtime.reloc_config.keyframe_interval_frames =
                utility::GetProgramOptionAsInt(
                        argc, argv, "--keyframe_interval",
                        runtime.reloc_config.keyframe_interval_frames);
    }
    if (utility::ProgramOptionExists(argc, argv, "--keyframe_max")) {
        runtime.reloc_config.max_keyframes = utility::GetProgramOptionAsInt(
                argc, argv, "--keyframe_max",
                runtime.reloc_config.max_keyframes);
    }
    if (utility::ProgramOptionExists(argc, argv, "--reloc_method")) {
        runtime.reloc_config.global_method =
                utility::GetProgramOptionAsString(argc, argv, "--reloc_method",
                                                runtime.reloc_config.global_method);
    }
    if (utility::ProgramOptionExists(argc, argv, "--reloc_retry_interval")) {
        runtime.reloc_config.retry_interval_frames =
                utility::GetProgramOptionAsInt(
                        argc, argv, "--reloc_retry_interval",
                        runtime.reloc_config.retry_interval_frames);
    }
    if (utility::ProgramOptionExists(argc, argv, "--reloc_candidate_radius")) {
        runtime.reloc_config.candidate_radius_m =
                utility::GetProgramOptionAsDouble(
                        argc, argv, "--reloc_candidate_radius",
                        runtime.reloc_config.candidate_radius_m);
    }
    if (utility::ProgramOptionExists(argc, argv, "--reloc_min_fitness")) {
        runtime.reloc_config.min_fitness = utility::GetProgramOptionAsDouble(
                argc, argv, "--reloc_min_fitness",
                runtime.reloc_config.min_fitness);
    }
    if (utility::ProgramOptionExists(argc, argv, "--reloc_max_pose_jump")) {
        runtime.reloc_config.max_pose_jump_m =
                utility::GetProgramOptionAsDouble(
                        argc, argv, "--reloc_max_pose_jump",
                        runtime.reloc_config.max_pose_jump_m);
    }
    if (utility::ProgramOptionExists(argc, argv, "--reloc_ransac_iter")) {
        runtime.reloc_config.ransac_max_iter = utility::GetProgramOptionAsInt(
                argc, argv, "--reloc_ransac_iter",
                runtime.reloc_config.ransac_max_iter);
    }
    if (utility::ProgramOptionExists(argc, argv, "--reloc_self_test")) {
        runtime.reloc_self_test.enabled = true;
        runtime.reloc_config.enabled = true;
        runtime.reloc_config.retry_interval_frames = 1;
        runtime.reloc_config.keyframe_interval_frames = 10;
        runtime.reloc_config.keyframe_min_translation = 0.05;
        runtime.reloc_config.keyframe_min_rotation_deg = 5.0;
        // Async reloc needs a few frames after inject for queue+result+track.
        runtime.reloc_self_test.recovery_timeout_frames = 20;
        utility::LogInfo(
                "RELOC_SELF_TEST mode: warmup {} frames, inject drift, run "
                "global reloc, then exit.",
                runtime.reloc_self_test.warmup_frames);
    }

    const std::string device_code =
            utility::GetProgramOptionAsString(argc, argv, "--device", "CUDA:0");
    const core::Device device(device_code);
    utility::LogInfo("Compute device: {}", device.ToString());
    utility::LogInfo("SLAM profile: {} (voxel {:.4f} m, depth_max {:.1f} m, "
                     "blocks {}, est. points {})",
                     profile, params.voxel_size, params.depth_max,
                     params.block_count, params.estimated_points);
    if (runtime.reloc_config.enabled) {
        utility::LogInfo(
                "Global relocalization: keyframes max {}, interval {}, "
                "method {}, retry {} frames.",
                runtime.reloc_config.max_keyframes,
                runtime.reloc_config.keyframe_interval_frames,
                runtime.reloc_config.global_method,
                runtime.reloc_config.retry_interval_frames);
    }

    std::unique_ptr<tio::RealSenseSensor> rs;
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

        // USB / previous-process release can cancel the first StartCapture
        // (HRESULT 0x800703e3). Recreate the sensor and retry with backoff.
        constexpr int kMaxAttempts = 5;
        constexpr auto kRetryDelay = std::chrono::milliseconds(1500);
        bool capture_started = false;
        for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
            rs = std::make_unique<tio::RealSenseSensor>();
            try {
                if (!rs->InitSensor(rs_cfg, 0, record_bag)) {
                    utility::LogWarning(
                            "RealSense InitSensor failed (attempt {}/{}). "
                            "Check that the camera is connected and not used "
                            "by another application.",
                            attempt, kMaxAttempts);
                } else if (!rs->StartCapture(!record_bag.empty())) {
                    utility::LogWarning(
                            "RealSense StartCapture failed (attempt {}/{}).",
                            attempt, kMaxAttempts);
                    rs->StopCapture();
                } else {
                    capture_started = true;
                    utility::LogInfo("{}", rs->GetMetadata().ToString());
                    depth_scale =
                            static_cast<float>(rs->GetMetadata().depth_scale_);
                    params.depth_scale = depth_scale;
                    cam_intrinsic = rs->GetMetadata().intrinsics_;
                    intrinsic = core::eigen_converter::EigenMatrixToTensor(
                            rs->GetMetadata().intrinsics_.intrinsic_matrix_);
                    if (attempt > 1) {
                        utility::LogInfo(
                                "RealSense capture started on attempt {}/{}.",
                                attempt, kMaxAttempts);
                    }
                    break;
                }
            } catch (const std::exception& e) {
                utility::LogWarning(
                        "RealSense startup attempt {}/{} failed: {}", attempt,
                        kMaxAttempts, e.what());
                try {
                    rs->StopCapture();
                } catch (...) {
                }
            }
            rs.reset();
            if (attempt < kMaxAttempts) {
                utility::LogInfo(
                        "Retrying RealSense open in {} ms (attempt {}/{})...",
                        kRetryDelay.count(), attempt + 1, kMaxAttempts);
                std::this_thread::sleep_for(kRetryDelay);
            }
        }
        if (!capture_started || !rs) {
            utility::LogWarning(
                    "RealSense startup failed after {} attempts. Unplug/replug "
                    "the camera, close RealSense Viewer / other capture apps, "
                    "then retry.",
                    kMaxAttempts);
            return 1;
        }
        // RealSense pipelines need a short warmup before the first frame.
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        capture_frame = [rs_ptr = rs.get(), align_streams]() -> t::geometry::RGBDImage {
            return rs_ptr->CaptureFrame(true, align_streams);
        };
    }

    DisplayState display_state;
    display_state.regions_enabled.store(params.regions.enabled);
    std::thread region_thread;
    if (params.regions.enabled) {
        region_thread = std::thread(RegionWorker, std::ref(runtime),
                                    std::cref(params), std::ref(display_state));
    }
    std::thread slam_thread(SlamWorker, capture_frame, intrinsic, cam_intrinsic,
                            device, params, std::ref(runtime),
                            std::ref(display_state));

    if (runtime.reloc_self_test.enabled) {
        utility::LogInfo("RELOC_SELF_TEST: headless mode (GUI skipped).");
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::seconds(120);
        while (!runtime.reloc_self_test_finished.load() &&
               std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        display_state.request_stop.store(true);
        runtime.region_stop.store(true);
        runtime.region_cv.notify_all();
        slam_thread.join();
        if (params.regions.enabled && region_thread.joinable()) {
            region_thread.join();
        }
        if (!use_bag) {
            if (rs) {
                rs->StopCapture();
            }
        } else {
            bag_reader.Close();
        }
        return runtime.reloc_self_test_finished.load() &&
                               runtime.reloc_self_test_passed.load()
                       ? 0
                       : 2;
    }

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
        if (rs) {
            rs->StopCapture();
        }
    } else {
        bag_reader.Close();
    }

    if (runtime.reloc_self_test.enabled) {
        return runtime.reloc_self_test_finished.load() &&
                       runtime.reloc_self_test_passed.load()
                       ? 0
                       : 2;
    }

    return 0;
}
