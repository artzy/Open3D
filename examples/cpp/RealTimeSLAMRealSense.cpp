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
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "open3d/Open3D.h"

namespace {

using namespace open3d;
namespace tio = open3d::t::io;

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
    utility::LogInfo("Indoor room scan tips:");
    utility::LogInfo("    Walk slowly (~0.3 m/s), keep 30%% overlap between views.");
    utility::LogInfo("    Use --profile high --depth_max 8 for large rooms.");
    utility::LogInfo("    scene_mesh.ply is the watertight mesh; scene.ply is points.");
    utility::LogInfo("");
    utility::LogInfo("Output (on window close / ESC):");
    utility::LogInfo("    scene.ply        Reconstructed point cloud");
    utility::LogInfo("    scene_mesh.ply   Reconstructed triangle mesh");
    utility::LogInfo("    trajectory.log   Camera trajectory (strong tracking only)");
    utility::LogInfo("");
    utility::LogInfo("Note: sudden fast motion may pause integration; move slowly");
    utility::LogInfo("      back to the scanned area to resume reconstruction.");
    utility::LogInfo("");
}


std::string DefaultConfigPath() {
    const std::string rel = "examples/test_data/rs_d415_slam.json";
    if (utility::filesystem::FileExists(rel)) {
        return rel;
    }
    return "";
}

// Tracking tiers: strict pose+integrate vs pose bridge vs reject outlier jumps.
static constexpr double kPoseFitnessMin = 0.12;
static constexpr double kPoseTranslationMax = 0.15;
static constexpr double kWeakFitnessMin = 0.08;
static constexpr double kWeakTranslationMax = 0.30;
static constexpr double kOutlierTranslation = 0.50;
static constexpr float kOdometryHuberDelta = 0.05f;
static constexpr int kLostTrackingThreshold = 5;
static constexpr int kStrongStreakAfterLost = 2;
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

TrackingTier ClassifyTracking(double fitness, double translation) {
    if (translation >= kOutlierTranslation) {
        return TrackingTier::kOutlier;
    }
    if (fitness >= kPoseFitnessMin && translation < kPoseTranslationMax) {
        return TrackingTier::kStrong;
    }
    if (fitness >= kWeakFitnessMin && translation < kWeakTranslationMax) {
        return TrackingTier::kWeak;
    }
    return TrackingTier::kFail;
}

bool FrameToFrameBridgeAccepted(double fitness, double translation) {
    return fitness >= kWeakFitnessMin && translation < kWeakTranslationMax &&
           translation < kOutlierTranslation;
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

class SharedState {
public:
    void SetPointCloud(std::shared_ptr<geometry::PointCloud> pcd) {
        std::lock_guard<std::mutex> lock(mutex_);
        display_pcd_ = std::move(pcd);
        has_update_ = true;
    }

    bool TakeUpdate(std::shared_ptr<geometry::PointCloud>& out) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_update_) {
            return false;
        }
        out = display_pcd_;
        has_update_ = false;
        return out != nullptr;
    }

    void SetStatus(const std::string& status) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_ = status;
    }

    std::string Status() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return status_;
    }

    std::atomic<bool> request_stop{false};
    std::atomic<bool> slam_finished{false};

private:
    mutable std::mutex mutex_;
    std::shared_ptr<geometry::PointCloud> display_pcd_;
    bool has_update_ = false;
    std::string status_;
};

class RealTimeVisualizer : public visualization::VisualizerWithKeyCallback {
public:
    void SetStatusTitle(const std::string& status) {
        window_name_ = "RealTimeSLAMRealSense | " + status;
        UpdateWindowTitle();
    }
};

void SlamWorker(std::function<t::geometry::RGBDImage()> capture_frame,
                const core::Tensor& intrinsic,
                const camera::PinholeCameraIntrinsic& cam_intrinsic,
                const core::Device& device,
                SlamParams params,
                SharedState& state) {
    using t::pipelines::slam::Frame;
    using t::pipelines::slam::Model;

    t::geometry::RGBDImage first = capture_frame();
    if (first.IsEmpty()) {
        utility::LogError("Failed to capture the first RGB-D frame.");
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

    t::geometry::RGBDImage prev_rgbd;
    int consecutive_tracking_failures = 0;
    int consecutive_strong = 0;
    int f2f_bridge_successes = 0;
    int integrated_frames = 0;
    bool tracking_was_unstable = false;
    int empty_capture_retries = 0;
    bool hash_full_warned = false;

    int frame_id = 0;
    while (!state.request_stop.load()) {
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
                    "Voxel hash nearly full ({}/{}). Pausing integration and "
                    "tracking. Finish the scan or use --profile high / "
                    "--block_count.",
                    hash_size_before, hash_capacity);
            hash_full_warned = true;
        }

        bool integrate = (frame_id == 0) && !hash_near_full;
        TrackingTier tracking_tier = TrackingTier::kInit;

        if (frame_id > 0 && !hash_near_full) {
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

                auto result = run_model_tracking(params.depth_diff);
                double track_fitness = result.fitness_;
                double track_translation =
                        TranslationNorm(result.transformation_);
                tracking_tier =
                        ClassifyTracking(track_fitness, track_translation);

                if (tracking_tier == TrackingTier::kFail &&
                    track_fitness < kWeakFitnessMin) {
                    result = run_model_tracking(params.depth_diff * 2.0f);
                    track_fitness = result.fitness_;
                    track_translation =
                            TranslationNorm(result.transformation_);
                    tracking_tier =
                            ClassifyTracking(track_fitness, track_translation);
                }

                if (tracking_tier == TrackingTier::kStrong) {
                    T_frame_to_model =
                            T_frame_to_model.Matmul(result.transformation_);
                    ++consecutive_strong;
                    consecutive_tracking_failures = 0;
                    if (!tracking_was_unstable) {
                        integrate = true;
                    } else if (consecutive_strong >= kStrongStreakAfterLost) {
                        integrate = true;
                        tracking_was_unstable = false;
                        utility::LogInfo(
                                "Tracking restabilized at frame {} — "
                                "resuming integration.",
                                frame_id);
                    }
                } else {
                    consecutive_strong = 0;
                    tracking_was_unstable = true;
                    ++consecutive_tracking_failures;
                    const char* tier_name = TrackingTierName(tracking_tier);
                    utility::LogWarning(
                            "Tracking {} for frame {}, fitness: {:.3f}, "
                            "translation: {:.3f}. Skipping integration.",
                            tier_name, frame_id, track_fitness,
                            track_translation);

                    if (tracking_tier != TrackingTier::kOutlier &&
                        !prev_rgbd.IsEmpty()) {
                        try {
                            auto f2f_result =
                                    t::pipelines::odometry::RGBDOdometryMultiScale(
                                            rgbd, prev_rgbd, intrinsic,
                                            identity_pose, params.depth_scale,
                                            params.depth_max, odom_criteria,
                                            t::pipelines::odometry::Method::
                                                    PointToPlane,
                                            t::pipelines::odometry::
                                                    OdometryLossParams(
                                                            SafeOdometryDepthDiff(
                                                                    params
                                                                            .depth_diff)));
                            const double f2f_fitness = f2f_result.fitness_;
                            const double f2f_translation =
                                    TranslationNorm(f2f_result.transformation_);
                            if (FrameToFrameBridgeAccepted(f2f_fitness,
                                                           f2f_translation)) {
                                T_frame_to_model = T_frame_to_model.Matmul(
                                        f2f_result.transformation_);
                                ++f2f_bridge_successes;
                                utility::LogDebug(
                                        "Frame-to-frame bridge frame {}: "
                                        "fitness {:.3f}, translation {:.3f}.",
                                        frame_id, f2f_fitness, f2f_translation);
                            }
                        } catch (const std::exception& f2f_e) {
                            if (!IsOdometrySingularError(f2f_e)) {
                                utility::LogWarning(
                                        "Frame-to-frame odometry failed at "
                                        "frame {}: {}",
                                        frame_id, f2f_e.what());
                            }
                        }
                    }
                }
            } catch (const std::exception& e) {
                tracking_tier = TrackingTier::kFail;
                consecutive_strong = 0;
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
        } else if (frame_id > 0 && hash_near_full) {
            tracking_tier = TrackingTier::kFail;
            integrate = false;
            ++consecutive_tracking_failures;
        } else {
            consecutive_strong = 1;
        }

        model.UpdateFramePose(frame_id, T_frame_to_model);
        if (integrate && !hash_near_full) {
            model.Integrate(input_frame, params.depth_scale, params.depth_max,
                            params.trunc_multiplier);
            ++integrated_frames;
        }
        model.SynthesizeModelFrame(raycast_frame, params.depth_scale, 0.1f,
                                   params.depth_max, params.trunc_multiplier,
                                   false);

        if (integrate) {
            camera::PinholeCameraParameters cam_params;
            cam_params.intrinsic_ = cam_intrinsic;
            cam_params.extrinsic_ = core::eigen_converter::TensorToEigenMatrixXd(
                    T_frame_to_model);
            trajectory->parameters_.push_back(cam_params);
        }

        if (ShouldRefreshDisplay(frame_id, params.update_interval)) {
            const float weight = ExtractWeightThreshold(frame_id);
            const int extract_budget =
                    GetExtractPointBudget(params.estimated_points,
                                          hash_size_before);
            try {
                auto pcd_t = model.ExtractPointCloud(weight, extract_budget);
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
        if (hash_near_full) {
            status += " | HASH FULL";
        } else if (consecutive_tracking_failures > kLostTrackingThreshold) {
            status += " | LOST " + std::to_string(consecutive_tracking_failures) +
                      " — move slowly back";
        } else if (frame_id > 0 && tracking_tier != TrackingTier::kStrong) {
            status += " | tracking " + std::string(TrackingTierName(tracking_tier));
        }
        state.SetStatus(status);

        utility::LogInfo("SLAM frame {} | hash blocks {}/{} | tier {}", frame_id,
                         hash_size_before, hash_capacity,
                         TrackingTierName(tracking_tier));
        prev_rgbd = rgbd;
        ++frame_id;
    }

    const int64_t final_hash_size = model.GetHashMap().Size();
    utility::LogInfo(
            "Extracting final scene (2-pass point count, hash blocks {})...",
            final_hash_size);

    try {
        // estimated_number=-1: two-pass extract, no truncation warning.
        auto final_pcd_t = model.ExtractPointCloud(3.f, -1);
        auto final_pcd =
                std::make_shared<geometry::PointCloud>(final_pcd_t.ToLegacy());
        ClampPointColors(*final_pcd);
        state.SetPointCloud(final_pcd);
        io::WritePointCloud("scene.ply", *final_pcd);
        utility::LogInfo("Saved scene.ply ({} points).", final_pcd->points_.size());

        auto final_mesh_t = model.ExtractTriangleMesh(3.f, -1);
        auto final_mesh =
                std::make_shared<geometry::TriangleMesh>(final_mesh_t.ToLegacy());
        ClampVertexColors(*final_mesh);
        io::WriteTriangleMesh("scene_mesh.ply", *final_mesh);
        utility::LogInfo("Saved scene_mesh.ply ({} vertices).",
                         final_mesh->vertices_.size());
    } catch (const std::exception& e) {
        utility::LogError("Failed to extract/save scene: {}", e.what());
    }

    io::WritePinholeCameraTrajectory("trajectory.log", *trajectory);

    const double integrate_ratio =
            frame_id > 0 ? 100.0 * integrated_frames / frame_id : 100.0;
    utility::LogInfo(
            "Saved scene.ply, scene_mesh.ply, trajectory.log "
            "({} SLAM frames, {} integrated ({:.1f}%), {} strong poses, {} "
            "f2f bridges).",
            frame_id, integrated_frames, integrate_ratio,
            trajectory->parameters_.size(), f2f_bridge_successes);
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

        rs.ListDevices();
        rs.InitSensor(rs_cfg, 0, record_bag);
        utility::LogInfo("{}", rs.GetMetadata().ToString());
        depth_scale = static_cast<float>(rs.GetMetadata().depth_scale_);
        params.depth_scale = depth_scale;
        cam_intrinsic = rs.GetMetadata().intrinsics_;
        intrinsic = core::eigen_converter::EigenMatrixToTensor(
                rs.GetMetadata().intrinsics_.intrinsic_matrix_);
        rs.StartCapture(!record_bag.empty());

        capture_frame = [&rs, align_streams]() -> t::geometry::RGBDImage {
            return rs.CaptureFrame(true, align_streams);
        };
    }

    SharedState shared;
    std::thread slam_thread(SlamWorker, capture_frame, intrinsic, cam_intrinsic,
                            device, params, std::ref(shared));

    RealTimeVisualizer vis;
    if (!vis.CreateVisualizerWindow("RealTimeSLAMRealSense", 1280, 720)) {
        utility::LogError("Failed to create visualizer window.");
        shared.request_stop.store(true);
        slam_thread.join();
        if (!use_bag) {
            rs.StopCapture();
        } else {
            bag_reader.Close();
        }
        return 1;
    }

    vis.GetRenderOption().background_color_ = Eigen::Vector3d(0.1, 0.1, 0.1);
    vis.GetRenderOption().point_size_ = 2.0;

    bool geometry_added = false;
    std::shared_ptr<geometry::PointCloud> render_pcd;
    std::string last_window_status;

    vis.RegisterKeyCallback(
            GLFW_KEY_ESCAPE, [&](visualization::Visualizer*) {
                shared.request_stop.store(true);
                return false;
            });

    while (!shared.request_stop.load() && !shared.slam_finished.load()) {
        std::shared_ptr<geometry::PointCloud> updated;
        if (shared.TakeUpdate(updated) && updated && !updated->IsEmpty()) {
            if (!geometry_added) {
                render_pcd = updated;
                vis.AddGeometry(render_pcd);
                vis.ResetViewPoint(true);
                geometry_added = true;
            } else {
                // Visualizer matches geometry by pointer; reuse render_pcd and
                // copy new points/colors into the registered object.
                render_pcd->points_ = updated->points_;
                render_pcd->colors_ = updated->colors_;
                render_pcd->normals_ = updated->normals_;
                vis.UpdateGeometry(render_pcd);
            }
        }

        const std::string window_status = shared.Status();
        if (window_status != last_window_status) {
            vis.SetStatusTitle(window_status);
            last_window_status = window_status;
        }

        if (!vis.PollEvents()) {
            shared.request_stop.store(true);
            break;
        }
        vis.UpdateRender();
    }

    shared.request_stop.store(true);
    slam_thread.join();

    if (!use_bag) {
        rs.StopCapture();
    } else {
        bag_reader.Close();
    }

    if (geometry_added && render_pcd) {
        std::shared_ptr<geometry::PointCloud> final_update;
        if (shared.TakeUpdate(final_update) && final_update &&
            !final_update->IsEmpty()) {
            render_pcd->points_ = final_update->points_;
            render_pcd->colors_ = final_update->colors_;
            render_pcd->normals_ = final_update->normals_;
        }
        vis.UpdateGeometry(render_pcd);
        vis.PollEvents();
        vis.UpdateRender();
    }

    utility::LogInfo("Press ESC or close the window to exit.");
    while (vis.PollEvents()) {
        vis.UpdateRender();
    }
    vis.DestroyVisualizerWindow();

    return 0;
}
