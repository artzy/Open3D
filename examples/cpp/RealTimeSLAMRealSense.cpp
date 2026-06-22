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
    int estimated_points = 4000000;
    float depth_max = 3.f;
    float depth_diff = 0.07f;
    float depth_scale = 1000.f;
    int update_interval = 15;
    int odom_iter_coarse = 4;
    int odom_iter_mid = 2;
    int odom_iter_fine = 1;
};

SlamParams GetProfile(const std::string& profile) {
    SlamParams p;
    if (profile == "low") {
        p.voxel_size = 0.008f;
        p.block_count = 16384;
        p.estimated_points = 1500000;
        p.depth_max = 2.f;
        p.update_interval = 20;
        return p;
    }
    if (profile == "high") {
        p.block_count = 50000;
        p.estimated_points = 8000000;
        p.depth_max = 5.f;
        p.update_interval = 10;
        p.odom_iter_coarse = 6;
        p.odom_iter_mid = 3;
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
    utility::LogInfo("");
    utility::LogInfo("SLAM:");
    utility::LogInfo("    [--device CUDA:0|CPU:0] Compute device (default: CUDA:0).");
    utility::LogInfo("    [--profile low|medium|high]  Memory/quality preset (default: medium).");
    utility::LogInfo("    [--update_interval N]   Refresh 3D view every N frames (default: profile).");
    utility::LogInfo("");
    utility::LogInfo("Output (on window close / ESC):");
    utility::LogInfo("    scene.ply        Reconstructed point cloud");
    utility::LogInfo("    trajectory.log   Camera trajectory");
    utility::LogInfo("");
}


std::string DefaultConfigPath() {
    const std::string rel = "examples/test_data/rs_d415_slam.json";
    if (utility::filesystem::FileExists(rel)) {
        return rel;
    }
    return "";
}

bool TrackingAccepted(const t::pipelines::odometry::OdometryResult& result) {
    core::Tensor translation =
            result.transformation_.Slice(0, 0, 3).Slice(1, 3, 4);
    const double translation_norm = std::sqrt(
            (translation * translation).Sum({0, 1}).Item<double>());
    return result.fitness_ >= 0.1 && translation_norm < 0.15;
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
    Model model(params.voxel_size, 16, params.block_count, T_frame_to_model,
                device);

    Frame input_frame(height, width, intrinsic, device);
    Frame raycast_frame(height, width, intrinsic, device);

    auto trajectory = std::make_shared<camera::PinholeCameraTrajectory>();
    const std::vector<t::pipelines::odometry::OdometryConvergenceCriteria>
            odom_criteria = {params.odom_iter_coarse, params.odom_iter_mid,
                             params.odom_iter_fine};

    int frame_id = 0;
    while (!state.request_stop.load()) {
        t::geometry::RGBDImage rgbd = (frame_id == 0) ? first : capture_frame();
        if (rgbd.IsEmpty()) {
            if (frame_id == 0) {
                utility::LogError("Empty RGB-D frame at startup.");
            }
            break;
        }

        rgbd = rgbd.To(device);
        input_frame.SetDataFromImage("depth", rgbd.depth_);
        input_frame.SetDataFromImage("color", rgbd.color_);

        bool tracking_success = true;
        if (frame_id > 0) {
            auto result = model.TrackFrameToModel(
                    input_frame, raycast_frame, params.depth_scale,
                    params.depth_max, params.depth_diff,
                    t::pipelines::odometry::Method::PointToPlane, odom_criteria);
            if (TrackingAccepted(result)) {
                T_frame_to_model =
                        T_frame_to_model.Matmul(result.transformation_);
            } else {
                tracking_success = false;
                utility::LogWarning(
                        "Tracking weak at frame {} (fitness {:.3f}). Keeping "
                        "previous pose.",
                        frame_id, result.fitness_);
            }
        }

        model.UpdateFramePose(frame_id, T_frame_to_model);
        if (tracking_success) {
            model.Integrate(input_frame, params.depth_scale, params.depth_max,
                            params.trunc_multiplier);
        }
        model.SynthesizeModelFrame(raycast_frame, params.depth_scale, 0.1f,
                                   params.depth_max, params.trunc_multiplier,
                                   false);

        camera::PinholeCameraParameters cam_params;
        cam_params.intrinsic_ = cam_intrinsic;
        cam_params.extrinsic_ =
                core::eigen_converter::TensorToEigenMatrixXd(T_frame_to_model);
        trajectory->parameters_.push_back(cam_params);

        if (frame_id % params.update_interval == 0) {
            auto pcd_t = model.ExtractPointCloud(3.f, params.estimated_points);
            auto pcd = std::make_shared<geometry::PointCloud>(pcd_t.ToLegacy());
            state.SetPointCloud(pcd);
        }

        state.SetStatus("Frame " + std::to_string(frame_id) + " | blocks " +
                        std::to_string(model.GetHashMap().Size()));
        utility::LogInfo("SLAM frame {} | hash blocks {}", frame_id,
                         model.GetHashMap().Size());
        ++frame_id;
    }

    auto final_pcd_t = model.ExtractPointCloud(3.f, params.estimated_points);
    auto final_pcd =
            std::make_shared<geometry::PointCloud>(final_pcd_t.ToLegacy());
    state.SetPointCloud(final_pcd);

    io::WritePointCloud("scene.ply", *final_pcd);
    io::WritePinholeCameraTrajectory("trajectory.log", *trajectory);
    utility::LogInfo("Saved scene.ply and trajectory.log ({} frames).",
                     frame_id);

    state.slam_finished.store(true);
}

}  // namespace

int main(int argc, char* argv[]) {
    utility::SetVerbosityLevel(utility::VerbosityLevel::Info);

    if (argc <= 1 ||
        utility::ProgramOptionExistsAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
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

    const std::string device_code =
            utility::GetProgramOptionAsString(argc, argv, "--device", "CUDA:0");
    const core::Device device(device_code);
    utility::LogInfo("Compute device: {}", device.ToString());
    utility::LogInfo("SLAM profile: {}", profile);

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

    visualization::VisualizerWithKeyCallback vis;
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

    vis.RegisterKeyCallback(
            GLFW_KEY_ESCAPE, [&](visualization::Visualizer*) {
                shared.request_stop.store(true);
                return false;
            });

    while (!shared.request_stop.load() && !shared.slam_finished.load()) {
        std::shared_ptr<geometry::PointCloud> updated;
        if (shared.TakeUpdate(updated)) {
            render_pcd = updated;
            if (!geometry_added && render_pcd) {
                vis.AddGeometry(render_pcd);
                geometry_added = true;
            } else if (geometry_added && render_pcd) {
                vis.UpdateGeometry(render_pcd);
            }
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
