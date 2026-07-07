// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>

#include "OnlineSLAMUtil.h"
#include "open3d/Open3D.h"

namespace {

using open3d::examples::online_slam::GetPerfPreset;
using open3d::examples::online_slam::GetSlamProfile;

void PrintHelp() {
    using namespace open3d;
    PrintOpen3DVersion();

    // clang-format off
    utility::LogInfo("Usage:");
    utility::LogInfo("    > OnlineSLAMRealSense [options]");
    utility::LogInfo("Basic options:");
    utility::LogInfo("    [-V]");
    utility::LogInfo("    [--use_bag_file /path/to/realsense_video_file.bag] If not provided, it will look for realsense sensor.");
    utility::LogInfo("    [-l|--list-devices]");
    utility::LogInfo("    [--align]              Align depth to color (default).");
    utility::LogInfo("    [--no-align]           Skip live depth alignment (saves ~33ms/frame).");
    utility::LogInfo("    [--record rgbd_video_file.bag]");
    utility::LogInfo("    [-c|--config rs-config.json]");
    utility::LogInfo("    [--device CUDA:0]");
    utility::LogInfo("    [--profile low|medium|high]  SLAM memory profile (default: medium).");
    utility::LogInfo("        low    - 16k blocks, 1.5M points, depth_max 2m (~0.8 GB TSDF)");
    utility::LogInfo("        medium - 40k blocks, 6M points, depth_max 3m (~1.9 GB TSDF)");
    utility::LogInfo("        high   - 50k blocks, 8M points, depth_max 5m (~2.4 GB TSDF)");
    utility::LogInfo("    [--perf fast|balanced|quality]  SLAM speed preset (default: fast).");
    utility::LogInfo("        fast     - odom 3/2/1, GUI every 3 frames, raycast color off");
    utility::LogInfo("        balanced - odom 4/2/1, GUI every 2 frames, raycast color on");
    utility::LogInfo("        quality  - odom 6/3/1, GUI every frame, raycast color on");
    // clang-format on
    utility::LogInfo("");
}

// Apply RealSense metadata hints (plan section 4). Auto-adjust only when
// --profile was explicitly set; otherwise log recommendations only.
void ApplyMetadataHints(std::unordered_map<std::string, double>& params,
                        const open3d::t::io::RGBDVideoMetadata& metadata,
                        bool profile_explicit,
                        bool is_bag,
                        const std::string& visual_preset,
                        const std::string& slam_profile) {
    using namespace open3d;
    const int depth_w = metadata.width_;
    const int depth_h = metadata.height_;
    const int depth_pixels = depth_w * depth_h;

    if (depth_pixels >= 1024 * 768) {
        utility::LogWarning(
                "High depth resolution ({}x{}). Consider 480p config or "
                "--no-align to stay within 30fps (~33ms/frame).",
                depth_w, depth_h);
        if (profile_explicit) {
            params["update_interval"] += 25.0;
            utility::LogInfo(
                    "Adjusted update_interval to {} (--profile set).",
                    static_cast<int>(params["update_interval"]));
        }
    }

    if (!is_bag) {
        if (metadata.device_name_.find("L515") != std::string::npos &&
            visual_preset.find("SHORT_RANGE") != std::string::npos &&
            params["depth_max"] > 3.0) {
            utility::LogInfo(
                    "L515 SHORT_RANGE preset: depth_max <= 3.0 m recommended "
                    "(current: {:.1f} m).",
                    params["depth_max"]);
        }
    } else if (std::abs(metadata.depth_scale_ - 1000.0) < 1.0 &&
               depth_pixels <= 640 * 480 && slam_profile != "low") {
        utility::LogInfo(
                "Small bag dataset detected (depth_scale=1000, {}x{}). "
                "Consider --profile low for lower VRAM usage.",
                depth_w, depth_h);
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    using namespace open3d;
    using namespace open3d::visualization;

    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    if (argc < 2 ||
        utility::ProgramOptionExistsAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }

    std::string bag_file;
    bool use_bag_file = false;
    if (utility::ProgramOptionExists(argc, argv, "--use_bag_file")) {
        bag_file =
                utility::GetProgramOptionAsString(argc, argv, "--use_bag_file");
    }
    if (!bag_file.empty()) {
        use_bag_file = true;
    }

    if (utility::ProgramOptionExists(argc, argv, "--list-devices") ||
        utility::ProgramOptionExists(argc, argv, "-l")) {
        t::io::RealSenseSensor::ListDevices();
        return 0;
    }
    if (utility::ProgramOptionExists(argc, argv, "-V")) {
        utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    } else {
        utility::SetVerbosityLevel(utility::VerbosityLevel::Info);
    }

    bool align_streams = true;
    std::string config_file, record_to_bag_file;

    if (utility::ProgramOptionExists(argc, argv, "-c")) {
        config_file = utility::GetProgramOptionAsString(argc, argv, "-c");
    } else if (utility::ProgramOptionExists(argc, argv, "--config")) {
        config_file = utility::GetProgramOptionAsString(argc, argv, "--config");
    }
    if (utility::ProgramOptionExists(argc, argv, "--no-align")) {
        align_streams = false;
    } else if (utility::ProgramOptionExists(argc, argv, "--align")) {
        align_streams = true;
    }
    if (utility::ProgramOptionExists(argc, argv, "--record")) {
        record_to_bag_file =
                utility::GetProgramOptionAsString(argc, argv, "--record");
    }

    const bool profile_explicit =
            utility::ProgramOptionExists(argc, argv, "--profile");
    std::string profile =
            utility::GetProgramOptionAsString(argc, argv, "--profile", "medium");
    if (profile != "low" && profile != "medium" && profile != "high") {
        utility::LogError(
                "Unknown --profile '{}'. Use low, medium, or high.", profile);
        return 1;
    }

    std::string perf =
            utility::GetProgramOptionAsString(argc, argv, "--perf", "fast");
    if (perf != "fast" && perf != "balanced" && perf != "quality") {
        utility::LogError(
                "Unknown --perf '{}'. Use fast, balanced, or quality.", perf);
        return 1;
    }

    std::string device_code =
            utility::GetProgramOptionAsString(argc, argv, "--device", "CUDA:0");
    core::Device device(device_code);
    utility::LogInfo("Using device {}.", device_code);
    utility::LogInfo("SLAM memory profile: {}.", profile);
    utility::LogInfo("SLAM performance preset: {}.", perf);

    std::function<t::geometry::RGBDImage(const int)> get_rgbd_image_input;
    core::Tensor intrinsic_t;
    std::unordered_map<std::string, double> default_params = GetSlamProfile(profile);
    for (const auto& it : GetPerfPreset(perf)) {
        default_params[it.first] = it.second;
    }
    t::io::RealSenseSensor rs;
    t::io::RSBagReader bag_reader;

    if (!use_bag_file) {
        if (config_file.empty()) {
            utility::LogInfo(
                    "Tip: use -c examples/test_data/rs_slam_lowmem.json for "
                    "540p/480p capture (lower VRAM and faster processing).");
        }

        t::io::RealSenseSensorConfig rs_cfg;
        if (!config_file.empty()) {
            open3d::io::ReadIJsonConvertible(config_file, rs_cfg);
        }

        std::string visual_preset;
        const auto preset_it = rs_cfg.config_.find("visual_preset");
        if (preset_it != rs_cfg.config_.end()) {
            visual_preset = preset_it->second;
        }

        rs.ListDevices();
        rs.InitSensor(rs_cfg, 0, record_to_bag_file);
        utility::LogInfo("{}", rs.GetMetadata().ToString());
        if (!align_streams) {
            utility::LogInfo(
                    "Live depth-to-color alignment disabled (--no-align).");
        }
        rs.StartCapture();

        get_rgbd_image_input = [&](const size_t idx) {
            return rs.CaptureFrame(true, align_streams);
        };
        intrinsic_t = core::eigen_converter::EigenMatrixToTensor(
                rs.GetMetadata().intrinsics_.intrinsic_matrix_);
        default_params["depth_scale"] = rs.GetMetadata().depth_scale_;

        ApplyMetadataHints(default_params, rs.GetMetadata(), profile_explicit,
                           false, visual_preset, profile);
    } else {
        bag_reader.Open(bag_file);

        if (!bag_reader.IsOpened()) {
            utility::LogError("Unable to open {}", bag_file);
            return 1;
        }
        const auto bag_metadata = bag_reader.GetMetadata();
        utility::LogInfo("{}", bag_metadata.ToString());

        get_rgbd_image_input = [&](const int idx) {
            if (!bag_reader.IsEOF()) {
                return bag_reader.NextFrame();
            }
            return open3d::t::geometry::RGBDImage();
        };

        intrinsic_t = core::eigen_converter::EigenMatrixToTensor(
                bag_metadata.intrinsics_.intrinsic_matrix_);
        default_params["depth_scale"] = bag_metadata.depth_scale_;

        ApplyMetadataHints(default_params, bag_metadata, profile_explicit,
                           true, "", profile);
    }

    auto& app = gui::Application::GetInstance();
    app.Initialize();
    auto mono =
            app.AddFont(gui::FontDescription(gui::FontDescription::MONOSPACE));
    app.AddWindow(std::make_shared<examples::online_slam::ReconstructionWindow>(
            get_rgbd_image_input, intrinsic_t, default_params, device, mono,
            use_bag_file));
    app.Run();

    if (!use_bag_file) {
        rs.StopCapture();
    } else {
        bag_reader.Close();
    }

    return 0;
}
