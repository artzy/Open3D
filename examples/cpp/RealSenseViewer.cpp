// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------
//
// Simple RealSense RGB-D visualization (live camera or .bag playback).
//
// No SLAM or recording — only displays live data.
//   --mode pointcloud   Colored 3D point cloud (default)
//   --mode rgbd         Separate color and depth 2D windows
//
// Build: BUILD_LIBREALSENSE=ON
//
// Examples:
//   RealSenseViewer -l
//   RealSenseViewer -c examples/test_data/rs_d415_slam.json
//   RealSenseViewer --input capture.bag --mode pointcloud
//

#include <functional>
#include <memory>
#include <string>

#include "open3d/Open3D.h"

namespace {

using namespace open3d;
namespace tio = open3d::t::io;

constexpr float kDefaultDepthMax = 3.f;

void PrintHelp() {
    PrintOpen3DVersion();
    utility::LogInfo("Usage:");
    utility::LogInfo("    > RealSenseViewer [options]");
    utility::LogInfo("");
    utility::LogInfo("Input:");
    utility::LogInfo("    [-l|--list-devices]       List connected RealSense devices.");
    utility::LogInfo("    [-c|--config PATH]         RealSense JSON (live camera).");
    utility::LogInfo("                               Default: examples/test_data/rs_d415_slam.json");
    utility::LogInfo("    [--input PATH.bag]        Play a RealSense bag file.");
    utility::LogInfo("    [--default_dataset NAME]  Bag when no camera/input (jack_jack).");
    utility::LogInfo("");
    utility::LogInfo("Display:");
    utility::LogInfo("    [--mode pointcloud|rgbd]  Visualization mode (default: pointcloud).");
    utility::LogInfo("    [--align]                 Align depth to color (default for pointcloud).");
    utility::LogInfo("    [--no-align]              Skip depth alignment.");
    utility::LogInfo("    [--depth_max M]           Max depth in meters (default: {:.1f}).",
                     kDefaultDepthMax);
    utility::LogInfo("");
    utility::LogInfo("Press [ESC] to exit. Bag playback: [SPACE] pause/resume.");
    utility::LogInfo("");
}

std::string DefaultConfigPath() {
    const std::string rel = "examples/test_data/rs_d415_slam.json";
    if (utility::filesystem::FileExists(rel)) {
        return rel;
    }
    return "";
}

std::string ResolveBagPath(int argc, char* argv[]) {
    std::string input =
            utility::GetProgramOptionAsString(argc, argv, "--input", "");
    if (!input.empty()) {
        if (!utility::filesystem::FileExists(input)) {
            utility::LogError("Bag file not found: {}", input);
        }
        return input;
    }

    if (!utility::ProgramOptionExists(argc, argv, "--default_dataset")) {
        return "";
    }

    const std::string dataset =
            utility::GetProgramOptionAsString(argc, argv, "--default_dataset",
                                              "jack_jack");
    utility::LogInfo("Using default dataset: {}", dataset);
    if (dataset == "jack_jack") {
        data::JackJackL515Bag bag_dataset;
        return bag_dataset.GetPath();
    }
    if (dataset == "sample_l515") {
        data::SampleL515Bag bag_dataset;
        return bag_dataset.GetPath();
    }
    utility::LogError(
            "Unknown --default_dataset '{}'. Use jack_jack or sample_l515.",
            dataset);
    return "";
}

geometry::PointCloud RgbdToPointCloud(const t::geometry::RGBDImage& rgbd,
                                      const core::Tensor& intrinsic,
                                      float depth_scale,
                                      float depth_max) {
    auto pcd_t = t::geometry::PointCloud::CreateFromRGBDImage(
            rgbd, intrinsic, core::Tensor::Eye(4, core::Float32,
                                               core::Device("CPU:0")),
            depth_scale, depth_max);
    return pcd_t.ToLegacy();
}

void RunPointCloudViewer(
        const std::function<t::geometry::RGBDImage()>& capture,
        const core::Tensor& intrinsic,
        float depth_scale,
        float depth_max,
        bool is_bag) {
    bool flag_exit = false;
    bool flag_play = true;

    visualization::VisualizerWithKeyCallback vis;
    vis.RegisterKeyCallback(
            GLFW_KEY_ESCAPE, [&](visualization::Visualizer*) {
                flag_exit = true;
                return false;
            });
    if (is_bag) {
        vis.RegisterKeyCallback(
                GLFW_KEY_SPACE, [&](visualization::Visualizer*) {
                    flag_play = !flag_play;
                    utility::LogInfo(flag_play ? "Playback resumed."
                                               : "Playback paused.");
                    return false;
                });
    }

    if (!vis.CreateVisualizerWindow("RealSenseViewer | Point Cloud", 1280, 720)) {
        utility::LogError("Failed to create visualizer window.");
    }
    vis.GetRenderOption().background_color_ = Eigen::Vector3d(0.1, 0.1, 0.1);
    vis.GetRenderOption().point_size_ = 2.0;

    bool geometry_added = false;
    std::shared_ptr<geometry::PointCloud> render_pcd;
    int frame_id = 0;

    while (!flag_exit) {
        if (!is_bag || flag_play) {
            t::geometry::RGBDImage rgbd = capture();
            if (rgbd.IsEmpty()) {
                if (is_bag) {
                    utility::LogInfo("End of bag file.");
                    break;
                }
                continue;
            }

            geometry::PointCloud pcd =
                    RgbdToPointCloud(rgbd, intrinsic, depth_scale, depth_max);
            render_pcd = std::make_shared<geometry::PointCloud>(std::move(pcd));

            if (!geometry_added) {
                vis.AddGeometry(render_pcd);
                geometry_added = true;
            } else {
                vis.UpdateGeometry(render_pcd);
            }

            if (frame_id++ % 30 == 0) {
                utility::LogInfo("Frame {} | points {}", frame_id - 1,
                                 render_pcd->points_.size());
            }
        }

        if (!vis.PollEvents()) {
            flag_exit = true;
            break;
        }
        vis.UpdateRender();
    }

    vis.DestroyVisualizerWindow();
}

void RunRgbdViewer(const std::function<t::geometry::RGBDImage()>& capture,
                   bool is_bag) {
    bool flag_exit = false;
    bool flag_play = true;

    visualization::VisualizerWithKeyCallback depth_vis, color_vis;
    auto callback_exit = [&](visualization::Visualizer*) {
        flag_exit = true;
        return false;
    };
    depth_vis.RegisterKeyCallback(GLFW_KEY_ESCAPE, callback_exit);
    color_vis.RegisterKeyCallback(GLFW_KEY_ESCAPE, callback_exit);
    if (is_bag) {
        auto toggle = [&](visualization::Visualizer*) {
            flag_play = !flag_play;
            return false;
        };
        depth_vis.RegisterKeyCallback(GLFW_KEY_SPACE, toggle);
        color_vis.RegisterKeyCallback(GLFW_KEY_SPACE, toggle);
    }

    using legacyImage = geometry::Image;
    std::shared_ptr<legacyImage> depth_ptr, color_ptr;
    geometry::RGBDImage im_rgbd;
    bool geometry_added = false;
    int frame_id = 0;

    while (!flag_exit) {
        if (!is_bag || flag_play) {
            im_rgbd = capture().ToLegacy();
            if (im_rgbd.IsEmpty()) {
                if (is_bag) {
                    break;
                }
                continue;
            }

            depth_ptr = std::shared_ptr<legacyImage>(
                    &im_rgbd.depth_, [](legacyImage*) {});
            color_ptr = std::shared_ptr<legacyImage>(
                    &im_rgbd.color_, [](legacyImage*) {});

            if (!geometry_added) {
                if (!depth_vis.CreateVisualizerWindow(
                            "RealSenseViewer | Depth", depth_ptr->width_,
                            depth_ptr->height_, 15, 50) ||
                    !depth_vis.AddGeometry(depth_ptr) ||
                    !color_vis.CreateVisualizerWindow(
                            "RealSenseViewer | Color", color_ptr->width_,
                            color_ptr->height_, 675, 50) ||
                    !color_vis.AddGeometry(color_ptr)) {
                    utility::LogError("Window creation failed.");
                }
                geometry_added = true;
            } else {
                depth_vis.UpdateGeometry();
                color_vis.UpdateGeometry();
            }

            if (frame_id++ % 30 == 0) {
                utility::LogInfo("Frame {}", frame_id - 1);
            }
        }

        if (geometry_added) {
            depth_vis.PollEvents();
            color_vis.PollEvents();
            depth_vis.UpdateRender();
            color_vis.UpdateRender();
        }
    }

    if (geometry_added) {
        depth_vis.DestroyVisualizerWindow();
        color_vis.DestroyVisualizerWindow();
    }
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

    const std::string mode =
            utility::GetProgramOptionAsString(argc, argv, "--mode", "pointcloud");
    if (mode != "pointcloud" && mode != "rgbd") {
        utility::LogError("Unknown --mode '{}'. Use pointcloud or rgbd.", mode);
        return 1;
    }

    const float depth_max = static_cast<float>(utility::GetProgramOptionAsDouble(
            argc, argv, "--depth_max", kDefaultDepthMax));

    bool align_streams = (mode == "pointcloud");
    if (utility::ProgramOptionExists(argc, argv, "--no-align")) {
        align_streams = false;
    } else if (utility::ProgramOptionExists(argc, argv, "--align")) {
        align_streams = true;
    }

    const std::string bag_path = ResolveBagPath(argc, argv);
    const bool use_bag = !bag_path.empty();

    std::function<t::geometry::RGBDImage()> capture;
    core::Tensor intrinsic;
    float depth_scale = 1000.f;

    tio::RealSenseSensor rs;
    tio::RSBagReader bag_reader;

    if (use_bag) {
        bag_reader.Open(bag_path);
        if (!bag_reader.IsOpened()) {
            utility::LogError("Unable to open bag: {}", bag_path);
            return 1;
        }
        const auto meta = bag_reader.GetMetadata();
        utility::LogInfo("{}", meta.ToString());
        depth_scale = static_cast<float>(meta.depth_scale_);
        intrinsic = core::eigen_converter::EigenMatrixToTensor(
                meta.intrinsics_.intrinsic_matrix_);
        capture = [&bag_reader]() -> t::geometry::RGBDImage {
            if (bag_reader.IsEOF()) {
                return t::geometry::RGBDImage();
            }
            return bag_reader.NextFrame();
        };
    } else {
        std::string config_file;
        if (utility::ProgramOptionExists(argc, argv, "-c")) {
            config_file = utility::GetProgramOptionAsString(argc, argv, "-c");
        } else if (utility::ProgramOptionExists(argc, argv, "--config")) {
            config_file =
                    utility::GetProgramOptionAsString(argc, argv, "--config");
        } else {
            config_file = DefaultConfigPath();
            if (!config_file.empty()) {
                utility::LogInfo("Using default config: {}", config_file);
            }
        }

        tio::RealSenseSensorConfig rs_cfg;
        if (!config_file.empty()) {
            if (!io::ReadIJsonConvertible(config_file, rs_cfg)) {
                utility::LogError("Failed to read config: {}", config_file);
                return 1;
            }
        }

        rs.ListDevices();
        rs.InitSensor(rs_cfg, 0, "");
        utility::LogInfo("{}", rs.GetMetadata().ToString());
        depth_scale = static_cast<float>(rs.GetMetadata().depth_scale_);
        intrinsic = core::eigen_converter::EigenMatrixToTensor(
                rs.GetMetadata().intrinsics_.intrinsic_matrix_);
        rs.StartCapture();

        capture = [&rs, align_streams]() -> t::geometry::RGBDImage {
            return rs.CaptureFrame(true, align_streams);
        };
    }

    utility::LogInfo("Mode: {} | depth_max: {:.1f} m", mode, depth_max);

    if (mode == "pointcloud") {
        RunPointCloudViewer(capture, intrinsic, depth_scale, depth_max, use_bag);
    } else {
        RunRgbdViewer(capture, use_bag);
    }

    if (use_bag) {
        bag_reader.Close();
    } else {
        rs.StopCapture();
    }

    return 0;
}
