// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------
//
// Offline RGB-D scene reconstruction from Intel RealSense .bag files.
//
// Reads a RealSense bag, runs frame-to-model SLAM (track + TSDF integrate),
// and saves point cloud, mesh, and camera trajectory.
//
// Build: BUILD_LIBREALSENSE=ON, BUILD_CUDA_MODULE=ON (recommended)
//
// Examples:
//   OfflineReconstructionRealSense --input capture.bag --device CUDA:0
//   OfflineReconstructionRealSense --default_dataset jack_jack --profile medium
//   OfflineReconstructionRealSense --input capture.bag --mesh scene.ply --vis
//

#include <cmath>
#include <memory>
#include <string>

#include "open3d/Open3D.h"

namespace {

using namespace open3d;
namespace tio = open3d::t::io;

struct ReconParams {
    float voxel_size = 3.f / 512.f;
    float trunc_multiplier = 8.f;
    int block_count = 40000;
    int estimated_points = 4000000;
    float depth_max = 3.f;
    float depth_diff = 0.07f;
    float depth_scale = 1000.f;
    int odom_iter_coarse = 4;
    int odom_iter_mid = 2;
    int odom_iter_fine = 1;
};

ReconParams GetProfile(const std::string& profile) {
    ReconParams p;
    if (profile == "low") {
        p.voxel_size = 0.008f;
        p.block_count = 16384;
        p.estimated_points = 1500000;
        p.depth_max = 2.f;
        return p;
    }
    if (profile == "high") {
        p.block_count = 50000;
        p.estimated_points = 8000000;
        p.depth_max = 5.f;
        p.odom_iter_coarse = 6;
        p.odom_iter_mid = 3;
        return p;
    }
    return p;
}

void PrintHelp() {
    PrintOpen3DVersion();
    utility::LogInfo("Usage:");
    utility::LogInfo("    > OfflineReconstructionRealSense [options]");
    utility::LogInfo("");
    utility::LogInfo("Input (one required):");
    utility::LogInfo("    --input PATH.bag          RealSense bag file.");
    utility::LogInfo("    --default_dataset NAME    Built-in bag when --input is omitted.");
    utility::LogInfo("                              Options: jack_jack (default), sample_l515");
    utility::LogInfo("");
    utility::LogInfo("Reconstruction:");
    utility::LogInfo("    [--device CUDA:0|CPU:0]   Compute device (default: CUDA:0).");
    utility::LogInfo("    [--profile low|medium|high]  Quality/memory preset (default: medium).");
    utility::LogInfo("    [--max_frames N]          Process at most N frames (0 = all).");
    utility::LogInfo("    [--frame_step N]          Use every N-th frame (default: 1).");
    utility::LogInfo("    [--voxel_size M]          Override TSDF voxel size (meters).");
    utility::LogInfo("    [--depth_max M]           Depth truncation (meters).");
    utility::LogInfo("");
    utility::LogInfo("Output:");
    utility::LogInfo("    [--pointcloud PATH]       Save point cloud (default: scene.ply).");
    utility::LogInfo("    [--mesh PATH]             Save triangle mesh (default: scene_mesh.ply).");
    utility::LogInfo("    [--trajectory PATH]       Save camera path (default: trajectory.log).");
    utility::LogInfo("    [--no_mesh]               Skip mesh extraction.");
    utility::LogInfo("    [--no_pointcloud]         Skip point cloud extraction.");
    utility::LogInfo("    [--vis]                   Visualize result after reconstruction.");
    utility::LogInfo("");
}

bool TrackingAccepted(const t::pipelines::odometry::OdometryResult& result) {
    core::Tensor translation =
            result.transformation_.Slice(0, 0, 3).Slice(1, 3, 4);
    const double translation_norm = std::sqrt(
            (translation * translation).Sum({0, 1}).Item<double>());
    return result.fitness_ >= 0.1 && translation_norm < 0.15;
}

std::string ResolveBagPath(int argc, char* argv[]) {
    const std::string input =
            utility::GetProgramOptionAsString(argc, argv, "--input", "");
    if (!input.empty()) {
        if (!utility::filesystem::FileExists(input)) {
            utility::LogError("Bag file not found: {}", input);
        }
        return input;
    }

    const std::string dataset = utility::GetProgramOptionAsString(
            argc, argv, "--default_dataset", "jack_jack");
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

}  // namespace

int main(int argc, char* argv[]) {
    utility::SetVerbosityLevel(utility::VerbosityLevel::Info);

    if (argc <= 1 ||
        utility::ProgramOptionExistsAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }

    const std::string bag_path = ResolveBagPath(argc, argv);

    const std::string device_code =
            utility::GetProgramOptionAsString(argc, argv, "--device", "CUDA:0");
    const core::Device device(device_code);

    const std::string profile =
            utility::GetProgramOptionAsString(argc, argv, "--profile", "medium");
    if (profile != "low" && profile != "medium" && profile != "high") {
        utility::LogError(
                "Unknown --profile '{}'. Use low, medium, or high.", profile);
        return 1;
    }

    ReconParams params = GetProfile(profile);
    if (utility::ProgramOptionExists(argc, argv, "--voxel_size")) {
        params.voxel_size = static_cast<float>(utility::GetProgramOptionAsDouble(
                argc, argv, "--voxel_size", params.voxel_size));
    }
    if (utility::ProgramOptionExists(argc, argv, "--depth_max")) {
        params.depth_max = static_cast<float>(utility::GetProgramOptionAsDouble(
                argc, argv, "--depth_max", params.depth_max));
    }

    const int max_frames =
            utility::GetProgramOptionAsInt(argc, argv, "--max_frames", 0);
    const int frame_step =
            std::max(1, utility::GetProgramOptionAsInt(argc, argv, "--frame_step", 1));

    const bool save_pcd =
            !utility::ProgramOptionExists(argc, argv, "--no_pointcloud");
    const bool save_mesh = !utility::ProgramOptionExists(argc, argv, "--no_mesh");
    const std::string pcd_path = utility::GetProgramOptionAsString(
            argc, argv, "--pointcloud", "scene.ply");
    const std::string mesh_path = utility::GetProgramOptionAsString(
            argc, argv, "--mesh", "scene_mesh.ply");
    const std::string traj_path = utility::GetProgramOptionAsString(
            argc, argv, "--trajectory", "trajectory.log");
    const bool visualize = utility::ProgramOptionExists(argc, argv, "--vis");

    utility::LogInfo("Bag file: {}", bag_path);
    utility::LogInfo("Device: {}", device.ToString());
    utility::LogInfo("Profile: {}", profile);

    tio::RSBagReader bag_reader;
    bag_reader.Open(bag_path);
    if (!bag_reader.IsOpened()) {
        utility::LogError("Unable to open bag file: {}", bag_path);
        return 1;
    }

    const auto metadata = bag_reader.GetMetadata();
    utility::LogInfo("{}", metadata.ToString());
    params.depth_scale = static_cast<float>(metadata.depth_scale_);

    const core::Tensor intrinsic = core::eigen_converter::EigenMatrixToTensor(
            metadata.intrinsics_.intrinsic_matrix_);
    const camera::PinholeCameraIntrinsic cam_intrinsic = metadata.intrinsics_;

    t::geometry::RGBDImage first = bag_reader.NextFrame();
    if (first.IsEmpty()) {
        utility::LogError("Bag file contains no frames.");
        return 1;
    }

    const int height = first.depth_.GetRows();
    const int width = first.depth_.GetCols();

    core::Tensor T_frame_to_model =
            core::Tensor::Eye(4, core::Dtype::Float64, core::Device("CPU:0"));
    t::pipelines::slam::Model model(params.voxel_size, 16, params.block_count,
                                    T_frame_to_model, device);

    t::pipelines::slam::Frame input_frame(height, width, intrinsic, device);
    t::pipelines::slam::Frame raycast_frame(height, width, intrinsic, device);

    const std::vector<t::pipelines::odometry::OdometryConvergenceCriteria>
            odom_criteria = {params.odom_iter_coarse, params.odom_iter_mid,
                             params.odom_iter_fine};

    auto trajectory = std::make_shared<camera::PinholeCameraTrajectory>();

    int frame_id = 0;
    int processed = 0;
    t::geometry::RGBDImage rgbd = first;

    while (!rgbd.IsEmpty()) {
        if (max_frames > 0 && processed >= max_frames) {
            break;
        }

        const bool use_frame = (frame_id % frame_step == 0);
        if (use_frame) {
            rgbd = rgbd.To(device);
            input_frame.SetDataFromImage("depth", rgbd.depth_);
            input_frame.SetDataFromImage("color", rgbd.color_);

            bool tracking_success = true;
            if (processed > 0) {
                auto result = model.TrackFrameToModel(
                        input_frame, raycast_frame, params.depth_scale,
                        params.depth_max, params.depth_diff,
                        t::pipelines::odometry::Method::PointToPlane,
                        odom_criteria);
                if (TrackingAccepted(result)) {
                    T_frame_to_model =
                            T_frame_to_model.Matmul(result.transformation_);
                } else {
                    tracking_success = false;
                    utility::LogWarning(
                            "Tracking weak at frame {} (fitness {:.3f}).",
                            frame_id, result.fitness_);
                }
            }

            model.UpdateFramePose(processed, T_frame_to_model);
            if (tracking_success) {
                model.Integrate(input_frame, params.depth_scale,
                                params.depth_max, params.trunc_multiplier);
            }
            model.SynthesizeModelFrame(raycast_frame, params.depth_scale, 0.1f,
                                       params.depth_max,
                                       params.trunc_multiplier, false);

            camera::PinholeCameraParameters cam_params;
            cam_params.intrinsic_ = cam_intrinsic;
            cam_params.extrinsic_ =
                    core::eigen_converter::TensorToEigenMatrixXd(
                            T_frame_to_model);
            trajectory->parameters_.push_back(cam_params);

            utility::LogInfo("Processed frame {} ({}/{}) | hash blocks {}",
                             frame_id, processed + 1,
                             max_frames > 0 ? max_frames : -1,
                             model.GetHashMap().Size());
            ++processed;
        }

        ++frame_id;
        if (bag_reader.IsEOF()) {
            break;
        }
        rgbd = bag_reader.NextFrame();
    }

    bag_reader.Close();
    utility::LogInfo("Finished SLAM on {} frames ({} bag frames read).",
                     processed, frame_id);

    std::shared_ptr<geometry::TriangleMesh> mesh_legacy;
    std::shared_ptr<geometry::PointCloud> pcd_legacy;

    if (save_pcd) {
        auto pcd_t =
                model.ExtractPointCloud(3.f, params.estimated_points);
        pcd_legacy =
                std::make_shared<geometry::PointCloud>(pcd_t.ToLegacy());
        io::WritePointCloud(pcd_path, *pcd_legacy);
        utility::LogInfo("Saved point cloud: {}", pcd_path);
    }

    if (save_mesh) {
        auto mesh_t =
                model.ExtractTriangleMesh(3.f, params.estimated_points);
        mesh_legacy =
                std::make_shared<geometry::TriangleMesh>(mesh_t.ToLegacy());
        io::WriteTriangleMesh(mesh_path, *mesh_legacy);
        utility::LogInfo("Saved mesh: {}", mesh_path);
    }

    io::WritePinholeCameraTrajectory(traj_path, *trajectory);
    utility::LogInfo("Saved trajectory: {}", traj_path);

    if (visualize) {
        if (mesh_legacy) {
            visualization::Draw({mesh_legacy}, "Reconstructed Mesh");
        } else if (pcd_legacy) {
            visualization::Draw({pcd_legacy}, "Reconstructed Point Cloud");
        } else {
            utility::LogWarning(
                    "Nothing to visualize. Remove --no_mesh / --no_pointcloud.");
        }
    }

    return 0;
}
