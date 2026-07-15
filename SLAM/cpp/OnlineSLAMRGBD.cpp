// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include <chrono>
#include <memory>
#include <string>

#include "OnlineSLAMUtil.h"
#include "open3d/Open3D.h"

std::pair<std::vector<std::string>, std::vector<std::string>> LoadFilenames(
        const std::string dataset_path) {
    using namespace open3d;

    std::vector<std::string> rgb_candidates{"color", "image", "rgb"};
    std::vector<std::string> rgb_files;

    // Load rgb
    for (auto rgb_candidate : rgb_candidates) {
        const std::string rgb_dir = dataset_path + "/" + rgb_candidate;
        utility::filesystem::ListFilesInDirectoryWithExtension(rgb_dir, "jpg",
                                                               rgb_files);
        if (rgb_files.size() != 0) break;
        utility::filesystem::ListFilesInDirectoryWithExtension(rgb_dir, "png",
                                                               rgb_files);
        if (rgb_files.size() != 0) break;
    }
    if (rgb_files.size() == 0) {
        utility::LogError(
                "RGB images not found! Please ensure a folder named color, "
                "image, or rgb is in {}",
                dataset_path);
    }

    const std::string depth_dir = dataset_path + "/depth";
    std::vector<std::string> depth_files;
    utility::filesystem::ListFilesInDirectoryWithExtension(depth_dir, "png",
                                                           depth_files);
    if (depth_files.size() == 0) {
        utility::LogError(
                "Depth images not found! Please ensure a folder named "
                "depth is in {}",
                dataset_path);
    }

    if (depth_files.size() != rgb_files.size()) {
        utility::LogError(
                "Number of depth images ({}) and color image ({}) "
                "mismatch!",
                dataset_path);
    }

    std::sort(rgb_files.begin(), rgb_files.end());
    std::sort(depth_files.begin(), depth_files.end());
    return std::make_pair(rgb_files, depth_files);
}

void PrintHelp() {
    using namespace open3d;
    PrintOpen3DVersion();

    // clang-format off
    utility::LogInfo("Usage:");
    utility::LogInfo("    > OnlineSLAMRGBD [options]");
    utility::LogInfo("Basic options:");
    utility::LogInfo("    [-V]");
    utility::LogInfo("    [--dataset_path /path/to/dataset]"); 
    utility::LogInfo("                    - To use your own dataset, pass the path");
    utility::LogInfo("                      to the dataset root folder containing");
    utility::LogInfo("                      `image` and `depth` folder. If not");
    utility::LogInfo("                      provided, default dataset will be used.");
    utility::LogInfo("    [--intrinsic_path camera_intrinsic.json]");
    utility::LogInfo("    [--align]");
    utility::LogInfo("    [--device CUDA:0]");
    utility::LogInfo("    [--default_dataset lounge]");
    utility::LogInfo("                    - To change default dataset (used when");
    utility::LogInfo("                      dataset_path is not set).");
    utility::LogInfo("                      Available options: `lounge` and `bedroom`.");
    utility::LogInfo("");
    utility::LogInfo("Region freeze (--regions):");
    utility::LogInfo("    [--regions]             Detect stable scan regions, freeze");
    utility::LogInfo("                             TSDF blocks, extract triangle meshes,");
    utility::LogInfo("                             and save to --region_dir.");
    utility::LogInfo("    [--region_min_points N] Min cluster points (default: 2000 with --regions).");
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
    // clang-format on
    utility::LogInfo("");
}

int main(int argc, char* argv[]) {
    using namespace open3d;
    using namespace open3d::visualization;

    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    if (argc < 1 ||
        utility::ProgramOptionExistsAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }

    bool use_default_dataset = true;

    std::string dataset_path =
            utility::GetProgramOptionAsString(argc, argv, "--dataset_path", "");
    if (!dataset_path.empty()) {
        if (!utility::filesystem::DirectoryExists(dataset_path)) {
            utility::LogError(
                    "Expected an existing directory, but {} does not exist.",
                    dataset_path);
        }

        use_default_dataset = false;
    }

    if (use_default_dataset) {
        const std::string default_dataset = utility::GetProgramOptionAsString(
                argc, argv, "--default_dataset", "lounge");
        if (default_dataset == "lounge") {
            data::LoungeRGBDImages dataset;
            dataset_path = dataset.GetExtractDir();
        } else if (default_dataset == "bedroom") {
            data::BedroomRGBDImages dataset;
            dataset_path = dataset.GetExtractDir();
        } else {
            utility::LogError(
                    "The default_dataset {}, is not available. Please select "
                    "from `lounge` (default) and `bedroom` dataset.",
                    default_dataset);
        }
    }

    if (utility::ProgramOptionExists(argc, argv, "-V")) {
        utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    } else {
        utility::SetVerbosityLevel(utility::VerbosityLevel::Info);
    }

    std::string intrinsic_path = utility::GetProgramOptionAsString(
            argc, argv, "--intrinsics_path", "");

    bool align_streams = false;
    if (utility::ProgramOptionExists(argc, argv, "--align")) {
        align_streams = true;
    }

    std::string device_code =
            utility::GetProgramOptionAsString(argc, argv, "--device", "CUDA:0");
    core::Device device(device_code);
    utility::LogInfo("Using device {}.", device_code);

    examples::online_slam::RegionSettings region_settings;
    if (utility::ProgramOptionExists(argc, argv, "--regions")) {
        region_settings.enabled = true;
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_min_points")) {
        region_settings.min_points = utility::GetProgramOptionAsInt(
                argc, argv, "--region_min_points", region_settings.min_points);
    } else if (region_settings.enabled) {
        region_settings.min_points = 2000;
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_stability")) {
        region_settings.stability_frames = utility::GetProgramOptionAsInt(
                argc, argv, "--region_stability",
                region_settings.stability_frames);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_interval")) {
        region_settings.interval = utility::GetProgramOptionAsInt(
                argc, argv, "--region_interval", region_settings.interval);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_dir")) {
        region_settings.output_dir = utility::GetProgramOptionAsString(
                argc, argv, "--region_dir", region_settings.output_dir);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_min_readiness")) {
        region_settings.min_readiness = static_cast<float>(
                utility::GetProgramOptionAsDouble(
                        argc, argv, "--region_min_readiness",
                        region_settings.min_readiness));
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_max_holey_defer")) {
        region_settings.max_holey_defer = utility::GetProgramOptionAsInt(
                argc, argv, "--region_max_holey_defer",
                region_settings.max_holey_defer);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_no_holey_defer")) {
        region_settings.defer_holey = false;
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_max_void_ratio")) {
        region_settings.max_void_ratio = utility::GetProgramOptionAsDouble(
                argc, argv, "--region_max_void_ratio",
                region_settings.max_void_ratio);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_max_void_blob")) {
        region_settings.max_void_blob_cells = utility::GetProgramOptionAsInt(
                argc, argv, "--region_max_void_blob",
                region_settings.max_void_blob_cells);
    }
    if (utility::ProgramOptionExists(argc, argv,
                                     "--region_max_boundary_void")) {
        region_settings.max_boundary_void_ratio =
                utility::GetProgramOptionAsDouble(
                        argc, argv, "--region_max_boundary_void",
                        region_settings.max_boundary_void_ratio);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_extract_weight")) {
        region_settings.extract_weight = static_cast<float>(
                utility::GetProgramOptionAsDouble(
                        argc, argv, "--region_extract_weight",
                        region_settings.extract_weight));
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_depth_min")) {
        region_settings.depth_min_m = utility::GetProgramOptionAsDouble(
                argc, argv, "--region_depth_min", region_settings.depth_min_m);
    }
    bool region_depth_max_set = false;
    if (utility::ProgramOptionExists(argc, argv, "--region_depth_max")) {
        region_settings.depth_max_m = utility::GetProgramOptionAsDouble(
                argc, argv, "--region_depth_max", region_settings.depth_max_m);
        region_depth_max_set = true;
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_max_extent")) {
        region_settings.max_cluster_extent_m =
                utility::GetProgramOptionAsDouble(
                        argc, argv, "--region_max_extent",
                        region_settings.max_cluster_extent_m);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_min_motion_m")) {
        region_settings.min_motion_translation_m =
                utility::GetProgramOptionAsDouble(
                        argc, argv, "--region_min_motion_m",
                        region_settings.min_motion_translation_m);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_min_motion_deg")) {
        region_settings.min_motion_rotation_deg =
                utility::GetProgramOptionAsDouble(
                        argc, argv, "--region_min_motion_deg",
                        region_settings.min_motion_rotation_deg);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_min_hash_delta")) {
        region_settings.min_hash_delta_blocks = utility::GetProgramOptionAsInt(
                argc, argv, "--region_min_hash_delta",
                region_settings.min_hash_delta_blocks);
    }
    if (utility::ProgramOptionExists(argc, argv, "--region_motion_warmup")) {
        region_settings.region_motion_warmup_frames =
                utility::GetProgramOptionAsInt(
                        argc, argv, "--region_motion_warmup",
                        region_settings.region_motion_warmup_frames);
    }
    if (utility::ProgramOptionExists(argc, argv,
                                     "--region_allow_stationary_mesh")) {
        region_settings.require_camera_motion = false;
    }
    if (!region_depth_max_set) {
        // Medium GUI default depth_max is 3.0; match RealTime min(2.5, depth_max).
        region_settings.depth_max_m = std::min(2.5, 3.0);
    }

    // Load files
    std::vector<std::string> rgb_files, depth_files;
    std::tie(rgb_files, depth_files) = LoadFilenames(dataset_path);

    // Load intrinsics (if provided)
    // Default
    camera::PinholeCameraIntrinsic intrinsic = camera::PinholeCameraIntrinsic(
            camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);
    if (intrinsic_path.empty()) {
        utility::LogInfo("Using Primesense default intrinsics.");
    } else if (!io::ReadIJsonConvertible(intrinsic_path, intrinsic)) {
        utility::LogWarning(
                "Failed to load {}, using Primesense default intrinsics.",
                intrinsic_path);
    } else {
        utility::LogInfo("Loaded intrinsics from {}.", intrinsic_path);
    }
    core::Tensor intrinsic_t = core::eigen_converter::EigenMatrixToTensor(
            intrinsic.intrinsic_matrix_);

    const size_t max_idx = depth_files.size();
    auto get_rgbd_image_input = [&](const size_t idx) {
        if (idx < max_idx) {
            t::geometry::Image depth =
                    *t::io::CreateImageFromFile(depth_files[idx]);
            t::geometry::Image color =
                    *t::io::CreateImageFromFile(rgb_files[idx]);
            t::geometry::RGBDImage rgbd_im(color, depth, align_streams);
            return rgbd_im;
        } else {
            // Return empty image to indicate EOF.
            return t::geometry::RGBDImage();
        }
    };

    std::unordered_map<std::string, double> default_params = {
            {"depth_scale", 1000},
            {"gui_update_interval", 3},
            {"update_interval", 100}};
    if (region_settings.enabled) {
        default_params["auto_freeze"] = 1;
    }

    auto& app = gui::Application::GetInstance();
    app.Initialize();
    auto mono =
            app.AddFont(gui::FontDescription(gui::FontDescription::MONOSPACE));
    app.AddWindow(std::make_shared<examples::online_slam::ReconstructionWindow>(
            get_rgbd_image_input, intrinsic_t, default_params, device, mono,
            true, region_settings));
    app.Run();

    return 0;
}
