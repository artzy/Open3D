// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#if FMT_VERSION >= 100000
#include <fmt/std.h>
#endif

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <mutex>
#include <sstream>
#include <thread>
#include <unordered_map>

#include "open3d/Open3D.h"
#include "open3d/core/TensorFunction.h"
#include "ObjectMeshPipeline.h"

namespace open3d {
namespace examples {
namespace online_slam {
using namespace open3d::visualization;

// Filament upload budget for live preview (extract uses estimated_points).
static constexpr int kMaxRenderPoints = 500000;

// Tracking tiers: only Strong frames (good fitness, small per-frame motion)
// update the pose and integrate. Weak / failed / outlier frames are skipped
// entirely: integrating with an uncertain pose stamps duplicate copies of
// the scene ("ghost walls").
static constexpr double kPoseFitnessMin = 0.25;      // Min fitness slider default
static constexpr double kPoseTranslationMax = 0.12;  // m per frame
static constexpr double kPoseRotationMaxDeg = 8.0;   // deg per frame
static constexpr double kWeakFitnessMin = 0.08;
static constexpr double kWeakTranslationMax = 0.30;
static constexpr double kOutlierTranslation = 0.50;
static constexpr double kOutlierRotationDeg = 25.0;

// Relocalization gate: after this many consecutive non-Strong frames the
// pose is likely drifted, so integration is paused until the camera re-locks
// onto the model with several consecutive Strong frames. This prevents a
// drifted pose from integrating a shifted duplicate of already-scanned walls.
static constexpr int kRelocalizeAfterFailures = 3;
static constexpr int kRelocalizeStrongFrames = 5;

// Free-space carving: voxels that a trusted frame observes well in front of
// the measured surface are ghost candidates; halving their weight every pass
// erases mis-integrated geometry once the area is rescanned correctly.
// A pass costs ~60-130 ms (CUDA, medium profile), so it runs sparsely; real
// surfaces are re-integrated (+1 weight/frame) between passes and keep a
// stable equilibrium well above the extraction weight threshold.
static constexpr int kCarveIntervalCuda = 30;  // frames between carve passes
static constexpr int kCarveIntervalCpu = 90;
static constexpr float kCarveMarginTruncFactor = 3.0f;  // margin/sdf_trunc

// Stationary gate: when the camera has not moved for this many consecutive
// frames, Integrate/carving are skipped (tracking and GUI stay live). By
// then voxel weights are well above the extraction threshold, so further
// integration only burns GPU time and lets edge noise allocate new blocks.
static constexpr double kStationaryTranslationMax = 0.005;  // m per frame
static constexpr double kStationaryRotationMaxDeg = 0.3;    // deg per frame
static constexpr int kStationaryFrames = 30;

// OdometryLossParams uses depth_huber_delta=0.05 by default; truncation must
// be strictly greater to avoid Huber/L2 degeneracy warnings.
static constexpr float kOdometryHuberDelta = 0.05f;
float SafeOdometryDepthDiff(float depth_diff) {
    return std::max(kOdometryHuberDelta + 0.001f, depth_diff);
}

enum class TrackingTier { kInit, kStrong, kWeak, kOutlier, kFail };

// Scale GUI refresh rate when the voxel hash grows (ExtractPointCloud cost rises).
int GetEffectiveUpdateInterval(int base_interval, int64_t hash_size) {
    int interval = std::max(1, base_interval);
    if (hash_size > 25000) {
        interval = std::max(interval, 200);
    } else if (hash_size > 15000) {
        interval = std::max(interval, 100);
    } else if (hash_size > 8000) {
        interval = std::max(interval, 75);
    }
    return interval;
}

// Skip Integrate when the voxel hash is nearly full to avoid GPU rehash OOM.
static constexpr double kHashIntegrateFillRatio = 0.95;
// Allocate a modest hash first, then expand while empty (no mid-scan rehash).
static constexpr int64_t kInitialHashCapacity = 40000;
static constexpr int64_t kMaxHashCapacity = 50000;

// Cap live GPU extract budget as the hash grows (depth_max 8m fills blocks faster).
int GetLiveExtractBudget(int estimated_points, int64_t hash_size) {
    int budget = estimated_points;
    if (hash_size > 45000) {
        budget = std::min(budget, 2000000);
    } else if (hash_size > 30000) {
        budget = std::min(budget, 3500000);
    } else if (hash_size > 20000) {
        budget = std::min(budget, 5000000);
    }
    return budget;
}

// Rough TSDF VRAM estimate: ~48 KB per 16^3 voxel block (tsdf+weight+color).
int64_t EstimateTsdfVramMb(int block_count) {
    return (static_cast<int64_t>(block_count) * 48) / 1024;
}

// Memory profiles for RealSense SLAM (RTX 3060 12GB baseline: medium).
std::unordered_map<std::string, double> GetSlamProfile(
        const std::string& profile) {
    if (profile == "low") {
        return {{"voxel_size", 0.008},
                {"trunc_multiplier", 8.0},
                {"block_count", 16384},
                {"estimated_points", 1500000},
                {"update_interval", 30},
                {"depth_max", 2.0},
                {"depth_diff", 0.07}};
    }
    if (profile == "high") {
        return {{"voxel_size", 3.0 / 512.0},
                {"trunc_multiplier", 8.0},
                {"block_count", 50000},
                {"estimated_points", 8000000},
                {"update_interval", 50},
                {"depth_max", 5.0},
                {"depth_diff", 0.07}};
    }
    // medium (default): aligned with Python default_config.yml
    return {{"voxel_size", 3.0 / 512.0},
            {"trunc_multiplier", 8.0},
            {"block_count", 40000},
            {"estimated_points", 6000000},
            {"update_interval", 50},
            {"depth_max", 3.0},
            {"depth_diff", 0.07}};
}

// Performance presets (independent from --profile memory settings).
std::unordered_map<std::string, double> GetPerfPreset(
        const std::string& perf) {
    if (perf == "balanced") {
        return {{"odometry_iter_0", 4},
                {"odometry_iter_1", 2},
                {"odometry_iter_2", 1},
                {"gui_update_interval", 2},
                {"raycast_color", 1}};
    }
    if (perf == "quality") {
        return {{"odometry_iter_0", 6},
                {"odometry_iter_1", 3},
                {"odometry_iter_2", 1},
                {"gui_update_interval", 1},
                {"raycast_color", 1}};
    }
    // fast (default)
    return {{"odometry_iter_0", 3},
            {"odometry_iter_1", 2},
            {"odometry_iter_2", 1},
            {"gui_update_interval", 3},
            {"raycast_color", 0}};
}

// Tanglo colorscheme (see https://en.wikipedia.org/wiki/Tango_Desktop_Project)
static const Eigen::Vector3d kTangoOrange(0.961, 0.475, 0.000);
static const Eigen::Vector3d kTangoSkyBlueDark(0.125, 0.290, 0.529);

class PropertyPanel : public gui::VGrid {
    using Super = gui::VGrid;

public:
    PropertyPanel(int spacing, int left_margin)
        : gui::VGrid(2, spacing, gui::Margins(left_margin, 0, 0, 0)) {
        default_label_color_ =
                std::make_shared<gui::Label>("temp")->GetTextColor();
    }

    void AddBool(const std::string& name,
                 std::atomic<bool>* bool_addr,
                 bool default_val,
                 const std::string& tooltip = "") {
        auto cb = std::make_shared<gui::Checkbox>("");
        cb->SetChecked(default_val);
        *bool_addr = default_val;
        cb->SetOnChecked([bool_addr, this](bool is_checked) {
            *bool_addr = is_checked;
            this->NotifyChanged();
        });
        auto label = std::make_shared<gui::Label>(name.c_str());
        label->SetTooltip(tooltip.c_str());
        AddChild(label);
        AddChild(cb);
    }

    void AddFloatSlider(const std::string& name,
                        std::atomic<double>* num_addr,
                        double default_val,
                        double min_val,
                        double max_val,
                        const std::string& tooltip = "") {
        auto s = std::make_shared<gui::Slider>(gui::Slider::DOUBLE);
        s->SetLimits(min_val, max_val);
        s->SetValue(default_val);
        *num_addr = default_val;
        s->SetOnValueChanged([num_addr, this](double new_val) {
            *num_addr = new_val;
            this->NotifyChanged();
        });
        auto label = std::make_shared<gui::Label>(name.c_str());
        label->SetTooltip(tooltip.c_str());
        AddChild(label);
        AddChild(s);
    }

    void AddIntSlider(const std::string& name,
                      std::atomic<int>* num_addr,
                      int default_val,
                      int min_val,
                      int max_val,
                      const std::string& tooltip = "") {
        auto s = std::make_shared<gui::Slider>(gui::Slider::INT);
        s->SetLimits(min_val, max_val);
        s->SetValue(default_val);
        *num_addr = default_val;
        s->SetOnValueChanged([num_addr, this](int new_val) {
            *num_addr = new_val;
            this->NotifyChanged();
        });
        auto label = std::make_shared<gui::Label>(name.c_str());
        label->SetTooltip(tooltip.c_str());
        AddChild(label);
        AddChild(s);
    }

    void AddValues(const std::string& name,
                   std::atomic<int>* idx_addr,
                   int default_idx,
                   std::vector<std::string> values,
                   const std::string& tooltip = "") {
        auto combo = std::make_shared<gui::Combobox>();
        for (auto& v : values) {
            combo->AddItem(v.c_str());
        }
        combo->SetSelectedIndex(default_idx);
        *idx_addr = default_idx;
        combo->SetOnValueChanged(
                [idx_addr, this](const char* new_value, int new_idx) {
                    *idx_addr = new_idx;
                    this->NotifyChanged();
                });
        auto label = std::make_shared<gui::Label>(name.c_str());
        label->SetTooltip(tooltip.c_str());
        AddChild(label);
        AddChild(combo);
    }

    void SetEnabled(bool enable) override {
        Super::SetEnabled(enable);
        for (auto child : GetChildren()) {
            child->SetEnabled(enable);
            auto label = std::dynamic_pointer_cast<gui::Label>(child);
            if (label) {
                if (enable) {
                    label->SetTextColor(default_label_color_);
                } else {
                    label->SetTextColor(gui::Color(0.5f, 0.5f, 0.5f, 1.0f));
                }
            }
        }
    }

    void SetOnChanged(std::function<void()> f) { on_changed_ = f; }

private:
    gui::Color default_label_color_;
    std::function<void()> on_changed_;

    void NotifyChanged() {
        if (on_changed_) {
            on_changed_();
        }
    }
};

class ReconstructionWindow : public gui::Window {
    using Super = gui::Window;

public:
    ReconstructionWindow(
            const std::function<t::geometry::RGBDImage(const size_t idx)>
                    get_rgbd_image_input,
            const core::Tensor& intrinsic,
            const std::unordered_map<std::string, double> default_parameters,
            const core::Device device,
            gui::FontId monospace,
            bool exit_on_empty_frame = true)
        : gui::Window("Open3D - Reconstruction", 2560, 1600),
          get_rgbd_image_input_(get_rgbd_image_input),
          intrinsic_(intrinsic),
          default_parameters_(default_parameters),
          device_(device),
          is_running_(false),
          is_started_(false),
          exit_on_empty_frame_(exit_on_empty_frame),
          monospace_(monospace) {
        ////////////////////////////////////////
        /// General layout
        auto& theme = GetTheme();
        int em = theme.font_size;
        int spacing = int(std::round(0.25f * float(em)));
        int left_margin = em;
        int vspacing = int(std::round(0.5f * float(em)));
        gui::Margins margins(int(std::round(0.5f * float(em))));
        panel_ = std::make_shared<gui::Vert>(spacing, margins);
        widget3d_ = std::make_shared<gui::SceneWidget>();
        fps_panel_ = std::make_shared<gui::Vert>(spacing, margins);

        AddChild(panel_);
        AddChild(widget3d_);
        AddChild(fps_panel_);

        ////////////////////////////////////////
        /// Property panels
        /// Default value look up map.
        std::unordered_map<std::string, double> default_param = {
                {"depth_scale", 1000},
                {"voxel_size", 3.0 / 512.0},
                {"trunc_multiplier", 8.0},
                {"block_count", 40000},
                {"estimated_points", 6000000},
                {"update_interval", 50},
                {"depth_max", 3.0},
                {"depth_diff", 0.07},
                {"min_fitness", kPoseFitnessMin},
                {"min_weight", 3.0},
                {"odometry_iter_0", 6},
                {"odometry_iter_1", 3},
                {"odometry_iter_2", 1},
                {"gui_update_interval", 1},
                {"raycast_color", 1},
                {"auto_freeze", 1},
                {"stability_frames", 5},
                {"dbscan_eps_multiplier", 2.0},
                {"min_cluster_points", 5000},
                {"planar_tiles", 1},
                {"tile_size", 1.0}};
        /// Override values by user provided default parameters
        for (auto it : default_parameters_) {
            if (default_param.find(it.first) != default_param.end()) {
                default_param[it.first] = default_parameters_.at(it.first);
            }
        }

        fixed_props_ = std::make_shared<PropertyPanel>(spacing, left_margin);
        fixed_props_->AddIntSlider("Depth scale", &prop_values_.depth_scale,
                                   default_param.at("depth_scale"), 1000, 5000,
                                   "Scale factor applied to the depth values "
                                   "from the depth image.");
        fixed_props_->AddFloatSlider("Voxel size", &prop_values_.voxel_size,
                                     default_param.at("voxel_size"), 0.004,
                                     0.01,
                                     "Voxel size for the TSDF voxel grid.");
        fixed_props_->AddFloatSlider(
                "Trunc multiplier", &prop_values_.trunc_multiplier,
                default_param.at("trunc_multiplier"), 1.0, 20.0,
                "Truncate distance multiplier (in voxel size) to control "
                "the volumetric surface thickness.");
        fixed_props_->AddIntSlider(
                "Block count", &prop_values_.block_count,
                default_param.at("block_count"), 10000, 50000,
                "Target voxel hash capacity. Expanded while empty to avoid "
                "mid-scan GPU rehash. Restart required after change.");
        fixed_props_->AddIntSlider(
                "Estimated points", &prop_values_.estimated_points,
                default_param.at("estimated_points"), 1000000, 10000000,
                "Estimated number of points in the point cloud; used to speed "
                "extraction of points into the 3D scene. Increase if you see "
                "'Point cloud size larger than estimated' warnings.");
        fixed_props_->AddIntSlider(
                "Odom iter coarse", &prop_values_.odometry_iter_0,
                static_cast<int>(default_param.at("odometry_iter_0")), 1, 10,
                "RGB-D odometry max iterations (coarse pyramid level).");
        fixed_props_->AddIntSlider(
                "Odom iter mid", &prop_values_.odometry_iter_1,
                static_cast<int>(default_param.at("odometry_iter_1")), 1, 10,
                "RGB-D odometry max iterations (mid pyramid level).");
        fixed_props_->AddIntSlider(
                "Odom iter fine", &prop_values_.odometry_iter_2,
                static_cast<int>(default_param.at("odometry_iter_2")), 1, 10,
                "RGB-D odometry max iterations (fine pyramid level).");
        fixed_props_->AddIntSlider(
                "GUI interval", &prop_values_.gui_update_interval,
                static_cast<int>(default_param.at("gui_update_interval")), 1, 10,
                "Refresh 2D preview and info every N frames (SLAM runs every "
                "frame).");

        adjustable_props_ =
                std::make_shared<PropertyPanel>(spacing, left_margin);
        adjustable_props_->AddIntSlider(
                "Update interval", &prop_values_.update_interval,
                default_param.at("update_interval"), 1, 500,
                "The number of iterations between updating the 3D display.");

        adjustable_props_->AddFloatSlider("Depth max", &prop_values_.depth_max,
                                          default_param.at("depth_max"), 1.0,
                                          5.0,
                                          "Maximum depth before point is "
                                          "discarded. Default 3m (indoor).");
        adjustable_props_->AddFloatSlider(
                "Depth diff", &prop_values_.depth_diff,
                default_param.at("depth_diff"), 0.06, 0.5,
                "Depth truncation for tracking outlier rejection. Must be "
                "> 0.05 (internal Huber delta). Default 0.07.");
        adjustable_props_->AddFloatSlider(
                "Min fitness", &prop_values_.min_fitness,
                default_param.at("min_fitness"), 0.10, 0.50,
                "Minimum odometry fitness to accept a frame for integration. "
                "Higher values reject uncertain poses (fewer ghost walls) at "
                "the cost of skipping more frames. Default 0.25.");
        adjustable_props_->AddFloatSlider(
                "Min weight", &prop_values_.min_weight,
                default_param.at("min_weight"), 1.0, 10.0,
                "TSDF weight threshold used by 'Clean ghosts' and by the "
                "final scene.ply export. Voxels observed fewer times than "
                "this are treated as unreliable. Default 3.");
        adjustable_props_->AddBool("Update surface",
                                   &prop_values_.update_surface, true,
                                   "Update surface every several frames, "
                                   "determined by the update interval.");
        adjustable_props_->AddBool(
                "Raycast color", &prop_values_.raycast_color,
                default_param.at("raycast_color") > 0.5,
                "Enable bilinear interpolated color image for visualization.");
        adjustable_props_->AddBool(
                "Auto freeze", &prop_values_.auto_freeze,
                default_param.at("auto_freeze") > 0.5,
                "Automatically freeze stable clusters as mesh assets.");
        adjustable_props_->AddIntSlider(
                "Stability frames", &prop_values_.stability_frames,
                static_cast<int>(default_param.at("stability_frames")), 2, 20,
                "Consecutive stable extracts before mesh freeze.");
        adjustable_props_->AddFloatSlider(
                "DBSCAN eps x voxel", &prop_values_.dbscan_eps_multiplier,
                default_param.at("dbscan_eps_multiplier"), 1.0, 5.0,
                "DBSCAN neighborhood radius as a multiple of voxel size.");
        adjustable_props_->AddIntSlider(
                "Min cluster points", &prop_values_.min_cluster_points,
                static_cast<int>(default_param.at("min_cluster_points")), 500,
                50000,
                "Ignore clusters smaller than this point count.");
        adjustable_props_->AddBool(
                "Planar tiles", &prop_values_.planar_tiles,
                default_param.at("planar_tiles") > 0.5,
                "Partition walls/floors into planar patches subdivided into "
                "tiles so large surfaces freeze incrementally.");
        adjustable_props_->AddFloatSlider(
                "Tile size", &prop_values_.tile_size,
                default_param.at("tile_size"), 0.5, 2.0,
                "Edge length (m) of plane-local tiles used to partition "
                "large planar regions.");

        panel_->AddChild(std::make_shared<gui::Label>("Starting settings"));
        panel_->AddChild(fixed_props_);
        panel_->AddFixed(vspacing);
        panel_->AddChild(
                std::make_shared<gui::Label>("Reconstruction settings"));
        panel_->AddChild(adjustable_props_);

        auto b = std::make_shared<gui::ToggleSwitch>("Resume/Pause");
        resume_toggle_ = b;
        b->SetOn(true);
        b->SetOnClicked([this](bool is_on) {
            if (is_on) {
                if (is_started_) {
                    is_running_ = true;
                    adjustable_props_->SetEnabled(true);
                }
            } else {
                PauseSlam();
            }
        });
        panel_->AddChild(b);
        panel_->AddFixed(vspacing);

        // Manually erase low-weight (ghost) voxels and reclaim empty blocks.
        auto clean_button = std::make_shared<gui::Button>("Clean ghosts");
        clean_button->SetOnClicked([this]() {
            if (!is_started_ || !model_) {
                return;
            }
            CleanLowWeightVoxels(
                    static_cast<float>(prop_values_.min_weight.load()));
            const int64_t hash_size = model_->GetHashMap().Size();
            RequestAsyncExtract(
                    3.0f, GetLiveExtractBudget(
                                  prop_values_.estimated_points.load(),
                                  hash_size));
        });
        panel_->AddChild(clean_button);
        panel_->AddFixed(vspacing);

        panel_->AddStretch();

        ////////////////////////////////////////
        /// Tabs
        gui::Margins tab_margins(0, int(std::round(0.5f * float(em))), 0, 0);
        auto tabs = std::make_shared<gui::TabControl>();
        panel_->AddChild(tabs);
        auto tab1 = std::make_shared<gui::Vert>(0, tab_margins);
        input_color_image_ = std::make_shared<gui::ImageWidget>();
        input_depth_image_ = std::make_shared<gui::ImageWidget>();
        tab1->AddChild(input_color_image_);
        tab1->AddFixed(vspacing);
        tab1->AddChild(input_depth_image_);
        tabs->AddTab("Input images", tab1);

        auto tab2 = std::make_shared<gui::Vert>(0, tab_margins);
        raycast_color_image_ = std::make_shared<gui::ImageWidget>();
        raycast_depth_image_ = std::make_shared<gui::ImageWidget>();
        tab2->AddChild(raycast_color_image_);
        tab2->AddFixed(vspacing);
        tab2->AddChild(raycast_depth_image_);
        tabs->AddTab("Raycast images", tab2);

        auto tab3 = std::make_shared<gui::Vert>(0, tab_margins);
        output_info_ = std::make_shared<gui::Label>("");
        output_info_->SetFontId(monospace_);
        tab3->AddChild(output_info_);
        tabs->AddTab("Info", tab3);

        widget3d_->SetScene(
                std::make_shared<rendering::Open3DScene>(GetRenderer()));

        output_fps_ = std::make_shared<gui::Label>("FPS: 0.0");
        fps_panel_->AddChild(output_fps_);

        is_done_ = false;
        SetOnClose([this]() {
            is_done_ = true;

            if (is_started_) {
                utility::LogInfo("Writing reconstruction to scene.ply...");
                try {
                    const int save_budget = prop_values_.estimated_points.load();
                    // Use the Min weight slider so low-confidence (ghost)
                    // voxels are excluded from the exported model.
                    const float save_weight = static_cast<float>(
                            prop_values_.min_weight.load());
                    auto pcd = model_->ExtractPointCloudExcludingFrozen(
                            save_weight, save_budget);
                    auto pcd_legacy =
                            std::make_shared<geometry::PointCloud>(pcd.ToLegacy());
                    io::WritePointCloud("scene.ply", *pcd_legacy);
                } catch (const std::exception& e) {
                    utility::LogWarning(
                            "Failed to save scene.ply: {}", e.what());
                }

                if (!frozen_objects_.empty()) {
                    utility::filesystem::MakeDirectoryHierarchy("objects");
                    std::ofstream blocks_json("objects/frozen_blocks.json");
                    blocks_json << "{\n  \"objects\": [\n";
                    for (size_t i = 0; i < frozen_objects_.size(); ++i) {
                        const auto& obj = frozen_objects_[i];
                        // Growing plane surfaces are saved as surface_*.ply,
                        // discrete objects keep the object_*.ply naming.
                        const std::string mesh_path =
                                (object_mesh::IsPlanarType(obj.type)
                                         ? "surface_"
                                         : "object_") +
                                std::to_string(obj.id) + ".ply";
                        try {
                            io::WriteTriangleMesh("objects/" + mesh_path,
                                                  obj.mesh.ToLegacy());
                        } catch (const std::exception& e) {
                            utility::LogWarning(
                                    "Failed to save {}: {}", mesh_path,
                                    e.what());
                        }
                        blocks_json << "    {\n";
                        blocks_json << "      \"id\": " << obj.id << ",\n";
                        blocks_json << "      \"type\": \""
                                    << object_mesh::ObjectTypeName(obj.type)
                                    << "\",\n";
                        blocks_json << "      \"state\": \"frozen\",\n";
                        if (object_mesh::IsPlanarType(obj.type)) {
                            blocks_json << fmt::format(
                                    "      \"plane\": [{:.6f}, {:.6f}, "
                                    "{:.6f}, {:.6f}],\n",
                                    obj.plane(0), obj.plane(1), obj.plane(2),
                                    obj.plane(3));
                        }
                        blocks_json << fmt::format(
                                "      \"area_m2\": {:.3f},\n", obj.area_m2);
                        blocks_json << "      \"mesh\": \"" << mesh_path
                                    << "\"";
                        if (obj.block_keys.NumElements() > 0) {
                            blocks_json << ",\n      \"block_keys\": [";
                            core::Tensor keys =
                                    obj.block_keys.To(core::Device("CPU:0"))
                                            .Contiguous();
                            const int32_t* data = keys.GetDataPtr<int32_t>();
                            for (int64_t k = 0; k < keys.GetLength(); ++k) {
                                if (k > 0) {
                                    blocks_json << ", ";
                                }
                                blocks_json << "[" << data[k * 3 + 0] << ", "
                                            << data[k * 3 + 1] << ", "
                                            << data[k * 3 + 2] << "]";
                            }
                            blocks_json << "]";
                        }
                        blocks_json << "\n    }";
                        if (i + 1 < frozen_objects_.size()) {
                            blocks_json << ",";
                        }
                        blocks_json << "\n";
                    }
                    blocks_json << "  ],\n  \"surfaces\": [\n";
                    const auto surfaces = plane_atlas_.Snapshot();
                    for (size_t i = 0; i < surfaces.size(); ++i) {
                        const auto& s = surfaces[i];
                        blocks_json << fmt::format(
                                "    {{\"id\": {}, \"type\": \"{}\", "
                                "\"plane\": [{:.6f}, {:.6f}, {:.6f}, "
                                "{:.6f}], \"cell_size\": {:.3f}, "
                                "\"cell_count\": {}, \"closure\": {:.3f}, "
                                "\"connected\": [",
                                s.id, object_mesh::ObjectTypeName(s.type),
                                s.plane(0), s.plane(1), s.plane(2),
                                s.plane(3), s.cell_size, s.cell_count,
                                s.closure);
                        for (size_t k = 0; k < s.connected.size(); ++k) {
                            if (k > 0) {
                                blocks_json << ", ";
                            }
                            blocks_json << s.connected[k];
                        }
                        blocks_json << "]}";
                        if (i + 1 < surfaces.size()) {
                            blocks_json << ",";
                        }
                        blocks_json << "\n";
                    }
                    blocks_json << "  ]\n}\n";
                    utility::LogInfo(
                            "Saved {} frozen region meshes ({} surfaces) to "
                            "objects/.",
                            frozen_objects_.size(), surfaces.size());
                }

                utility::LogInfo("Writing trajectory to trajectory.log...");
                io::WritePinholeCameraTrajectory("trajectory.log",
                                                 *trajectory_);
            }
            return true;  // false would cancel the close
        });
        extract_thread_ = std::thread([this]() { this->ExtractWorker(); });
        segmentation_thread_ =
                std::thread([this]() { this->SegmentationWorker(); });
        SetOnTickEvent([this]() {
            if (!slam_thread_started_.exchange(true)) {
                update_thread_ = std::thread([this]() { this->UpdateMain(); });
                StartSlam();
            }
            return false;
        });
    }

    ~ReconstructionWindow() {
        is_done_ = true;
        {
            std::lock_guard<std::mutex> lock(extract_mutex_);
            extract_requested_ = true;
        }
        extract_cv_.notify_all();
        if (extract_thread_.joinable()) {
            extract_thread_.join();
        }
        {
            std::lock_guard<std::mutex> lock(segmentation_mutex_);
            segmentation_requested_ = true;
        }
        segmentation_cv_.notify_all();
        if (segmentation_thread_.joinable()) {
            segmentation_thread_.join();
        }
        if (update_thread_.joinable()) {
            update_thread_.join();
        }
    }

    void Layout(const gui::LayoutContext& context) override {
        int em = context.theme.font_size;
        int panel_width = 20 * em;
        // The usable part of the window may not be the full size if there
        // is a menu.
        auto content_rect = GetContentRect();
        panel_->SetFrame(gui::Rect(content_rect.x, content_rect.y, panel_width,
                                   content_rect.height));
        int x = panel_->GetFrame().GetRight();
        widget3d_->SetFrame(gui::Rect(x, content_rect.y,
                                      content_rect.GetRight() - x,
                                      content_rect.height));

        int fps_panel_width = 7 * em;
        int fps_panel_height = 2 * em;
        fps_panel_->SetFrame(
                gui::Rect(content_rect.GetRight() - fps_panel_width,
                          content_rect.y, fps_panel_width, fps_panel_height));

        // Now that all the children are sized correctly, we can super to
        // layout all their children.
        Super::Layout(context);
    }

    void SetInfo(const std::string& output) {
        output_info_->SetText(output.c_str());
    }
    void SetFPS(const std::string& output) {
        output_fps_->SetText(output.c_str());
    }

protected:
    std::function<t::geometry::RGBDImage(const size_t idx)>
            get_rgbd_image_input_;
    core::Tensor intrinsic_;
    std::unordered_map<std::string, double> default_parameters_;
    core::Device device_;

    // General logic
    std::atomic<bool> is_running_;
    std::atomic<bool> is_started_;
    std::atomic<bool> is_done_;
    bool exit_on_empty_frame_;

    // Panels and controls
    gui::FontId monospace_;
    std::shared_ptr<gui::Vert> panel_;
    std::shared_ptr<gui::Label> output_info_;
    std::shared_ptr<PropertyPanel> fixed_props_;
    std::shared_ptr<PropertyPanel> adjustable_props_;

    std::shared_ptr<gui::SceneWidget> widget3d_;

    std::shared_ptr<gui::Vert> fps_panel_;
    std::shared_ptr<gui::Label> output_fps_;

    // Images
    std::shared_ptr<gui::ImageWidget> input_color_image_;
    std::shared_ptr<gui::ImageWidget> input_depth_image_;
    std::shared_ptr<gui::ImageWidget> raycast_color_image_;
    std::shared_ptr<gui::ImageWidget> raycast_depth_image_;

    struct {
        std::atomic<int> update_interval;
        std::atomic<int> estimated_points;
        std::atomic<int> depth_scale;
        std::atomic<int> block_count;
        std::atomic<int> odometry_iter_0;
        std::atomic<int> odometry_iter_1;
        std::atomic<int> odometry_iter_2;
        std::atomic<int> gui_update_interval;
        std::atomic<double> voxel_size;
        std::atomic<double> trunc_multiplier;
        std::atomic<double> depth_max;
        std::atomic<double> depth_diff;
        std::atomic<double> min_fitness;
        std::atomic<double> min_weight;
        std::atomic<bool> raycast_color;
        std::atomic<bool> update_surface;
        std::atomic<bool> auto_freeze;
        std::atomic<int> stability_frames;
        std::atomic<double> dbscan_eps_multiplier;
        std::atomic<int> min_cluster_points;
        std::atomic<bool> planar_tiles;
        std::atomic<double> tile_size;
    } prop_values_;

    struct {
        std::mutex lock;
        t::geometry::PointCloud pcd;
        std::atomic<uint64_t> version{0};
    } surface_;
    std::shared_ptr<geometry::PointCloud> display_points_legacy_;
    bool points_geometry_added_ = false;
    bool trajectory_geometry_added_ = false;
    int64_t last_render_point_count_ = 0;
    uint64_t last_surface_version_gui_ = 0;
    bool camera_fitted_ = false;
    int consecutive_tracking_failures_ = 0;
    // Relocalization gate state (SLAM thread only).
    bool relocalizing_ = false;
    int relocalize_strong_streak_ = 0;
    // Stationary-integration gate state (SLAM thread only).
    int stationary_frames_ = 0;
    int consecutive_slow_frames_ = 0;
    int capture_skip_count_ = 0;
    std::atomic<bool> slam_thread_started_{false};

    std::shared_ptr<gui::ToggleSwitch> resume_toggle_;

    std::shared_ptr<t::pipelines::slam::Model> model_;
    std::shared_ptr<camera::PinholeCameraTrajectory> trajectory_;
    std::thread update_thread_;
    std::thread extract_thread_;
    std::mutex model_mutex_;
    std::mutex gui_sync_mutex_;
    std::condition_variable gui_sync_cv_;
    std::atomic<int> pending_gui_posts_{0};
    std::mutex extract_mutex_;
    std::condition_variable extract_cv_;
    std::atomic<bool> extract_requested_{false};
    float pending_extract_weight_threshold_ = 3.0f;
    int pending_extract_budget_ = 0;

    struct FrozenObjectEntry {
        int id = -1;
        object_mesh::ObjectType type = object_mesh::ObjectType::kGeneric;
        t::geometry::TriangleMesh mesh;
        core::Tensor block_keys;
        Eigen::Vector4d plane = Eigen::Vector4d::Zero();
        int tile_i = 0;
        int tile_j = 0;
        double area_m2 = 0.0;
        double cell_size = 0.0;
        int cell_count = 0;
        double closure = 0.0;
        std::vector<int> connected;
    };
    std::vector<FrozenObjectEntry> frozen_objects_;
    std::mutex frozen_mutex_;
    std::atomic<uint64_t> frozen_version_{0};
    uint64_t last_frozen_version_gui_ = 0;
    // Region partitioning/indexing: block-key spatial index + state machine.
    object_mesh::RegionRegistry region_registry_;
    // Growing plane surfaces (open quad meshes, merged and corner-snapped).
    object_mesh::PlaneSurfaceAtlas plane_atlas_;
    int next_object_id_ = 0;

    std::thread segmentation_thread_;
    std::mutex segmentation_mutex_;
    std::condition_variable segmentation_cv_;
    std::atomic<bool> segmentation_requested_{false};
    t::geometry::PointCloud pending_segmentation_pcd_;

    void WaitForPendingGui() {
        std::unique_lock<std::mutex> lock(gui_sync_mutex_);
        gui_sync_cv_.wait(lock, [this]() {
            return pending_gui_posts_.load() == 0;
        });
    }

    void PostGuiTask(std::function<void()> task) {
        pending_gui_posts_.fetch_add(1);
        gui::Application::GetInstance().PostToMainThread(this, [this, task]() {
            task();
            pending_gui_posts_.fetch_sub(1);
            gui_sync_cv_.notify_all();
        });
    }

    std::vector<t::pipelines::odometry::OdometryConvergenceCriteria>
    BuildOdometryCriteria() {
        return {{prop_values_.odometry_iter_0.load()},
                {prop_values_.odometry_iter_1.load()},
                {prop_values_.odometry_iter_2.load()}};
    }

    void RequestAsyncExtract(float weight_threshold, int extract_budget) {
        {
            std::lock_guard<std::mutex> lock(extract_mutex_);
            pending_extract_weight_threshold_ = weight_threshold;
            pending_extract_budget_ = extract_budget;
            extract_requested_ = true;
        }
        extract_cv_.notify_one();
    }

    // Decay TSDF confidence of voxels that the current (trusted) frame
    // observes as free space, i.e. well in front of the measured surface.
    // The Integrate kernel ignores such observations (sdf < -sdf_trunc), so
    // ghost walls stamped by a mis-tracked pose would otherwise stay forever.
    // Halving the weight per pass makes ghosts drop below the extraction /
    // raycast weight threshold within a few passes once the area is rescanned
    // from a correct pose. Caller must hold model_mutex_.
    void CarveFreeSpace(const core::Tensor& depth_tensor,
                        const core::Tensor& T_frame_to_model,
                        float depth_scale,
                        float depth_max) {
        auto& grid = model_->voxel_grid_;
        core::HashMap hashmap = grid.GetHashMap();
        core::Tensor active_indices = hashmap.GetActiveIndices();
        if (active_indices.GetLength() == 0) {
            return;
        }

        core::Tensor weight = grid.GetAttribute("weight");
        const int64_t resolution = weight.GetShape()[1];
        const float voxel_size =
                static_cast<float>(prop_values_.voxel_size.load());
        const float sdf_trunc =
                voxel_size *
                static_cast<float>(prop_values_.trunc_multiplier.load());
        // Conservative margin: only voxels clearly in front of the observed
        // surface are carved, so thin real structures are not eroded.
        const float carve_margin = kCarveMarginTruncFactor * sdf_trunc;

        core::Tensor K_cpu =
                intrinsic_.To(core::Device("CPU:0"), core::Float64)
                        .Contiguous();
        const double* k_ptr = K_cpu.GetDataPtr<double>();
        const float fx = static_cast<float>(k_ptr[0]);
        const float fy = static_cast<float>(k_ptr[4]);
        const float cx = static_cast<float>(k_ptr[2]);
        const float cy = static_cast<float>(k_ptr[5]);
        core::Tensor T_world_to_cam =
                T_frame_to_model.Inverse().To(device_, core::Float32);
        core::Tensor R_t = T_world_to_cam.Slice(0, 0, 3)
                                   .Slice(1, 0, 3)
                                   .T()
                                   .Contiguous();
        core::Tensor t_vec = T_world_to_cam.Slice(0, 0, 3)
                                     .Slice(1, 3, 4)
                                     .Contiguous()
                                     .Reshape({1, 3});

        const int64_t rows = depth_tensor.GetShape()[0];
        const int64_t cols = depth_tensor.GetShape()[1];

        // Block-level frustum prefilter (projected block centers, with a
        // half-diagonal margin) keeps the per-voxel pass tractable.
        const float block_size = voxel_size * static_cast<float>(resolution);
        const float block_radius = 0.87f * block_size;
        core::Tensor block_centers =
                (hashmap.GetKeyTensor()
                                 .IndexGet({active_indices.To(core::Int64)})
                                 .To(core::Float32) +
                 0.5f) *
                block_size;
        core::Tensor centers_cam = block_centers.Matmul(R_t) + t_vec;
        core::Tensor xc = centers_cam.Slice(1, 0, 1).Contiguous().Reshape({-1});
        core::Tensor yc = centers_cam.Slice(1, 1, 2).Contiguous().Reshape({-1});
        core::Tensor zc = centers_cam.Slice(1, 2, 3).Contiguous().Reshape({-1});
        core::Tensor uc = xc / zc * fx + cx;
        core::Tensor vc = yc / zc * fy + cy;
        core::Tensor margin_u = (fx * block_radius) / zc;
        core::Tensor margin_v = (fy * block_radius) / zc;
        core::Tensor in_frustum =
                zc.Gt(0.05f)
                        .LogicalAnd(zc.Lt(depth_max + block_radius))
                        .LogicalAnd(uc.Ge(0.0f - margin_u))
                        .LogicalAnd(uc.Lt(margin_u +
                                          static_cast<float>(cols)))
                        .LogicalAnd(vc.Ge(0.0f - margin_v))
                        .LogicalAnd(vc.Lt(margin_v +
                                          static_cast<float>(rows)));
        core::Tensor frustum_indices = active_indices.IndexGet({in_frustum});
        if (frustum_indices.GetLength() == 0) {
            return;
        }

        // Per-voxel pass: project voxel centers into the depth image and
        // compare against the measured depth.
        core::Tensor voxel_coords, flat_indices;
        std::tie(voxel_coords, flat_indices) =
                grid.GetVoxelCoordinatesAndFlattenedIndices(frustum_indices);
        core::Tensor cam = voxel_coords.Matmul(R_t) + t_vec;
        core::Tensor x = cam.Slice(1, 0, 1).Contiguous().Reshape({-1});
        core::Tensor y = cam.Slice(1, 1, 2).Contiguous().Reshape({-1});
        core::Tensor z = cam.Slice(1, 2, 3).Contiguous().Reshape({-1});
        core::Tensor ui = (x / z * fx + cx).Floor().To(core::Int64);
        core::Tensor vi = (y / z * fy + cy).Floor().To(core::Int64);
        core::Tensor valid = z.Gt(0.05f)
                                     .LogicalAnd(ui.Ge(0))
                                     .LogicalAnd(ui.Lt(cols))
                                     .LogicalAnd(vi.Ge(0))
                                     .LogicalAnd(vi.Lt(rows));
        ui = ui.IndexGet({valid});
        vi = vi.IndexGet({valid});
        z = z.IndexGet({valid});
        flat_indices = flat_indices.IndexGet({valid});
        if (flat_indices.GetLength() == 0) {
            return;
        }

        core::Tensor depth_flat =
                depth_tensor.Reshape({rows * cols}).To(core::Float32) /
                depth_scale;
        core::Tensor d = depth_flat.IndexGet({vi * cols + ui});
        core::Tensor carve_mask = d.Gt(0.0f)
                                          .LogicalAnd(d.Le(depth_max))
                                          .LogicalAnd((d - z).Gt(carve_margin));
        core::Tensor carve_indices = flat_indices.IndexGet({carve_mask});
        if (carve_indices.GetLength() == 0) {
            return;
        }

        core::Tensor weight_flat = weight.View({weight.NumElements()});
        core::Tensor w = weight_flat.IndexGet({carve_indices}).To(core::Int32);
        // Only touch voxels that still carry weight (most free-space voxels
        // in allocated blocks are already 0).
        core::Tensor occupied = w.Gt(0);
        carve_indices = carve_indices.IndexGet({occupied});
        if (carve_indices.GetLength() == 0) {
            return;
        }
        w = w.IndexGet({occupied});
        weight_flat.IndexSet({carve_indices}, (w / 2).To(weight.GetDtype()));
        utility::LogDebug("Free-space carving: decayed {} voxels.",
                          carve_indices.GetLength());
    }

    // Reset all voxels whose weight is below \p min_weight (typical for ghost
    // surfaces integrated only briefly with a bad pose) and erase blocks that
    // become completely empty, reclaiming voxel hash capacity. Blocks are
    // zeroed before Erase so a reused hash slot integrates from scratch.
    void CleanLowWeightVoxels(float min_weight) {
        std::lock_guard<std::mutex> lock(model_mutex_);
        if (!model_) {
            return;
        }
        auto& grid = model_->voxel_grid_;
        core::HashMap hashmap = grid.GetHashMap();
        core::Tensor buf_indices = hashmap.GetActiveIndices().To(core::Int64);
        const int64_t num_blocks = buf_indices.GetLength();
        if (num_blocks == 0) {
            return;
        }

        core::Tensor weight = grid.GetAttribute("weight");
        core::Tensor tsdf = grid.GetAttribute("tsdf");

        // Chunked to bound temporary memory (a full 40k-block volume would
        // need several hundred MB of masks at once).
        constexpr int64_t kChunkBlocks = 8192;
        std::vector<core::Tensor> empty_block_keys;
        for (int64_t start = 0; start < num_blocks; start += kChunkBlocks) {
            core::Tensor idx = buf_indices.Slice(
                    0, start, std::min(start + kChunkBlocks, num_blocks));
            core::Tensor w = weight.IndexGet({idx});
            core::Tensor keep = w.Ge(min_weight);
            weight.IndexSet({idx}, w * keep.To(weight.GetDtype()));
            core::Tensor t = tsdf.IndexGet({idx});
            tsdf.IndexSet({idx}, t * keep.To(tsdf.GetDtype()));

            const int64_t m = idx.GetLength();
            core::Tensor kept_per_block =
                    keep.To(core::Int32).Reshape({m, -1}).Sum({1});
            core::Tensor empty_idx =
                    idx.IndexGet({kept_per_block.Eq(0)});
            if (empty_idx.GetLength() > 0) {
                empty_block_keys.push_back(
                        hashmap.GetKeyTensor().IndexGet({empty_idx}));
            }
        }

        int64_t erased = 0;
        if (!empty_block_keys.empty()) {
            core::Tensor keys = core::Concatenate(empty_block_keys, 0);
            hashmap.Erase(keys);
            erased = keys.GetLength();
        }
        utility::LogInfo(
                "Clean ghosts: reset voxels with weight < {}, erased {} empty "
                "blocks ({} -> {} active).",
                min_weight, erased, num_blocks, hashmap.Size());
    }

    object_mesh::SegmentationConfig BuildSegmentationConfig() const {
        object_mesh::SegmentationConfig config;
        config.auto_freeze = prop_values_.auto_freeze.load();
        config.stability_frames = prop_values_.stability_frames.load();
        config.min_cluster_points = prop_values_.min_cluster_points.load();
        config.voxel_size =
                static_cast<float>(prop_values_.voxel_size.load());
        config.trunc_multiplier =
                static_cast<float>(prop_values_.trunc_multiplier.load());
        config.dbscan_eps = prop_values_.dbscan_eps_multiplier.load() *
                              prop_values_.voxel_size.load();
        config.use_planar_patches = prop_values_.planar_tiles.load();
        config.tile_size = prop_values_.tile_size.load();
        return config;
    }

    void AddFrozenMeshesToScene(
            const std::vector<FrozenObjectEntry>& new_objects) {
        using namespace rendering;
        auto o3d_scene = widget3d_->GetScene();
        MaterialRecord mesh_mat;
        mesh_mat.shader = "defaultLit";
        mesh_mat.sRGB_vertex_color = true;
        for (const auto& obj : new_objects) {
            if (!obj.mesh.HasVertexPositions()) {
                continue;
            }
            const std::string name =
                    "region_" + std::to_string(obj.id);
            if (o3d_scene->HasGeometry(name)) {
                o3d_scene->RemoveGeometry(name);
            }
            o3d_scene->AddGeometry(name, &obj.mesh, mesh_mat, false);
        }
    }

    void SegmentationWorker() {
        while (!is_done_) {
            t::geometry::PointCloud segmentation_pcd;
            {
                std::unique_lock<std::mutex> lock(segmentation_mutex_);
                segmentation_cv_.wait(lock, [this]() {
                    return segmentation_requested_ || is_done_;
                });
                if (is_done_) {
                    break;
                }
                segmentation_requested_ = false;
                segmentation_pcd = pending_segmentation_pcd_;
            }

            if (!is_started_ || !model_ || segmentation_pcd.IsEmpty()) {
                continue;
            }

            try {
                object_mesh::SegmentationConfig config =
                        BuildSegmentationConfig();
                std::vector<FrozenObjectEntry> new_objects;
                {
                    std::lock_guard<std::mutex> model_lock(model_mutex_);
                    auto candidates = object_mesh::ProcessExtractedSurface(
                            segmentation_pcd, *model_, region_registry_,
                            plane_atlas_, config, next_object_id_);
                    for (auto& candidate : candidates) {
                        if (!candidate.mesh.HasVertexPositions()) {
                            continue;
                        }
                        FrozenObjectEntry entry;
                        entry.id = candidate.id;
                        entry.type = candidate.type;
                        entry.mesh = std::move(candidate.mesh);
                        entry.block_keys = candidate.block_keys;
                        entry.plane = candidate.plane;
                        entry.tile_i = candidate.tile_i;
                        entry.tile_j = candidate.tile_j;
                        entry.area_m2 = candidate.area_m2;
                        entry.cell_size = candidate.cell_size;
                        entry.cell_count = candidate.cell_count;
                        entry.closure = candidate.closure;
                        entry.connected = candidate.connected;
                        new_objects.push_back(std::move(entry));
                    }
                }

                if (!new_objects.empty()) {
                    {
                        std::lock_guard<std::mutex> lock(frozen_mutex_);
                        // Growing surfaces re-emit the same id: replace the
                        // stored entry so JSON/scene stay deduplicated.
                        for (auto& entry : new_objects) {
                            bool replaced = false;
                            for (auto& existing : frozen_objects_) {
                                if (existing.id == entry.id) {
                                    existing = entry;
                                    replaced = true;
                                    break;
                                }
                            }
                            if (!replaced) {
                                frozen_objects_.push_back(entry);
                            }
                        }
                    }
                    frozen_version_.fetch_add(1);
                    utility::LogInfo(
                            "Updated {} frozen region(s). Total: {}.",
                            new_objects.size(), frozen_objects_.size());
                    PostGuiTask([this, new_objects]() {
                        AddFrozenMeshesToScene(new_objects);
                    });
                }
            } catch (const std::exception& e) {
                utility::LogWarning("Object segmentation failed: {}", e.what());
            }
        }
    }

    void ExtractWorker() {
        while (!is_done_) {
            {
                std::unique_lock<std::mutex> lock(extract_mutex_);
                extract_cv_.wait(lock, [this]() {
                    return extract_requested_ || is_done_;
                });
                if (is_done_) {
                    break;
                }
                extract_requested_ = false;
            }

            if (!is_started_ || !model_) {
                continue;
            }

            const float weight_threshold = pending_extract_weight_threshold_;
            const int extract_budget = pending_extract_budget_;
            try {
                t::geometry::PointCloud extracted;
                {
                    std::lock_guard<std::mutex> model_lock(model_mutex_);
                    extracted =
                            model_->ExtractPointCloudExcludingFrozen(
                                    weight_threshold, extract_budget)
                                    .To(core::Device("CPU:0"));
                }
                {
                    std::lock_guard<std::mutex> locker(surface_.lock);
                    surface_.pcd = std::move(extracted);
                    surface_.version.fetch_add(1);
                }
                if (prop_values_.auto_freeze.load()) {
                    t::geometry::PointCloud segmentation_copy;
                    {
                        std::lock_guard<std::mutex> locker(surface_.lock);
                        segmentation_copy = surface_.pcd;
                    }
                    {
                        std::lock_guard<std::mutex> lock(segmentation_mutex_);
                        pending_segmentation_pcd_ = std::move(segmentation_copy);
                        segmentation_requested_ = true;
                    }
                    segmentation_cv_.notify_one();
                }
            } catch (const std::exception& e) {
                utility::LogWarning("Async surface extract failed: {}",
                                    e.what());
            }
        }
    }

    t::geometry::PointCloud PrepareRenderPointCloud(
            const t::geometry::PointCloud& surface_pcd) {
        if (!surface_pcd.HasPointPositions() ||
            !surface_pcd.HasPointColors()) {
            return {};
        }
        const int64_t raw_count =
                surface_pcd.GetPointPositions().GetLength();
        t::geometry::PointCloud render_pcd = surface_pcd;
        if (raw_count > kMaxRenderPoints) {
            const double vs = prop_values_.voxel_size.load();
            render_pcd = surface_pcd.VoxelDownSample(vs * 2.0);
            const int64_t down_count =
                    render_pcd.GetPointPositions().GetLength();
            if (down_count > kMaxRenderPoints) {
                render_pcd = render_pcd.RandomDownSample(
                        static_cast<double>(kMaxRenderPoints) /
                        static_cast<double>(down_count));
            }
        }
        return render_pcd;
    }

    void UpdateSurfaceGeometryOnScene(
            const t::geometry::PointCloud& surface_pcd) {
        using namespace rendering;
        t::geometry::PointCloud render_pcd =
                PrepareRenderPointCloud(surface_pcd);
        if (!render_pcd.HasPointPositions() || !render_pcd.HasPointColors()) {
            return;
        }
        const int64_t render_count =
                render_pcd.GetPointPositions().GetLength();
        if (render_count <= 0) {
            return;
        }

        auto o3d_scene = widget3d_->GetScene();
        MaterialRecord pcd_mat;
        pcd_mat.shader = "defaultUnlit";
        pcd_mat.sRGB_vertex_color = true;

        display_points_legacy_ =
                std::make_shared<geometry::PointCloud>(render_pcd.ToLegacy());
        if (points_geometry_added_) {
            o3d_scene->RemoveGeometry("points");
        }
        o3d_scene->AddGeometry("points", display_points_legacy_.get(), pcd_mat,
                               false);
        points_geometry_added_ = true;
        last_render_point_count_ = render_count;

        auto tbbox = render_pcd.GetAxisAlignedBoundingBox();
        geometry::AxisAlignedBoundingBox bbox = tbbox.ToLegacy();
        if (bbox.Volume() > 0 && !camera_fitted_) {
            // World frame equals the first camera frame (x right, y DOWN,
            // z forward), so the default bbox fit shows the room upside
            // down. Fit the projection, then enforce an upright view
            // (up = -y) from the initial camera side. Fit only once: later
            // bbox changes (e.g. live points shrinking when surfaces are
            // frozen) must not re-orient the view.
            const Eigen::Vector3f center = bbox.GetCenter().cast<float>();
            widget3d_->SetupCamera(60.0f, bbox, center);
            Eigen::Vector3f up(0.0f, -1.0f, 0.0f);
            Eigen::Vector3f dir = center;
            if (dir.norm() < 0.3f) {
                dir = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
            }
            dir.normalize();
            if (std::abs(dir.dot(up)) > 0.95f) {
                // Looking straight up/down: use z as the up direction.
                up = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
            }
            const float dist = std::max(
                    1.0f,
                    1.5f * static_cast<float>(bbox.GetExtent().norm()));
            widget3d_->LookAt(center, center - dir * dist, up);
            camera_fitted_ = true;
        }
    }

    void InitSlamModel() {
        if (is_started_) {
            return;
        }
        trajectory_ = std::make_shared<camera::PinholeCameraTrajectory>();

        float voxel_size = prop_values_.voxel_size;
        const int requested_capacity = prop_values_.block_count.load();
        const int init_capacity = static_cast<int>(std::min(
                static_cast<int64_t>(requested_capacity), kInitialHashCapacity));
        model_ = std::make_shared<t::pipelines::slam::Model>(
                voxel_size, 16, init_capacity,
                core::Tensor::Eye(4, core::Dtype::Float64,
                                  core::Device("CPU:0")),
                device_);

        const int64_t target_capacity = std::min(
                static_cast<int64_t>(requested_capacity), kMaxHashCapacity);
        if (target_capacity > model_->GetHashMap().GetCapacity()) {
            try {
                model_->GetHashMap().Reserve(target_capacity);
                utility::LogInfo(
                        "Voxel hash expanded to {} blocks (requested {}).",
                        model_->GetHashMap().GetCapacity(), requested_capacity);
            } catch (const std::exception& e) {
                utility::LogWarning(
                        "Could not expand voxel hash to {}: {}. Continuing "
                        "with {} blocks. Try --profile low or reduce "
                        "block_count.",
                        target_capacity, e.what(),
                        model_->GetHashMap().GetCapacity());
            }
        }
        const int64_t hash_cap = model_->GetHashMap().GetCapacity();
        const int est_points = prop_values_.estimated_points.load();
        utility::LogInfo(
                "VRAM budget (estimate): TSDF ~{} MB ({} blocks), extract "
                "buffer ~{} MB ({} points).",
                EstimateTsdfVramMb(static_cast<int>(hash_cap)), hash_cap,
                est_points * 16 / (1024 * 1024), est_points);
        utility::LogInfo("SLAM hash capacity: {}/{} blocks.",
                        model_->GetHashMap().Size(), hash_cap);
        is_started_ = true;
    }

    void StartSlam() {
        is_running_ = true;
        if (resume_toggle_) {
            resume_toggle_->SetOn(true);
        }
        adjustable_props_->SetEnabled(true);
    }

    void PauseSlam() {
        is_running_ = false;
        if (resume_toggle_) {
            resume_toggle_->SetOn(false);
        }
    }

    t::geometry::RGBDImage CaptureInputFrame(size_t idx) {
        try {
            return get_rgbd_image_input_(idx);
        } catch (const std::exception& e) {
            utility::LogWarning("CaptureFrame failed: {}", e.what());
            return t::geometry::RGBDImage();
        }
    }

    bool HandleEmptyFrame(size_t idx, const char* context) {
        if (!exit_on_empty_frame_) {
            utility::LogWarning(
                    "Empty frame at {} (idx {}), retrying capture...",
                    context, idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            return false;
        }
        utility::LogInfo("Reached EOF. Empty frame received.");
        is_done_ = true;
        return true;
    }

protected:
    // Note that we cannot update the GUI on this thread, we must post to
    // the main thread!
    void UpdateMain() {
        try {
            UpdateMainImpl();
        } catch (const std::exception& e) {
            utility::LogWarning("UpdateMain failed: {}", e.what());
        } catch (...) {
            utility::LogWarning("UpdateMain failed with unknown exception.");
        }
    }

    void UpdateMainImpl() {
        while (!is_running_.load() && !is_done_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (is_done_) {
            return;
        }

        InitSlamModel();
        if (!model_) {
            utility::LogWarning("SLAM model initialization failed.");
            return;
        }

        // Only set at initialization
        float depth_scale = prop_values_.depth_scale;
        core::Tensor T_frame_to_model = core::Tensor::Eye(
                4, core::Dtype::Float64, core::Device("CPU:0"));

        t::geometry::RGBDImage ref_rgbd_cpu = CaptureInputFrame(0);
        if (ref_rgbd_cpu.IsEmpty()) {
            if (HandleEmptyFrame(0, "startup")) {
                return;
            }
            ref_rgbd_cpu = CaptureInputFrame(0);
            if (ref_rgbd_cpu.IsEmpty() &&
                HandleEmptyFrame(0, "startup_retry")) {
                return;
            }
        }
        if (ref_rgbd_cpu.IsEmpty()) {
            return;
        }
        t::geometry::RGBDImage ref_rgbd_input = ref_rgbd_cpu.To(device_);

        t::pipelines::slam::Frame input_frame(ref_rgbd_input.depth_.GetRows(),
                                              ref_rgbd_input.depth_.GetCols(),
                                              intrinsic_, device_);
        t::pipelines::slam::Frame raycast_frame(ref_rgbd_input.depth_.GetRows(),
                                                ref_rgbd_input.depth_.GetCols(),
                                                intrinsic_, device_);

        // Odometry
        auto traj = std::make_shared<geometry::LineSet>();
        auto frustum = std::make_shared<geometry::LineSet>();
        auto color = std::make_shared<geometry::Image>();
        auto depth_colored = std::make_shared<geometry::Image>();

        auto raycast_color = std::make_shared<geometry::Image>();
        auto raycast_depth_colored = std::make_shared<geometry::Image>();

        color = std::make_shared<geometry::Image>(
                ref_rgbd_input.color_.To(core::Device("CPU:0")).ToLegacy());

        depth_colored = std::make_shared<geometry::Image>(
                ref_rgbd_input.depth_.To(core::Device("CPU:0"))
                        .ColorizeDepth(
                                static_cast<float>(depth_scale), 0.3,
                                prop_values_.depth_max.load())
                        .ToLegacy());

        raycast_color = std::make_shared<geometry::Image>(
                t::geometry::Image(core::Tensor::Zeros(
                                           {ref_rgbd_input.depth_.GetRows(),
                                            ref_rgbd_input.depth_.GetCols(), 3},
                                           core::Dtype::UInt8,
                                           core::Device("CPU:0")))
                        .ToLegacy());
        raycast_depth_colored = std::make_shared<geometry::Image>(
                t::geometry::Image(core::Tensor::Zeros(
                                           {ref_rgbd_input.depth_.GetRows(),
                                            ref_rgbd_input.depth_.GetCols(), 3},
                                           core::Dtype::UInt8,
                                           core::Device("CPU:0")))
                        .ToLegacy());

        // Placeholder color must live on the same device as raycast_frame.
        raycast_frame.SetData(
                "color",
                core::Tensor::Zeros({ref_rgbd_input.depth_.GetRows(),
                                     ref_rgbd_input.depth_.GetCols(), 3},
                                    core::Dtype::UInt8, device_));

        camera::PinholeCameraParameters traj_param;
        traj_param.intrinsic_ = camera::PinholeCameraIntrinsic(
                ref_rgbd_input.depth_.GetRows(),
                ref_rgbd_input.depth_.GetCols(),
                core::eigen_converter::TensorToEigenMatrixXd(intrinsic_));

        Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", "[",
                                 "]");

        const int fps_interval_len = 30;
        double time_interval = 0;
        size_t idx = 0;

        utility::Timer timer;
        timer.Start();
        while (!is_done_) {
            float depth_scale = prop_values_.depth_scale;

            if (!is_started_ || !is_running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            if (capture_skip_count_ > 0) {
                CaptureInputFrame(idx);
                --capture_skip_count_;
                ++idx;
                continue;
            }

            utility::Timer frame_timer;
            frame_timer.Start();

            try {
            t::geometry::RGBDImage rgbd_input =
                    CaptureInputFrame(idx).To(device_);
            if (rgbd_input.IsEmpty()) {
                if (HandleEmptyFrame(idx, "loop")) {
                    break;
                }
                continue;
            }

            // Input
            input_frame.SetDataFromImage("depth", rgbd_input.depth_);
            input_frame.SetDataFromImage("color", rgbd_input.color_);

            const int64_t hash_size_before = model_->GetHashMap().Size();
            const int64_t hash_capacity = model_->GetHashMap().GetCapacity();
            const bool hash_near_full =
                    hash_size_before >
                    static_cast<int64_t>(hash_capacity * kHashIntegrateFillRatio);

            bool tracking_success = true;
            TrackingTier tracking_tier = TrackingTier::kInit;
            double track_fitness = 1.0;
            double track_translation = 0.0;
            double track_rotation_deg = 0.0;
            if (idx > 0 && !hash_near_full) {
                const double min_fitness = std::max(
                        kWeakFitnessMin, prop_values_.min_fitness.load());
                auto classify_tracking =
                        [min_fitness](double fitness, double translation,
                                      double rotation_deg) -> TrackingTier {
                    if (translation >= kOutlierTranslation ||
                        rotation_deg >= kOutlierRotationDeg) {
                        return TrackingTier::kOutlier;
                    }
                    if (fitness >= min_fitness &&
                        translation < kPoseTranslationMax &&
                        rotation_deg < kPoseRotationMaxDeg) {
                        return TrackingTier::kStrong;
                    }
                    if (fitness >= kWeakFitnessMin &&
                        translation < kWeakTranslationMax) {
                        return TrackingTier::kWeak;
                    }
                    return TrackingTier::kFail;
                };
                // Per-frame motion magnitude from the odometry delta.
                auto measure_motion = [](const core::Tensor& transformation,
                                         double& translation,
                                         double& rotation_deg) {
                    core::Tensor T_cpu =
                            transformation
                                    .To(core::Device("CPU:0"), core::Float64)
                                    .Contiguous();
                    const double* t_ptr = T_cpu.GetDataPtr<double>();
                    translation = std::sqrt(t_ptr[3] * t_ptr[3] +
                                            t_ptr[7] * t_ptr[7] +
                                            t_ptr[11] * t_ptr[11]);
                    const double trace = t_ptr[0] + t_ptr[5] + t_ptr[10];
                    const double cos_angle = std::max(
                            -1.0, std::min(1.0, 0.5 * (trace - 1.0)));
                    rotation_deg = std::acos(cos_angle) *
                                   (180.0 / 3.14159265358979323846);
                };

                try {
                    const auto criteria = BuildOdometryCriteria();
                    auto run_tracking =
                            [&](float depth_diff) {
                                return model_->TrackFrameToModel(
                                        input_frame, raycast_frame, depth_scale,
                                        prop_values_.depth_max,
                                        SafeOdometryDepthDiff(depth_diff),
                                        t::pipelines::odometry::Method::PointToPlane,
                                        criteria);
                            };

                    std::lock_guard<std::mutex> model_lock(model_mutex_);
                    auto result = run_tracking(
                            static_cast<float>(prop_values_.depth_diff));
                    measure_motion(result.transformation_, track_translation,
                                   track_rotation_deg);
                    track_fitness = result.fitness_;
                    tracking_tier = classify_tracking(track_fitness,
                                                      track_translation,
                                                      track_rotation_deg);

                    if (tracking_tier == TrackingTier::kFail &&
                        track_fitness < kWeakFitnessMin) {
                        result = run_tracking(static_cast<float>(
                                prop_values_.depth_diff * 2.0));
                        measure_motion(result.transformation_,
                                       track_translation, track_rotation_deg);
                        track_fitness = result.fitness_;
                        tracking_tier = classify_tracking(track_fitness,
                                                          track_translation,
                                                          track_rotation_deg);
                    }

                    if (tracking_tier == TrackingTier::kStrong) {
                        // Strong frames always update the pose so the raycast
                        // follows the camera (also during relocalization).
                        T_frame_to_model =
                                T_frame_to_model.Matmul(result.transformation_);
                    }
                } catch (const std::exception& e) {
                    tracking_tier = TrackingTier::kFail;
                    utility::LogWarning(
                            "Tracking exception for frame {}: {}", idx,
                            e.what());
                }

                if (tracking_tier == TrackingTier::kStrong) {
                    consecutive_tracking_failures_ = 0;
                    if (relocalizing_) {
                        // Integration stays paused until enough consecutive
                        // Strong frames confirm the pose is locked again.
                        ++relocalize_strong_streak_;
                        if (relocalize_strong_streak_ >=
                            kRelocalizeStrongFrames) {
                            relocalizing_ = false;
                            relocalize_strong_streak_ = 0;
                            tracking_success = true;
                            utility::LogInfo(
                                    "Relocalized at frame {}. Resuming "
                                    "integration.",
                                    idx);
                        } else {
                            tracking_success = false;
                        }
                    } else {
                        tracking_success = true;
                    }
                } else {
                    // Weak / failed / outlier: never integrate and never move
                    // the pose; a wrong pose would stamp a duplicate surface.
                    tracking_success = false;
                    relocalize_strong_streak_ = 0;
                    ++consecutive_tracking_failures_;
                    if (tracking_tier == TrackingTier::kWeak) {
                        utility::LogDebug(
                                "Weak tracking frame {}: fitness {:.3f}, "
                                "translation {:.3f} m, rotation {:.1f} deg. "
                                "Skipping integration.",
                                idx, track_fitness, track_translation,
                                track_rotation_deg);
                    } else {
                        const char* tier_name =
                                tracking_tier == TrackingTier::kOutlier
                                        ? "outlier"
                                        : "failed";
                        utility::LogWarning(
                                "Tracking {} for frame {}, fitness: {:.3f}, "
                                "translation: {:.3f} m, rotation: {:.1f} deg. "
                                "Skipping integration.",
                                tier_name, idx, track_fitness,
                                track_translation, track_rotation_deg);
                    }
                    if (!relocalizing_ &&
                        consecutive_tracking_failures_ >=
                                kRelocalizeAfterFailures) {
                        relocalizing_ = true;
                        utility::LogWarning(
                                "Tracking unreliable for {} frames. "
                                "Relocalizing: integration paused until {} "
                                "consecutive strong frames.",
                                consecutive_tracking_failures_,
                                kRelocalizeStrongFrames);
                    }
                }
            } else if (idx > 0 && hash_near_full) {
                tracking_success = false;
                tracking_tier = TrackingTier::kFail;
            }

            // Stationary gate: with the camera still, voxel weights are
            // already saturated, so further integration only wastes GPU time
            // and lets depth noise allocate new blocks.
            if (tracking_tier == TrackingTier::kStrong && !relocalizing_ &&
                track_translation < kStationaryTranslationMax &&
                track_rotation_deg < kStationaryRotationMaxDeg) {
                ++stationary_frames_;
            } else {
                stationary_frames_ = 0;
            }
            const bool integration_idle =
                    stationary_frames_ >= kStationaryFrames;

            model_->UpdateFramePose(idx, T_frame_to_model);
            const bool integrated = tracking_success && !integration_idle;
            {
                std::lock_guard<std::mutex> model_lock(model_mutex_);
                if (integrated && !hash_near_full) {
                    model_->Integrate(input_frame, depth_scale,
                                      prop_values_.depth_max,
                                      prop_values_.trunc_multiplier);
                    // Periodically erase ghost voxels observed as free space
                    // by this trusted (Strong-tracked) frame.
                    const int carve_interval = device_.IsCUDA()
                                                       ? kCarveIntervalCuda
                                                       : kCarveIntervalCpu;
                    if (tracking_tier == TrackingTier::kStrong &&
                        idx % carve_interval == 0) {
                        try {
                            utility::Timer carve_timer;
                            carve_timer.Start();
                            CarveFreeSpace(
                                    input_frame.GetDataAsImage("depth")
                                            .AsTensor(),
                                    T_frame_to_model, depth_scale,
                                    static_cast<float>(
                                            prop_values_.depth_max.load()));
                            carve_timer.Stop();
                            utility::LogDebug(
                                    "Free-space carving took {:.1f} ms.",
                                    carve_timer.GetDurationInMillisecond());
                        } catch (const std::exception& e) {
                            utility::LogWarning(
                                    "Free-space carving failed: {}", e.what());
                        }
                    }
                } else if (integrated && hash_near_full) {
                    utility::LogWarning(
                            "Voxel hash map nearly full ({}/{}). Skipping "
                            "integration to avoid GPU rehash failure.",
                            hash_size_before, hash_capacity);
                }
                model_->SynthesizeModelFrame(
                        raycast_frame, depth_scale, 0.1,
                        prop_values_.depth_max, prop_values_.trunc_multiplier,
                        prop_values_.raycast_color);
            }

            auto K_eigen =
                    core::eigen_converter::TensorToEigenMatrixXd(intrinsic_);
            auto T_eigen = core::eigen_converter::TensorToEigenMatrixXd(
                    T_frame_to_model);
            traj_param.extrinsic_ = T_eigen;
            trajectory_->parameters_.push_back(traj_param);

            traj->points_.push_back(T_eigen.block<3, 1>(0, 3));
            if (traj->points_.size() > 1) {
                int n = traj->points_.size();
                traj->lines_.push_back({n - 1, n - 2});
                traj->colors_.push_back(kTangoSkyBlueDark);
            }

            if (idx % fps_interval_len == 0) {
                timer.Stop();
                time_interval = timer.GetDurationInMillisecond();
                timer.Start();
            }

            const int gui_interval = std::max(
                    1, prop_values_.gui_update_interval.load());
            const int64_t hash_size = model_->GetHashMap().Size();
            const int effective_interval = GetEffectiveUpdateInterval(
                    static_cast<int>(prop_values_.update_interval), hash_size);
            const bool should_request_extract =
                    prop_values_.update_surface && idx > 0 &&
                    (idx == 3 || idx % effective_interval == 0);
            if (should_request_extract) {
                constexpr float kExtractWeightThreshold = 3.0f;
                const int extract_budget = GetLiveExtractBudget(
                        prop_values_.estimated_points.load(), hash_size);
                RequestAsyncExtract(kExtractWeightThreshold, extract_budget);
            }

            const uint64_t surface_version = surface_.version.load();
            const bool surface_updated =
                    surface_version > last_surface_version_gui_;
            const bool post_gui_this_frame =
                    (idx % gui_interval == 0) || surface_updated || idx <= 3;

            std::shared_ptr<geometry::Image> post_color;
            std::shared_ptr<geometry::Image> post_depth_colored;
            std::shared_ptr<geometry::Image> post_raycast_color;
            std::shared_ptr<geometry::Image> post_raycast_depth_colored;
            std::shared_ptr<geometry::LineSet> post_frustum;
            std::shared_ptr<geometry::LineSet> post_traj;
            std::string post_info;
            std::string post_fps;

            if (post_gui_this_frame) {
                std::stringstream info, fps;
                info.setf(std::ios::fixed, std::ios::floatfield);
                info.precision(4);
                info << fmt::format("Frame {}\n\n", idx);
                info << "Transformation:\n";
                info << T_eigen.format(CleanFmt) << "\n\n";
                info << fmt::format("Active voxel blocks: {}/{}\n",
                                    model_->GetHashMap().Size(),
                                    model_->GetHashMap().GetCapacity());
                if (hash_near_full) {
                    info << "Hash map nearly full: integration paused.\n";
                }
                if (integration_idle) {
                    info << "Integration idle (camera stationary).\n";
                }
                if (relocalizing_) {
                    info << fmt::format(
                            "Relocalizing: move slowly back to the scanned "
                            "area ({}/{} strong frames).\n",
                            relocalize_strong_streak_,
                            kRelocalizeStrongFrames);
                } else if (consecutive_tracking_failures_ > 5) {
                    info << fmt::format(
                            "Tracking lost: {} frames. Move slowly back to the "
                            "scanned area.\n",
                            consecutive_tracking_failures_);
                }
                {
                    std::lock_guard<std::mutex> locker(surface_.lock);
                    int64_t len =
                            surface_.pcd.HasPointPositions()
                                    ? surface_.pcd.GetPointPositions()
                                              .GetLength()
                                    : 0;
                    info << fmt::format("Live surface points: {}/{}\n", len,
                                        prop_values_.estimated_points);
                }
                {
                    const std::string surface_summary =
                            plane_atlas_.FormatSummary(6);
                    if (!surface_summary.empty()) {
                        info << surface_summary;
                    }
                    const int frozen_regions =
                            region_registry_.FrozenCount();
                    if (frozen_regions > 0) {
                        info << fmt::format("Frozen regions: {}\n",
                                            frozen_regions);
                    }
                }
                info << "\n";
                post_fps = fmt::format(
                        "FPS: {:.3f}\n",
                        1000.0 / (time_interval / fps_interval_len));
                info << post_fps;
                post_info = info.str();

                post_frustum = geometry::LineSet::CreateCameraVisualization(
                        input_frame.GetWidth(), input_frame.GetHeight(),
                        K_eigen, T_eigen.inverse(), 0.2);
                post_frustum->PaintUniformColor(kTangoOrange);

                if (traj->points_.size() > 1) {
                    post_traj = std::make_shared<geometry::LineSet>(*traj);
                }

                post_color = std::make_shared<geometry::Image>(
                        input_frame.GetDataAsImage("color")
                                .To(core::Device("CPU:0"))
                                .ToLegacy());
                post_depth_colored = std::make_shared<geometry::Image>(
                        input_frame.GetDataAsImage("depth")
                                .To(core::Device("CPU:0"))
                                .ColorizeDepth(
                                        static_cast<float>(depth_scale), 0.3,
                                        static_cast<float>(
                                                prop_values_.depth_max.load()))
                                .ToLegacy());
                if (prop_values_.raycast_color.load()) {
                    post_raycast_color = std::make_shared<geometry::Image>(
                            raycast_frame.GetDataAsImage("color")
                                    .To(core::Device("CPU:0"))
                                    .To(core::Dtype::UInt8, false, 255.0f)
                                    .ToLegacy());
                }
                post_raycast_depth_colored = std::make_shared<geometry::Image>(
                        raycast_frame.GetDataAsImage("depth")
                                .To(core::Device("CPU:0"))
                                .ColorizeDepth(
                                        static_cast<float>(depth_scale), 0.3,
                                        static_cast<float>(
                                                prop_values_.depth_max.load()))
                                .ToLegacy());
            }

            frame_timer.Stop();
            const double frame_ms = frame_timer.GetDurationInMillisecond();
            if (!exit_on_empty_frame_ && frame_ms > 40.0) {
                ++consecutive_slow_frames_;
                if (consecutive_slow_frames_ >= 3) {
                    utility::LogWarning(
                            "Frame {} processing took {:.1f} ms (>40 ms). "
                            "Skipping next capture to maintain throughput.",
                            idx, frame_ms);
                    capture_skip_count_ = 1;
                    consecutive_slow_frames_ = 0;
                }
            } else {
                consecutive_slow_frames_ = 0;
            }

            if (post_gui_this_frame) {
            PostGuiTask([this, post_color, post_depth_colored,
                         post_raycast_color, post_raycast_depth_colored,
                         post_traj, post_frustum, post_info, post_fps,
                         surface_version]() {
                        try {
                        last_surface_version_gui_ = surface_version;
                        this->fixed_props_->SetEnabled(false);

                        this->raycast_color_image_->SetVisible(
                                this->prop_values_.raycast_color.load());

                        this->SetInfo(post_info);
                        this->SetFPS(post_fps);
                        if (post_color) {
                            this->input_color_image_->UpdateImage(post_color);
                        }
                        if (post_depth_colored) {
                            this->input_depth_image_->UpdateImage(
                                    post_depth_colored);
                        }
                        if (prop_values_.raycast_color.load() &&
                            post_raycast_color) {
                            this->raycast_color_image_->UpdateImage(
                                    post_raycast_color);
                        }
                        if (post_raycast_depth_colored) {
                            this->raycast_depth_image_->UpdateImage(
                                    post_raycast_depth_colored);
                        }

                        this->widget3d_->GetScene()->RemoveGeometry("frustum");
                        auto mat = rendering::MaterialRecord();
                        mat.shader = "unlitLine";
                        mat.line_width = 5.0f;
                        if (post_frustum) {
                            this->widget3d_->GetScene()->AddGeometry(
                                    "frustum", post_frustum.get(), mat);
                        }

                        if (post_traj && post_traj->points_.size() > 1) {
                            if (!trajectory_geometry_added_) {
                                this->widget3d_->GetScene()->AddGeometry(
                                        "trajectory", post_traj.get(), mat);
                                trajectory_geometry_added_ = true;
                            } else {
                                this->widget3d_->GetScene()->RemoveGeometry(
                                        "trajectory");
                                this->widget3d_->GetScene()->AddGeometry(
                                        "trajectory", post_traj.get(), mat);
                            }
                        }

                        t::geometry::PointCloud surface_pcd;
                        {
                            std::lock_guard<std::mutex> locker(surface_.lock);
                            surface_pcd = surface_.pcd;
                        }
                        if (surface_pcd.HasPointPositions()) {
                            UpdateSurfaceGeometryOnScene(surface_pcd);
                        }
                        } catch (const std::exception& e) {
                            utility::LogWarning(
                                    "GUI update failed: {}", e.what());
                        } catch (...) {
                            utility::LogWarning(
                                    "GUI update failed with unknown exception.");
                        }
                    });
                WaitForPendingGui();
            }

            // Note that the user might have closed the window, in which case we
            // want to maintain a value of true.
            idx++;
            } catch (const std::exception& e) {
                utility::LogWarning("Frame {} processing failed: {}", idx,
                                    e.what());
                if (exit_on_empty_frame_) {
                    is_done_ = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            } catch (...) {
                utility::LogWarning("Frame {} processing failed with unknown "
                                    "exception.",
                                    idx);
                if (exit_on_empty_frame_) {
                    is_done_ = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    }
};

}  // namespace online_slam
}  // namespace examples
}  // namespace open3d
