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
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <sstream>
#include <thread>
#include <unordered_map>

#include "open3d/Open3D.h"
#include "ObjectMeshPipeline.h"

namespace open3d {
namespace examples {
namespace online_slam {
using namespace open3d::visualization;

// Filament upload budget for live preview (extract uses estimated_points).
static constexpr int kMaxRenderPoints = 500000;

// Tracking tiers: strict pose update vs weak integrate-only vs reject outlier jumps.
static constexpr double kPoseFitnessMin = 0.15;
static constexpr double kPoseTranslationMax = 0.12;
static constexpr double kWeakFitnessMin = 0.08;
static constexpr double kWeakTranslationMax = 0.30;
static constexpr double kOutlierTranslation = 0.50;

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
                {"odometry_iter_0", 6},
                {"odometry_iter_1", 3},
                {"odometry_iter_2", 1},
                {"gui_update_interval", 1},
                {"raycast_color", 1},
                {"auto_freeze", 1},
                {"stability_frames", 5},
                {"dbscan_eps_multiplier", 2.0},
                {"min_cluster_points", 5000}};
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
                StartSlam();
            } else {
                PauseSlam();
            }
        });
        panel_->AddChild(b);
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
                    auto pcd = model_->ExtractPointCloudExcludingFrozen(
                            3.0, save_budget);
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
                        const std::string mesh_path =
                                "object_" + std::to_string(obj.id) + ".ply";
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
                    blocks_json << "  ]\n}\n";
                    utility::LogInfo(
                            "Saved {} frozen object meshes to objects/.",
                            frozen_objects_.size());
                }

                utility::LogInfo("Writing trajectory to trajectory.log...");
                io::WritePinholeCameraTrajectory("trajectory.log",
                                                 *trajectory_);
            }
            return true;  // false would cancel the close
        });
        update_thread_ = std::thread([this]() { this->UpdateMain(); });
        extract_thread_ = std::thread([this]() { this->ExtractWorker(); });
        segmentation_thread_ =
                std::thread([this]() { this->SegmentationWorker(); });
        gui::Application::GetInstance().PostToMainThread(this, [this]() {
            StartSlam();
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
        std::atomic<bool> raycast_color;
        std::atomic<bool> update_surface;
        std::atomic<bool> auto_freeze;
        std::atomic<int> stability_frames;
        std::atomic<double> dbscan_eps_multiplier;
        std::atomic<int> min_cluster_points;
    } prop_values_;

    struct {
        std::mutex lock;
        t::geometry::PointCloud pcd;
        std::atomic<uint64_t> version{0};
    } surface_;
    bool points_geometry_added_ = false;
    bool trajectory_geometry_added_ = false;
    int64_t last_render_point_count_ = 0;
    uint64_t last_surface_version_gui_ = 0;
    bool camera_fitted_ = false;
    Eigen::Vector3f last_camera_center_ = Eigen::Vector3f::Zero();
    int consecutive_tracking_failures_ = 0;
    int consecutive_slow_frames_ = 0;
    int capture_skip_count_ = 0;

    std::shared_ptr<gui::ToggleSwitch> resume_toggle_;

    std::shared_ptr<t::pipelines::slam::Model> model_;
    std::shared_ptr<camera::PinholeCameraTrajectory> trajectory_;
    std::thread update_thread_;
    std::thread extract_thread_;
    std::mutex model_mutex_;
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
    };
    std::vector<FrozenObjectEntry> frozen_objects_;
    std::mutex frozen_mutex_;
    std::atomic<uint64_t> frozen_version_{0};
    uint64_t last_frozen_version_gui_ = 0;
    object_mesh::ObjectFreezeTracker freeze_tracker_{
            object_mesh::SegmentationConfig{}};
    int next_object_id_ = 0;

    std::thread segmentation_thread_;
    std::mutex segmentation_mutex_;
    std::condition_variable segmentation_cv_;
    std::atomic<bool> segmentation_requested_{false};
    t::geometry::PointCloud pending_segmentation_pcd_;

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
        return config;
    }

    void AddFrozenMeshesToScene(
            const std::vector<FrozenObjectEntry>& new_objects) {
        using namespace rendering;
        auto* scene = widget3d_->GetScene()->GetScene();
        rendering::MaterialRecord mesh_mat;
        mesh_mat.shader = "defaultLit";
        mesh_mat.sRGB_vertex_color = true;
        for (const auto& obj : new_objects) {
            if (!obj.mesh.HasVertexPositions()) {
                continue;
            }
            const std::string name =
                    "object_" + std::to_string(obj.id);
            if (scene->HasGeometry(name)) {
                scene->RemoveGeometry(name);
            }
            scene->AddGeometry(name, obj.mesh, mesh_mat);
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
                freeze_tracker_.SetConfig(config);
                std::vector<FrozenObjectEntry> new_objects;
                {
                    std::lock_guard<std::mutex> model_lock(model_mutex_);
                    auto candidates = object_mesh::ProcessExtractedSurface(
                            segmentation_pcd, *model_, freeze_tracker_, config,
                            next_object_id_);
                    for (auto& candidate : candidates) {
                        if (!candidate.mesh.HasVertexPositions()) {
                            continue;
                        }
                        FrozenObjectEntry entry;
                        entry.id = candidate.id;
                        entry.type = candidate.type;
                        entry.mesh = std::move(candidate.mesh);
                        entry.block_keys = candidate.block_keys;
                        new_objects.push_back(std::move(entry));
                    }
                }

                if (!new_objects.empty()) {
                    {
                        std::lock_guard<std::mutex> lock(frozen_mutex_);
                        for (auto& entry : new_objects) {
                            frozen_objects_.push_back(entry);
                        }
                    }
                    frozen_version_.fetch_add(1);
                    utility::LogInfo(
                            "Frozen {} object(s). Total frozen: {}.",
                            new_objects.size(), frozen_objects_.size());
                    gui::Application::GetInstance().PostToMainThread(
                            this, [this, new_objects]() {
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
            const t::geometry::PointCloud& render_pcd) {
        using namespace rendering;
        if (!render_pcd.HasPointPositions()) {
            return;
        }
        const int64_t render_count =
                render_pcd.GetPointPositions().GetLength();
        auto* scene = widget3d_->GetScene()->GetScene();
        auto pcd_mat = rendering::MaterialRecord();
        pcd_mat.shader = "defaultUnlit";
        pcd_mat.sRGB_vertex_color = true;

        const bool needs_rebuild =
                !points_geometry_added_ ||
                last_render_point_count_ == 0 ||
                render_count > last_render_point_count_ * 12 / 10 ||
                render_count < last_render_point_count_ * 8 / 10;

        if (needs_rebuild) {
            if (points_geometry_added_) {
                scene->RemoveGeometry("points");
            }
            scene->AddGeometry("points", render_pcd, pcd_mat);
            points_geometry_added_ = true;
        } else {
            scene->UpdateGeometry("points", render_pcd,
                                  Scene::kUpdatePointsFlag |
                                          Scene::kUpdateColorsFlag);
        }
        last_render_point_count_ = render_count;

        auto tbbox = render_pcd.GetAxisAlignedBoundingBox();
        geometry::AxisAlignedBoundingBox bbox = tbbox.ToLegacy();
        if (bbox.Volume() > 0) {
            Eigen::Vector3f center = bbox.GetCenter().cast<float>();
            const float center_shift =
                    camera_fitted_ ? (center - last_camera_center_).norm()
                                   : 0.0f;
            if (!camera_fitted_ || center_shift > 0.3f) {
                widget3d_->SetupCamera(60.0f, bbox, center);
                last_camera_center_ = center;
                camera_fitted_ = true;
            }
        }
    }

    void InitSlamModelOnMainThread() {
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
        InitSlamModelOnMainThread();
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
        while (!is_started_ && !is_done_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (is_done_) {
            return;
        }

        // Only set at initialization
        float depth_scale = prop_values_.depth_scale;
        core::Tensor T_frame_to_model = core::Tensor::Eye(
                4, core::Dtype::Float64, core::Device("CPU:0"));

        t::geometry::RGBDImage ref_rgbd_input =
                CaptureInputFrame(0).To(device_);
        if (ref_rgbd_input.IsEmpty()) {
            if (HandleEmptyFrame(0, "startup")) {
                return;
            }
            ref_rgbd_input = CaptureInputFrame(0).To(device_);
            if (ref_rgbd_input.IsEmpty() &&
                HandleEmptyFrame(0, "startup_retry")) {
                return;
            }
        }
        if (ref_rgbd_input.IsEmpty()) {
            return;
        }

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
                ref_rgbd_input.color_.ToLegacy());

        depth_colored = std::make_shared<geometry::Image>(
                ref_rgbd_input.depth_
                        .ColorizeDepth(depth_scale, 0.3, prop_values_.depth_max)
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

        // Add placeholder in case color raycast is disabled in the beginning.
        raycast_frame.SetData(
                "color",
                core::Tensor::Zeros({ref_rgbd_input.depth_.GetRows(),
                                     ref_rgbd_input.depth_.GetCols(), 3},
                                    core::Dtype::UInt8, core::Device("CPU:0")));

        camera::PinholeCameraParameters traj_param;
        traj_param.intrinsic_ = camera::PinholeCameraIntrinsic(
                ref_rgbd_input.depth_.GetRows(),
                ref_rgbd_input.depth_.GetCols(),
                core::eigen_converter::TensorToEigenMatrixXd(intrinsic_));

        // Render once to refresh
        gui::Application::GetInstance().PostToMainThread(
                this, [this, color, depth_colored, raycast_color,
                       raycast_depth_colored]() {
                    this->input_color_image_->UpdateImage(color);
                    this->input_depth_image_->UpdateImage(depth_colored);
                    this->raycast_color_image_->UpdateImage(color);
                    this->raycast_depth_image_->UpdateImage(depth_colored);
                    this->SetNeedsLayout();  // size of image changed

                    geometry::AxisAlignedBoundingBox bbox(
                            Eigen::Vector3d(-5, -5, -5),
                            Eigen::Vector3d(5, 5, 5));
                    Eigen::Vector3f center = bbox.GetCenter().cast<float>();
                    this->widget3d_->SetupCamera(60, bbox, center);
                    this->widget3d_->LookAt(center,
                                            center - Eigen::Vector3f{0, 1, 3},
                                            {0.0f, -1.0f, 0.0f});
                });

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
            bool pose_updated = false;
            TrackingTier tracking_tier = TrackingTier::kInit;
            double track_fitness = 1.0;
            double track_translation = 0.0;
            if (idx > 0 && !hash_near_full) {
                auto classify_tracking = [](double fitness,
                                            double translation) -> TrackingTier {
                    if (translation >= kOutlierTranslation) {
                        return TrackingTier::kOutlier;
                    }
                    if (fitness >= kPoseFitnessMin &&
                        translation < kPoseTranslationMax) {
                        return TrackingTier::kStrong;
                    }
                    if (fitness >= kWeakFitnessMin &&
                        translation < kWeakTranslationMax) {
                        return TrackingTier::kWeak;
                    }
                    return TrackingTier::kFail;
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
                    core::Tensor translation =
                            result.transformation_.Slice(0, 0, 3).Slice(1, 3, 4);
                    track_translation = std::sqrt(
                            (translation * translation)
                                    .Sum({0, 1})
                                    .Item<double>());
                    track_fitness = result.fitness_;
                    tracking_tier = classify_tracking(track_fitness,
                                                      track_translation);

                    if (tracking_tier == TrackingTier::kFail &&
                        track_fitness < kWeakFitnessMin) {
                        result = run_tracking(static_cast<float>(
                                prop_values_.depth_diff * 2.0));
                        translation = result.transformation_.Slice(0, 0, 3)
                                              .Slice(1, 3, 4);
                        track_translation = std::sqrt(
                                (translation * translation)
                                        .Sum({0, 1})
                                        .Item<double>());
                        track_fitness = result.fitness_;
                        tracking_tier = classify_tracking(track_fitness,
                                                          track_translation);
                    }

                    if (tracking_tier == TrackingTier::kStrong) {
                        T_frame_to_model =
                                T_frame_to_model.Matmul(result.transformation_);
                        pose_updated = true;
                        tracking_success = true;
                        consecutive_tracking_failures_ = 0;
                    } else if (tracking_tier == TrackingTier::kWeak) {
                        tracking_success = true;
                        consecutive_tracking_failures_ = 0;
                        utility::LogDebug(
                                "Weak tracking frame {}: fitness {:.3f}, "
                                "translation {:.3f}. Integrating with previous "
                                "pose.",
                                idx, track_fitness, track_translation);
                    } else {
                        tracking_success = false;
                        ++consecutive_tracking_failures_;
                        const char* tier_name =
                                tracking_tier == TrackingTier::kOutlier
                                        ? "outlier"
                                        : "failed";
                        utility::LogWarning(
                                "Tracking {} for frame {}, fitness: {:.3f}, "
                                "translation: {:.3f}. Skipping integration.",
                                tier_name, idx, track_fitness,
                                track_translation);
                    }
                } catch (const std::exception& e) {
                    tracking_success = false;
                    tracking_tier = TrackingTier::kFail;
                    ++consecutive_tracking_failures_;
                    utility::LogWarning(
                            "Tracking exception for frame {}: {}", idx,
                            e.what());
                }
            } else if (idx > 0 && hash_near_full) {
                tracking_success = false;
                tracking_tier = TrackingTier::kFail;
            }

            model_->UpdateFramePose(idx, T_frame_to_model);
            const bool integrated = tracking_success;
            {
                std::lock_guard<std::mutex> model_lock(model_mutex_);
                if (integrated && !hash_near_full) {
                    model_->Integrate(input_frame, depth_scale,
                                      prop_values_.depth_max,
                                      prop_values_.trunc_multiplier);
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
                if (consecutive_tracking_failures_ > 5) {
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
                    std::lock_guard<std::mutex> lock(frozen_mutex_);
                    if (!frozen_objects_.empty()) {
                        std::unordered_map<object_mesh::ObjectType, int>
                                type_counts;
                        for (const auto& obj : frozen_objects_) {
                            ++type_counts[obj.type];
                        }
                        info << fmt::format("Frozen objects: {} (",
                                            frozen_objects_.size());
                        bool first = true;
                        for (const auto& [type, count] : type_counts) {
                            if (!first) {
                                info << ", ";
                            }
                            info << fmt::format("{}×{}", count,
                                                object_mesh::ObjectTypeName(
                                                        type));
                            first = false;
                        }
                        info << ")\n";
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

                post_color = std::make_shared<geometry::Image>(
                        input_frame.GetDataAsImage("color").ToLegacy());
                post_depth_colored = std::make_shared<geometry::Image>(
                        input_frame.GetDataAsImage("depth")
                                .ColorizeDepth(depth_scale, 0.3,
                                               prop_values_.depth_max)
                                .ToLegacy());
                if (prop_values_.raycast_color) {
                    post_raycast_color = std::make_shared<geometry::Image>(
                            raycast_frame.GetDataAsImage("color")
                                    .To(core::Dtype::UInt8, false, 255.0f)
                                    .ToLegacy());
                }
                post_raycast_depth_colored = std::make_shared<geometry::Image>(
                        raycast_frame.GetDataAsImage("depth")
                                .ColorizeDepth(depth_scale, 0.3,
                                               prop_values_.depth_max)
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
            gui::Application::GetInstance().PostToMainThread(
                    this, [this, post_color, post_depth_colored,
                           post_raycast_color, post_raycast_depth_colored, traj,
                           post_frustum, post_info, post_fps, surface_version]() {
                        try {
                        last_surface_version_gui_ = surface_version;
                        this->fixed_props_->SetEnabled(false);

                        this->raycast_color_image_->SetVisible(
                                this->prop_values_.raycast_color);

                        this->SetInfo(post_info);
                        this->SetFPS(post_fps);
                        if (post_color) {
                            this->input_color_image_->UpdateImage(post_color);
                        }
                        if (post_depth_colored) {
                            this->input_depth_image_->UpdateImage(
                                    post_depth_colored);
                        }
                        if (prop_values_.raycast_color && post_raycast_color) {
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

                        if (traj->points_.size() > 1) {
                            if (!trajectory_geometry_added_) {
                                this->widget3d_->GetScene()->AddGeometry(
                                        "trajectory", traj.get(), mat);
                                trajectory_geometry_added_ = true;
                            } else {
                                this->widget3d_->GetScene()->RemoveGeometry(
                                        "trajectory");
                                this->widget3d_->GetScene()->AddGeometry(
                                        "trajectory", traj.get(), mat);
                            }
                        }

                        t::geometry::PointCloud surface_pcd;
                        {
                            std::lock_guard<std::mutex> locker(surface_.lock);
                            surface_pcd = surface_.pcd;
                        }
                        if (surface_pcd.HasPointPositions()) {
                            t::geometry::PointCloud render_pcd =
                                    PrepareRenderPointCloud(surface_pcd);
                            UpdateSurfaceGeometryOnScene(render_pcd);
                        }
                        } catch (const std::exception& e) {
                            utility::LogWarning(
                                    "GUI update failed: {}", e.what());
                        } catch (...) {
                            utility::LogWarning(
                                    "GUI update failed with unknown exception.");
                        }
                    });
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
