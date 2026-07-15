// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "open3d/Open3D.h"

namespace open3d {
namespace examples {
namespace realtime_slam {

using namespace open3d::visualization;

// Filament upload budget for live preview.
static constexpr int kMaxRenderPoints = 100000;

class RealTimeSLAMWindow : public gui::Window {
    using Super = gui::Window;

public:
    struct DisplayState {
        std::atomic<bool> capture_enabled{true};
        std::atomic<bool> regions_enabled{false};
        std::atomic<bool> show_region_mesh{true};
        std::atomic<bool> show_region_pcd{true};

        void SetPointCloud(std::shared_ptr<geometry::PointCloud> pcd) {
            std::lock_guard<std::mutex> lock(mutex_);
            display_pcd_ = std::move(pcd);
            has_pcd_update_ = true;
        }

        bool TakePointCloud(std::shared_ptr<geometry::PointCloud>& out) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!has_pcd_update_) {
                return false;
            }
            out = display_pcd_;
            has_pcd_update_ = false;
            return out != nullptr;
        }

        struct RegionPair {
            int id = -1;
            std::shared_ptr<geometry::TriangleMesh> mesh;
            std::shared_ptr<geometry::PointCloud> pcd;
        };

        void PushRegionPair(RegionPair pair) {
            std::lock_guard<std::mutex> lock(mutex_);
            pending_regions_.push_back(std::move(pair));
        }

        bool TakeRegionPair(RegionPair& out) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (pending_regions_.empty()) {
                return false;
            }
            out = std::move(pending_regions_.front());
            pending_regions_.erase(pending_regions_.begin());
            return true;
        }

        void SetLostCameraMarker(std::shared_ptr<geometry::LineSet> marker,
                                 bool visible) {
            std::lock_guard<std::mutex> lock(mutex_);
            lost_camera_marker_ = std::move(marker);
            lost_camera_visible_ = visible;
            has_lost_camera_update_ = true;
        }

        bool TakeLostCameraMarker(std::shared_ptr<geometry::LineSet>& marker,
                                  bool& visible) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!has_lost_camera_update_) {
                return false;
            }
            marker = lost_camera_marker_;
            visible = lost_camera_visible_;
            has_lost_camera_update_ = false;
            return true;
        }

        void SetPoseDiffText(const std::string& text, bool visible) {
            std::lock_guard<std::mutex> lock(mutex_);
            pose_diff_text_ = text;
            pose_diff_visible_ = visible;
            has_pose_diff_update_ = true;
        }

        bool TakePoseDiffText(std::string& text, bool& visible) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!has_pose_diff_update_) {
                return false;
            }
            text = pose_diff_text_;
            visible = pose_diff_visible_;
            has_pose_diff_update_ = false;
            return true;
        }

        void SetStatus(const std::string& status) {
            std::lock_guard<std::mutex> lock(mutex_);
            status_ = status;
        }

        std::string Status() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return status_;
        }

        void SetRelocDetail(const std::string& detail) {
            std::lock_guard<std::mutex> lock(mutex_);
            reloc_detail_ = detail;
            has_reloc_detail_update_ = true;
        }

        bool TakeRelocDetail(std::string& detail) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!has_reloc_detail_update_) {
                return false;
            }
            detail = reloc_detail_;
            has_reloc_detail_update_ = false;
            return true;
        }

        void SetKeyframeMarkers(std::vector<Eigen::Vector3d> markers) {
            std::lock_guard<std::mutex> lock(mutex_);
            keyframe_markers_ = std::move(markers);
            has_keyframe_marker_update_ = true;
        }

        bool TakeKeyframeMarkers(std::vector<Eigen::Vector3d>& markers) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!has_keyframe_marker_update_) {
                return false;
            }
            markers = std::move(keyframe_markers_);
            has_keyframe_marker_update_ = false;
            return true;
        }

        void SetShowKeyframeMarkers(bool show) {
            show_keyframe_markers_.store(show);
        }

        bool ShowKeyframeMarkers() const {
            return show_keyframe_markers_.load();
        }

        void SetRegionCount(int count) { region_count_.store(count); }
        int RegionCount() const { return region_count_.load(); }

        std::atomic<bool> request_stop{false};
        std::atomic<bool> slam_finished{false};
        std::atomic<bool> reloc_self_test_finished{false};

    private:
        mutable std::mutex mutex_;
        std::shared_ptr<geometry::PointCloud> display_pcd_;
        bool has_pcd_update_ = false;
        std::vector<RegionPair> pending_regions_;
        std::shared_ptr<geometry::LineSet> lost_camera_marker_;
        bool lost_camera_visible_ = false;
        bool has_lost_camera_update_ = false;
        std::string pose_diff_text_;
        bool pose_diff_visible_ = false;
        bool has_pose_diff_update_ = false;
        std::string status_;
        std::string reloc_detail_;
        bool has_reloc_detail_update_ = false;
        std::vector<Eigen::Vector3d> keyframe_markers_;
        bool has_keyframe_marker_update_ = false;
        std::atomic<bool> show_keyframe_markers_{false};
        std::atomic<int> region_count_{0};
    };

    RealTimeSLAMWindow(DisplayState& state, gui::FontId monospace)
        : gui::Window("RealTimeSLAMRealSense", 1920, 1080), state_(state) {
        auto& theme = GetTheme();
        const int em = theme.font_size;
        const int spacing = int(std::round(0.25f * float(em)));
        const int vspacing = int(std::round(0.5f * float(em)));
        const gui::Margins margins(int(std::round(0.5f * float(em))));

        panel_ = std::make_shared<gui::Vert>(spacing, margins);
        widget3d_ = std::make_shared<gui::SceneWidget>();
        AddChild(panel_);
        AddChild(widget3d_);

        panel_->AddChild(std::make_shared<gui::Label>("Display controls"));

        capture_toggle_ = std::make_shared<gui::ToggleSwitch>(
                "Cloud capture ON/OFF");
        capture_toggle_->SetOn(true);
        capture_toggle_->SetOnClicked([this](bool is_on) {
            state_.capture_enabled.store(is_on);
        });
        panel_->AddChild(capture_toggle_);
        panel_->AddFixed(vspacing);

        if (state_.regions_enabled.load()) {
            region_mesh_toggle_ =
                    std::make_shared<gui::ToggleSwitch>("Polygon ON/OFF");
            region_mesh_toggle_->SetOn(true);
            region_mesh_toggle_->SetOnClicked([this](bool is_on) {
                state_.show_region_mesh.store(is_on);
                gui::Application::GetInstance().PostToMainThread(
                        this, [this]() { ApplyRegionVisibility(); });
            });
            panel_->AddChild(region_mesh_toggle_);
            panel_->AddFixed(vspacing);

            region_pcd_toggle_ =
                    std::make_shared<gui::ToggleSwitch>("Point cloud ON/OFF");
            region_pcd_toggle_->SetOn(true);
            region_pcd_toggle_->SetOnClicked([this](bool is_on) {
                state_.show_region_pcd.store(is_on);
                gui::Application::GetInstance().PostToMainThread(
                        this, [this]() { ApplyRegionVisibility(); });
            });
            panel_->AddChild(region_pcd_toggle_);
            panel_->AddFixed(vspacing);
        } else {
            auto hint = std::make_shared<gui::Label>(
                    "Polygon freeze: use --regions");
            panel_->AddChild(hint);
            panel_->AddFixed(vspacing);
        }

        status_label_ = std::make_shared<gui::Label>("");
        status_label_->SetFontId(monospace);
        panel_->AddChild(status_label_);
        panel_->AddFixed(vspacing);

        pose_diff_title_ =
                std::make_shared<gui::Label>("Relocalization guidance");
        panel_->AddChild(pose_diff_title_);
        pose_diff_label_ = std::make_shared<gui::Label>("");
        pose_diff_label_->SetFontId(monospace);
        panel_->AddChild(pose_diff_label_);
        pose_diff_title_->SetVisible(false);
        pose_diff_label_->SetVisible(false);
        panel_->AddFixed(vspacing);

        reloc_detail_label_ = std::make_shared<gui::Label>("");
        reloc_detail_label_->SetFontId(monospace);
        panel_->AddChild(reloc_detail_label_);
        panel_->AddFixed(vspacing);

        keyframe_marker_toggle_ = std::make_shared<gui::ToggleSwitch>(
                "Keyframe markers");
        keyframe_marker_toggle_->SetOn(false);
        keyframe_marker_toggle_->SetOnClicked([this](bool is_on) {
            state_.SetShowKeyframeMarkers(is_on);
            gui::Application::GetInstance().PostToMainThread(
                    this, [this]() { ApplyKeyframeMarkerVisibility(); });
        });
        panel_->AddChild(keyframe_marker_toggle_);
        panel_->AddStretch();

        widget3d_->SetScene(
                std::make_shared<rendering::Open3DScene>(GetRenderer()));

        SetOnClose([this]() {
            state_.request_stop.store(true);
            return true;
        });

        SetOnTickEvent([this]() { return OnTick(); });
    }

    void Layout(const gui::LayoutContext& context) override {
        const int em = context.theme.font_size;
        const int panel_width = 18 * em;
        const auto content_rect = GetContentRect();
        panel_->SetFrame(gui::Rect(content_rect.x, content_rect.y, panel_width,
                                   content_rect.height));
        const int x = panel_->GetFrame().GetRight();
        widget3d_->SetFrame(gui::Rect(x, content_rect.y,
                                      content_rect.GetRight() - x,
                                      content_rect.height));
        Super::Layout(context);
    }

private:
    struct RegionSceneEntry {
        int id = -1;
        std::string mesh_name;
        std::string pcd_name;
    };

    DisplayState& state_;
    std::shared_ptr<gui::Vert> panel_;
    std::shared_ptr<gui::SceneWidget> widget3d_;
    std::shared_ptr<gui::ToggleSwitch> capture_toggle_;
    std::shared_ptr<gui::ToggleSwitch> region_mesh_toggle_;
    std::shared_ptr<gui::ToggleSwitch> region_pcd_toggle_;
    std::shared_ptr<gui::Label> status_label_;
    std::shared_ptr<gui::Label> pose_diff_title_;
    std::shared_ptr<gui::Label> pose_diff_label_;
    std::shared_ptr<gui::Label> reloc_detail_label_;
    std::shared_ptr<gui::ToggleSwitch> keyframe_marker_toggle_;

    bool live_pcd_added_ = false;
    bool live_pcd_visible_ = true;
    t::geometry::PointCloud live_render_pcd_;
    geometry::LineSet lost_camera_marker_;
    bool lost_camera_marker_added_ = false;
    bool camera_view_initialized_ = false;
    std::string last_status_;
    std::vector<RegionSceneEntry> region_entries_;
    struct StoredRegionGeometry {
        t::geometry::TriangleMesh mesh;
        t::geometry::PointCloud pcd;
        bool has_mesh = false;
        bool has_pcd = false;
    };
    std::vector<StoredRegionGeometry> region_geometry_;
    struct StoredKeyframeMarker {
        t::geometry::TriangleMesh mesh;
    };
    std::vector<StoredKeyframeMarker> keyframe_marker_geometry_;
    std::vector<std::string> keyframe_marker_names_;

    rendering::Open3DScene* GetOpen3DScene() {
        return widget3d_->GetScene().get();
    }

    void ApplyLivePointCloudVisibility() {
        if (!live_pcd_added_) {
            return;
        }
        auto* scene = GetOpen3DScene();
        if (scene->HasGeometry("live_points")) {
            scene->ShowGeometry("live_points", live_pcd_visible_);
        }
    }

    void ApplyRegionVisibility() {
        auto* scene = GetOpen3DScene();
        const bool show_mesh = state_.show_region_mesh.load();
        const bool show_pcd = state_.show_region_pcd.load();
        live_pcd_visible_ = show_pcd;
        for (const auto& entry : region_entries_) {
            if (scene->HasGeometry(entry.mesh_name)) {
                scene->ShowGeometry(entry.mesh_name, show_mesh);
            }
            if (scene->HasGeometry(entry.pcd_name)) {
                scene->ShowGeometry(entry.pcd_name, show_pcd);
            }
        }
        ApplyLivePointCloudVisibility();
    }

    void ConfigureSlamCameraView(const geometry::AxisAlignedBoundingBox& bbox) {
        if (bbox.IsEmpty()) {
            return;
        }
        const Eigen::Vector3f center = bbox.GetCenter().cast<float>();
        widget3d_->SetupCamera(60.0f, bbox, center);
        const float max_dim =
                std::max(1.0f, 1.25f * static_cast<float>(bbox.GetMaxExtent()));
        const Eigen::Vector3f eye(center.x(), center.y() - 0.2f * max_dim,
                                  center.z() - max_dim);
        widget3d_->LookAt(center, eye, Eigen::Vector3f(0.f, -1.f, 0.f));
    }

    void UpdateCameraCenterOfRotation(
            const geometry::AxisAlignedBoundingBox& bbox) {
        if (bbox.IsEmpty()) {
            return;
        }
        if (!camera_view_initialized_) {
            ConfigureSlamCameraView(bbox);
            camera_view_initialized_ = true;
        } else {
            widget3d_->SetCenterOfRotation(bbox.GetCenter().cast<float>());
        }
    }

    t::geometry::PointCloud PrepareRenderPointCloud(
            const geometry::PointCloud& legacy_pcd) {
        if (legacy_pcd.IsEmpty()) {
            return {};
        }
        t::geometry::PointCloud render_pcd =
                t::geometry::PointCloud::FromLegacy(legacy_pcd);
        if (!render_pcd.HasPointColors()) {
            return render_pcd;
        }
        const int64_t count = render_pcd.GetPointPositions().GetLength();
        if (count <= kMaxRenderPoints) {
            return render_pcd;
        }
        return render_pcd.RandomDownSample(
                static_cast<double>(kMaxRenderPoints) /
                static_cast<double>(count));
    }

    void UpdateLivePointCloud(const geometry::PointCloud& legacy_pcd) {
        using namespace rendering;
        if (legacy_pcd.IsEmpty()) {
            return;
        }
        const t::geometry::PointCloud render_pcd =
                PrepareRenderPointCloud(legacy_pcd);
        if (!render_pcd.HasPointPositions()) {
            return;
        }

        auto* scene = GetOpen3DScene();
        MaterialRecord pcd_mat;
        pcd_mat.shader = "defaultUnlit";
        pcd_mat.sRGB_vertex_color = true;

        if (live_pcd_added_) {
            scene->RemoveGeometry("live_points");
        }
        live_render_pcd_ = render_pcd;
        scene->AddGeometry("live_points", &live_render_pcd_, pcd_mat);
        live_pcd_added_ = true;
        ApplyLivePointCloudVisibility();

        UpdateCameraCenterOfRotation(
                render_pcd.GetAxisAlignedBoundingBox().ToLegacy());
    }

    void UpsertRegionPair(const DisplayState::RegionPair& pair) {
        using namespace rendering;
        auto* scene = GetOpen3DScene();

        int existing_idx = -1;
        for (size_t i = 0; i < region_entries_.size(); ++i) {
            if (region_entries_[i].id == pair.id) {
                existing_idx = static_cast<int>(i);
                break;
            }
        }

        RegionSceneEntry entry;
        if (existing_idx >= 0) {
            entry = region_entries_[existing_idx];
        } else {
            entry.id = pair.id;
            entry.mesh_name = "region_" + std::to_string(pair.id) + "_mesh";
            entry.pcd_name = "region_" + std::to_string(pair.id) + "_pcd";
        }

        const bool had_mesh =
                existing_idx >= 0 && region_geometry_[existing_idx].has_mesh;

        StoredRegionGeometry stored;
        if (existing_idx >= 0) {
            stored = region_geometry_[existing_idx];
        }
        if (pair.mesh && !pair.mesh->IsEmpty()) {
            MaterialRecord mesh_mat;
            mesh_mat.shader = "defaultLit";
            mesh_mat.sRGB_vertex_color = true;
            stored.mesh = t::geometry::TriangleMesh::FromLegacy(*pair.mesh);
            stored.has_mesh = stored.mesh.HasVertexPositions();
            if (stored.has_mesh) {
                if (scene->HasGeometry(entry.mesh_name)) {
                    scene->RemoveGeometry(entry.mesh_name);
                }
                scene->AddGeometry(entry.mesh_name, &stored.mesh, mesh_mat);
            }
        }

        if (pair.pcd && !pair.pcd->IsEmpty()) {
            MaterialRecord pcd_mat;
            pcd_mat.shader = "defaultUnlit";
            pcd_mat.sRGB_vertex_color = true;
            stored.pcd = t::geometry::PointCloud::FromLegacy(*pair.pcd);
            stored.has_pcd = stored.pcd.HasPointPositions();
            if (stored.has_pcd) {
                if (scene->HasGeometry(entry.pcd_name)) {
                    scene->RemoveGeometry(entry.pcd_name);
                }
                scene->AddGeometry(entry.pcd_name, &stored.pcd, pcd_mat);
            }
        }

        if (existing_idx >= 0) {
            region_geometry_[existing_idx] = std::move(stored);
            if (stored.has_mesh && !had_mesh) {
                utility::LogInfo("Updated region {} mesh (was empty).", pair.id);
            } else {
                utility::LogInfo(
                        "Updated region {} pair in scene (mesh: {}, pcd: {}).",
                        pair.id, pair.mesh ? pair.mesh->vertices_.size() : 0,
                        pair.pcd ? pair.pcd->points_.size() : 0);
            }
        } else {
            region_geometry_.push_back(std::move(stored));
            region_entries_.push_back(entry);
            utility::LogInfo(
                    "Added region {} pair to scene (mesh: {}, pcd: {}).",
                    pair.id, pair.mesh ? pair.mesh->vertices_.size() : 0,
                    pair.pcd ? pair.pcd->points_.size() : 0);
        }
        ApplyRegionVisibility();
    }

    void AddRegionPair(const DisplayState::RegionPair& pair) {
        UpsertRegionPair(pair);
    }

    void UpdateLostCameraMarker(std::shared_ptr<geometry::LineSet> marker,
                                bool visible) {
        using namespace rendering;
        auto* scene = GetOpen3DScene();
        if (lost_camera_marker_added_ && scene->HasGeometry("lost_camera")) {
            scene->RemoveGeometry("lost_camera");
            lost_camera_marker_added_ = false;
        }
        if (!visible || !marker || marker->IsEmpty()) {
            return;
        }

        lost_camera_marker_ = *marker;
        MaterialRecord mat;
        mat.shader = "unlitLine";
        mat.line_width = 6.0f;
        scene->AddGeometry("lost_camera", &lost_camera_marker_, mat);
        lost_camera_marker_added_ = true;
    }

    void ApplyKeyframeMarkerVisibility() {
        auto* scene = GetOpen3DScene();
        const bool show = state_.ShowKeyframeMarkers();
        for (const auto& name : keyframe_marker_names_) {
            if (scene->HasGeometry(name)) {
                scene->ShowGeometry(name, show);
            }
        }
    }

    void UpdateKeyframeMarkers(const std::vector<Eigen::Vector3d>& markers) {
        using namespace rendering;
        auto* scene = GetOpen3DScene();
        for (const auto& name : keyframe_marker_names_) {
            if (scene->HasGeometry(name)) {
                scene->RemoveGeometry(name);
            }
        }
        keyframe_marker_names_.clear();
        keyframe_marker_geometry_.clear();

        MaterialRecord mat;
        mat.shader = "defaultLit";
        mat.base_color = Eigen::Vector4f(0.2f, 0.7f, 1.0f, 1.0f);

        for (size_t i = 0; i < markers.size(); ++i) {
            auto sphere = geometry::TriangleMesh::CreateSphere(0.03);
            sphere->Translate(markers[i]);
            sphere->PaintUniformColor(Eigen::Vector3d(0.2, 0.7, 1.0));
            StoredKeyframeMarker stored;
            stored.mesh = t::geometry::TriangleMesh::FromLegacy(*sphere);
            keyframe_marker_geometry_.push_back(std::move(stored));
            const std::string name = "keyframe_" + std::to_string(i);
            keyframe_marker_names_.push_back(name);
            scene->AddGeometry(name, &keyframe_marker_geometry_.back().mesh,
                               mat);
        }
        ApplyKeyframeMarkerVisibility();
    }

    bool OnTick() {
        if (state_.reloc_self_test_finished.load()) {
            return false;
        }
        if (state_.request_stop.load() && state_.slam_finished.load()) {
            return false;
        }

        DisplayState::RegionPair region_pair;
        while (state_.TakeRegionPair(region_pair)) {
            AddRegionPair(region_pair);
        }

        std::shared_ptr<geometry::PointCloud> updated;
        if (state_.TakePointCloud(updated) && updated && !updated->IsEmpty()) {
            UpdateLivePointCloud(*updated);
        }

        std::shared_ptr<geometry::LineSet> lost_camera_marker;
        bool lost_camera_visible = false;
        if (state_.TakeLostCameraMarker(lost_camera_marker,
                                        lost_camera_visible)) {
            UpdateLostCameraMarker(lost_camera_marker, lost_camera_visible);
        }

        std::string pose_diff_text;
        bool pose_diff_visible = false;
        if (state_.TakePoseDiffText(pose_diff_text, pose_diff_visible)) {
            pose_diff_title_->SetVisible(pose_diff_visible);
            pose_diff_label_->SetVisible(pose_diff_visible);
            pose_diff_label_->SetText(pose_diff_text.c_str());
            SetNeedsLayout();
        }

        std::string reloc_detail;
        if (state_.TakeRelocDetail(reloc_detail)) {
            reloc_detail_label_->SetText(reloc_detail.c_str());
        }

        std::vector<Eigen::Vector3d> keyframe_markers;
        if (state_.TakeKeyframeMarkers(keyframe_markers)) {
            UpdateKeyframeMarkers(keyframe_markers);
        }

        const std::string status = state_.Status();
        if (status != last_status_) {
            status_label_->SetText(status.c_str());
            last_status_ = status;
        }

        if (state_.slam_finished.load()) {
            std::shared_ptr<geometry::PointCloud> final_update;
            if (state_.TakePointCloud(final_update) && final_update &&
                !final_update->IsEmpty()) {
                UpdateLivePointCloud(*final_update);
            }
        }

        return true;
    }
};

}  // namespace realtime_slam
}  // namespace examples
}  // namespace open3d
