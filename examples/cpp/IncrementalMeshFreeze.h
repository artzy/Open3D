// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------
//
// Incremental mesh freeze: stable surface clusters -> mesh + FreezeBlocks.
// Wraps ObjectMeshPipeline and persists objects/ artifacts.

#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "ObjectMeshPipeline.h"
#include "open3d/Open3D.h"
#include "open3d/t/pipelines/slam/Model.h"

namespace open3d {
namespace examples {

struct IncrementalFreezeConfig {
    bool enabled = true;
    bool auto_save = true;
    bool save_point_cloud_snapshot = true;
    std::string output_dir = "objects";
    object_mesh::SegmentationConfig segmentation;
};

struct FrozenMeshEntry {
    int id = -1;
    object_mesh::ObjectType type = object_mesh::ObjectType::kGeneric;
    geometry::TriangleMesh mesh_legacy;
    std::shared_ptr<geometry::PointCloud> snapshot_pcd;
    core::Tensor block_keys;
    Eigen::Vector4d plane = Eigen::Vector4d::Zero();
    int tile_i = 0;
    int tile_j = 0;
    double area_m2 = 0.0;
    // Growing plane surface fields; an entry re-emitted with a known id is
    // a grown replacement of the previous mesh.
    double cell_size = 0.0;
    int cell_count = 0;
    double closure = 0.0;
    std::vector<int> connected;
    double compare_rmse = 0.0;
    int patch_count = 0;
    std::string mesh_mode = "legacy_grid";
};

inline std::string FrozenSourcePointCloudFileName(int id) {
    return "source_" + std::to_string(id) + ".ply";
}

inline std::string FrozenMeshFileName(const FrozenMeshEntry& entry) {
    return (object_mesh::IsPlanarType(entry.type) ? "surface_" : "object_") +
           std::to_string(entry.id) + ".ply";
}

inline void ClampMeshColors(geometry::TriangleMesh& mesh) {
    for (auto& c : mesh.vertex_colors_) {
        c = c.cwiseMax(Eigen::Vector3d::Zero())
                    .cwiseMin(Eigen::Vector3d::Ones());
    }
}

inline void ClampPcdColors(geometry::PointCloud& pcd) {
    for (auto& c : pcd.colors_) {
        c = c.cwiseMax(Eigen::Vector3d::Zero())
                    .cwiseMin(Eigen::Vector3d::Ones());
    }
}

class IncrementalMeshFreeze {
public:
    explicit IncrementalMeshFreeze(const IncrementalFreezeConfig& config)
        : config_(config) {}

    void UpdateConfig(const IncrementalFreezeConfig& config) {
        config_ = config;
    }

    const IncrementalFreezeConfig& GetConfig() const { return config_; }

    /// Partitions the surface into regions (planar tiles + DBSCAN clusters),
    /// tracks stability in the RegionRegistry; freezes TSDF blocks and
    /// builds meshes.
    std::vector<FrozenMeshEntry> ProcessSurface(
            const t::geometry::PointCloud& surface_pcd,
            t::pipelines::slam::Model& model) {
        std::vector<FrozenMeshEntry> new_entries;
        if (!config_.enabled || !config_.segmentation.auto_freeze ||
            surface_pcd.IsEmpty()) {
            return new_entries;
        }

        object_mesh::SegmentationConfig seg_cfg = config_.segmentation;
        auto candidates = object_mesh::ProcessExtractedSurface(
                surface_pcd, model, registry_, atlas_, seg_cfg,
                next_object_id_);

        const geometry::PointCloud surface_legacy = surface_pcd.ToLegacy();
        (void)surface_legacy;

        for (auto& candidate : candidates) {
            if (!candidate.mesh.HasVertexPositions()) {
                continue;
            }
            FrozenMeshEntry entry;
            entry.id = candidate.id;
            entry.type = candidate.type;
            entry.mesh_legacy = candidate.mesh.ToLegacy();
            ClampMeshColors(entry.mesh_legacy);
            entry.block_keys = candidate.block_keys;
            entry.plane = candidate.plane;
            entry.tile_i = candidate.tile_i;
            entry.tile_j = candidate.tile_j;
            entry.area_m2 = candidate.area_m2;
            entry.cell_size = candidate.cell_size;
            entry.cell_count = candidate.cell_count;
            entry.closure = candidate.closure;
            entry.connected = candidate.connected;
            entry.compare_rmse = candidate.compare_rmse;
            entry.patch_count = candidate.patch_count;
            entry.mesh_mode = candidate.mesh_mode;

            if (config_.save_point_cloud_snapshot) {
                if (candidate.source_snapshot) {
                    entry.snapshot_pcd = candidate.source_snapshot;
                } else if (candidate.source_points) {
                    entry.snapshot_pcd = candidate.source_points;
                }
            }

            // Growing surfaces re-emit the same id: replace the previous
            // entry so the manifest and mesh files stay deduplicated.
            bool replaced = false;
            for (auto& existing : all_frozen_) {
                if (existing.id == entry.id) {
                    existing = entry;
                    replaced = true;
                    break;
                }
            }
            if (!replaced) {
                all_frozen_.push_back(entry);
            }
            new_entries.push_back(entry);

            utility::LogInfo(
                    "{} region {} ({}) — {} vertices, closure {:.0f}%.",
                    replaced ? "Grew" : "Frozen", entry.id,
                    object_mesh::ObjectTypeName(entry.type),
                    entry.mesh_legacy.vertices_.size(),
                    entry.closure * 100.0);
        }

        if (config_.auto_save && !new_entries.empty()) {
            SaveNewEntries(new_entries);
            WriteManifest();
        }

        return new_entries;
    }

    const std::vector<FrozenMeshEntry>& GetAllFrozen() const {
        return all_frozen_;
    }

    int FrozenCount() const {
        return static_cast<int>(all_frozen_.size());
    }

    void WriteManifest() const {
        if (all_frozen_.empty()) {
            return;
        }
        utility::filesystem::MakeDirectoryHierarchy(config_.output_dir);
        const std::string json_path =
                config_.output_dir + "/frozen_blocks.json";
        std::ofstream blocks_json(json_path);
        if (!blocks_json) {
            utility::LogWarning("Could not write {}", json_path);
            return;
        }

        blocks_json << "{\n  \"objects\": [\n";
        for (size_t i = 0; i < all_frozen_.size(); ++i) {
            const auto& obj = all_frozen_[i];
            const std::string mesh_path = FrozenMeshFileName(obj);
            const std::string pcd_path =
                    FrozenSourcePointCloudFileName(obj.id);

            blocks_json << "    {\n";
            blocks_json << "      \"id\": " << obj.id << ",\n";
            blocks_json << "      \"type\": \""
                        << object_mesh::ObjectTypeName(obj.type) << "\",\n";
            blocks_json << "      \"state\": \"frozen\",\n";
            if (object_mesh::IsPlanarType(obj.type)) {
                blocks_json << fmt::format(
                        "      \"plane\": [{:.6f}, {:.6f}, {:.6f}, "
                        "{:.6f}],\n",
                        obj.plane(0), obj.plane(1), obj.plane(2),
                        obj.plane(3));
            }
            blocks_json << fmt::format("      \"area_m2\": {:.3f},\n",
                                       obj.area_m2);
            blocks_json << "      \"mesh\": \"" << mesh_path << "\"";
            if (obj.snapshot_pcd) {
                blocks_json << ",\n      \"source_point_cloud\": \""
                            << pcd_path << "\"";
                blocks_json << ",\n      \"source_point_count\": "
                            << obj.snapshot_pcd->points_.size();
                if (obj.compare_rmse > 0.0) {
                    blocks_json << fmt::format(
                            ",\n      \"compare_rmse\": {:.6f}",
                            obj.compare_rmse);
                }
            }
            if (object_mesh::IsPlanarType(obj.type)) {
                blocks_json << fmt::format(
                        ",\n      \"mesh_mode\": \"{}\",\n"
                        "      \"patch_count\": {}",
                        obj.mesh_mode, obj.patch_count);
            }
            if (obj.block_keys.NumElements() > 0) {
                blocks_json << ",\n      \"block_keys\": [";
                core::Tensor keys =
                        obj.block_keys.To(core::Device("CPU:0")).Contiguous();
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
            if (i + 1 < all_frozen_.size()) {
                blocks_json << ",";
            }
            blocks_json << "\n";
        }
        blocks_json << "  ],\n  \"surfaces\": [\n";
        const auto surfaces = atlas_.Snapshot();
        for (size_t i = 0; i < surfaces.size(); ++i) {
            const auto& s = surfaces[i];
            blocks_json << fmt::format(
                    "    {{\"id\": {}, \"type\": \"{}\", \"plane\": "
                    "[{:.6f}, {:.6f}, {:.6f}, {:.6f}], \"cell_size\": "
                    "{:.3f}, \"cell_count\": {}, \"patch_count\": {}, "
                    "\"mesh_mode\": \"{}\", \"closure\": {:.3f}, "
                    "\"connected\": [",
                    s.id, object_mesh::ObjectTypeName(s.type), s.plane(0),
                    s.plane(1), s.plane(2), s.plane(3), s.cell_size,
                    s.cell_count, s.patch_count, s.mesh_mode, s.closure);
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
    }

    void SaveAllArtifacts() const {
        if (all_frozen_.empty()) {
            return;
        }
        utility::filesystem::MakeDirectoryHierarchy(config_.output_dir);
        for (const auto& obj : all_frozen_) {
            SaveEntryFiles(obj);
        }
        WriteManifest();
        utility::LogInfo("Saved {} frozen object(s) under {}/.",
                         all_frozen_.size(), config_.output_dir);
    }

private:
    void SaveNewEntries(const std::vector<FrozenMeshEntry>& entries) const {
        utility::filesystem::MakeDirectoryHierarchy(config_.output_dir);
        for (const auto& entry : entries) {
            SaveEntryFiles(entry);
        }
    }

    void SaveEntryFiles(const FrozenMeshEntry& entry) const {
        const std::string mesh_path =
                config_.output_dir + "/" + FrozenMeshFileName(entry);
        try {
            io::WriteTriangleMesh(mesh_path, entry.mesh_legacy);
        } catch (const std::exception& e) {
            utility::LogWarning("Failed to save {}: {}", mesh_path, e.what());
        }

        if (entry.snapshot_pcd) {
            const std::string pcd_path =
                    config_.output_dir + "/" +
                    FrozenSourcePointCloudFileName(entry.id);
            try {
                geometry::PointCloud pcd = *entry.snapshot_pcd;
                ClampPcdColors(pcd);
                io::WritePointCloud(pcd_path, pcd);
            } catch (const std::exception& e) {
                utility::LogWarning("Failed to save {}: {}", pcd_path, e.what());
            }
        }
    }

    IncrementalFreezeConfig config_;
    object_mesh::RegionRegistry registry_;
    object_mesh::PlaneSurfaceAtlas atlas_;
    int next_object_id_ = 0;
    std::vector<FrozenMeshEntry> all_frozen_;
};

}  // namespace examples
}  // namespace open3d
