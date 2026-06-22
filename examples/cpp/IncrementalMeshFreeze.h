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
};

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
        : config_(config), tracker_(config.segmentation) {}

    void UpdateConfig(const IncrementalFreezeConfig& config) {
        config_ = config;
        tracker_.SetConfig(config.segmentation);
    }

    const IncrementalFreezeConfig& GetConfig() const { return config_; }

    /// Runs DBSCAN + stability tracking; freezes TSDF blocks and builds meshes.
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
                surface_pcd, model, tracker_, seg_cfg, next_object_id_);

        const geometry::PointCloud surface_legacy = surface_pcd.ToLegacy();

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

            if (config_.save_point_cloud_snapshot &&
                candidate.bounds.Volume() > 0.0) {
                std::shared_ptr<geometry::PointCloud> cropped =
                        surface_legacy.Crop(candidate.bounds);
                if (cropped && !cropped->IsEmpty()) {
                    ClampPcdColors(*cropped);
                    entry.snapshot_pcd = cropped;
                }
            }

            all_frozen_.push_back(entry);
            new_entries.push_back(entry);

            utility::LogInfo(
                    "Frozen object {} ({}) — {} vertices, {} blocks.",
                    entry.id, object_mesh::ObjectTypeName(entry.type),
                    entry.mesh_legacy.vertices_.size(),
                    entry.block_keys.GetLength());
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
            const std::string mesh_path =
                    "object_" + std::to_string(obj.id) + ".ply";
            const std::string pcd_path =
                    "pcd_" + std::to_string(obj.id) + ".ply";

            blocks_json << "    {\n";
            blocks_json << "      \"id\": " << obj.id << ",\n";
            blocks_json << "      \"type\": \""
                        << object_mesh::ObjectTypeName(obj.type) << "\",\n";
            blocks_json << "      \"mesh\": \"" << mesh_path << "\"";
            if (obj.snapshot_pcd) {
                blocks_json << ",\n      \"point_cloud\": \"" << pcd_path
                            << "\"";
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
        const std::string mesh_path = config_.output_dir + "/object_" +
                                      std::to_string(entry.id) + ".ply";
        try {
            io::WriteTriangleMesh(mesh_path, entry.mesh_legacy);
        } catch (const std::exception& e) {
            utility::LogWarning("Failed to save {}: {}", mesh_path, e.what());
        }

        if (entry.snapshot_pcd) {
            const std::string pcd_path = config_.output_dir + "/pcd_" +
                                         std::to_string(entry.id) + ".ply";
            try {
                io::WritePointCloud(pcd_path, *entry.snapshot_pcd);
            } catch (const std::exception& e) {
                utility::LogWarning("Failed to save {}: {}", pcd_path, e.what());
            }
        }
    }

    IncrementalFreezeConfig config_;
    object_mesh::ObjectFreezeTracker tracker_;
    int next_object_id_ = 0;
    std::vector<FrozenMeshEntry> all_frozen_;
};

}  // namespace examples
}  // namespace open3d
