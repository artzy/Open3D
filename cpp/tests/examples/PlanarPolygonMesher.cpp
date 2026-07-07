// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include "PlanarPolygonMesher.h"
#include "tests/Tests.h"

namespace open3d {
namespace tests {

using open3d::examples::object_mesh::planar_polygon::AdaptiveMeshParams;
using open3d::examples::object_mesh::planar_polygon::AdaptiveMeshResult;
using open3d::examples::object_mesh::planar_polygon::AdaptiveSurfaceInput;
using open3d::examples::object_mesh::planar_polygon::BuildAdaptivePatchMesh;
using open3d::examples::object_mesh::planar_polygon::OccupiedCellView;

TEST(PlanarPolygonMesher, FlatRegionProducesLeaves) {
    AdaptiveSurfaceInput surface;
    surface.plane = Eigen::Vector4d(0.0, 0.0, 1.0, 0.0);
    surface.u = Eigen::Vector3d::UnitX();
    surface.v = Eigen::Vector3d::UnitY();
    surface.cell_size = 0.25;

    std::map<std::pair<int, int>, OccupiedCellView> cells;
    OccupiedCellView& cell = cells[{0, 0}];
    cell.points = 500;
    cell.color_sum = Eigen::Vector3d(0.5, 0.5, 0.5);
    cell.min_u = 0.0;
    cell.max_u = 0.2;
    cell.min_v = 0.0;
    cell.max_v = 0.2;

    geometry::PointCloud archive;
    for (int i = 0; i < 50; ++i) {
        for (int j = 0; j < 50; ++j) {
            archive.points_.emplace_back(0.004 * i, 0.004 * j, 0.0);
            archive.colors_.emplace_back(0.5, 0.5, 0.5);
        }
    }
    surface.cells = &cells;
    surface.source_archive = &archive;

    AdaptiveMeshParams params;
    params.voxel_size = 0.006f;
    params.min_cell_points = 10;
    params.min_cell_coverage = 0.01;
    params.poly_min_edge = 0.012;
    params.poly_max_edge = 0.25;
    params.poly_max_plane_error = 0.05;
    params.poly_max_leaf_count = 5000;

    const AdaptiveMeshResult adaptive_mesh = BuildAdaptivePatchMesh(surface, params);
    EXPECT_GT(adaptive_mesh.patches.size(), 0u);
    EXPECT_FALSE(adaptive_mesh.mesh.vertices_.empty());
    EXPECT_EQ(adaptive_mesh.mesh.triangles_.size(),
              adaptive_mesh.patches.size() * 2);
}

TEST(PlanarPolygonMesher, HighNoiseSubdivides) {
    AdaptiveSurfaceInput surface;
    surface.plane = Eigen::Vector4d(0.0, 0.0, 1.0, 0.0);
    surface.u = Eigen::Vector3d::UnitX();
    surface.v = Eigen::Vector3d::UnitY();
    surface.cell_size = 0.25;

    std::map<std::pair<int, int>, OccupiedCellView> cells;
    OccupiedCellView& cell = cells[{0, 0}];
    cell.points = 2500;
    cell.color_sum = Eigen::Vector3d(0.5, 0.5, 0.5);
    cell.min_u = 0.0;
    cell.max_u = 0.2;
    cell.min_v = 0.0;
    cell.max_v = 0.2;

    geometry::PointCloud archive;
    for (int i = 0; i < 50; ++i) {
        for (int j = 0; j < 50; ++j) {
            const double u = 0.004 * i;
            const double v = 0.004 * j;
            const double z = 0.05 * std::sin(u * 50.0) * std::sin(v * 50.0);
            archive.points_.emplace_back(u, v, z);
            archive.colors_.emplace_back(0.5, 0.5, 0.5);
        }
    }
    surface.cells = &cells;
    surface.source_archive = &archive;

    AdaptiveMeshParams params;
    params.voxel_size = 0.006f;
    params.min_cell_points = 10;
    params.min_cell_coverage = 0.001;
    params.poly_min_edge = 0.004;
    params.poly_max_edge = 0.25;
    params.poly_max_plane_error = 0.001;
    params.poly_max_leaf_count = 5000;

    const AdaptiveMeshResult adaptive_mesh =
            BuildAdaptivePatchMesh(surface, params);
    EXPECT_GT(adaptive_mesh.patches.size(), 1u);
}

TEST(PlanarPolygonMesher, LeafCountCap) {
    AdaptiveSurfaceInput surface;
    surface.plane = Eigen::Vector4d(0.0, 0.0, 1.0, 0.0);
    surface.u = Eigen::Vector3d::UnitX();
    surface.v = Eigen::Vector3d::UnitY();
    surface.cell_size = 0.25;

    std::map<std::pair<int, int>, OccupiedCellView> cells;
    OccupiedCellView& cell = cells[{0, 0}];
    cell.points = 2000;
    cell.color_sum = Eigen::Vector3d(0.5, 0.5, 0.5);
    cell.min_u = 0.0;
    cell.max_u = 0.2;
    cell.min_v = 0.0;
    cell.max_v = 0.2;

    geometry::PointCloud archive;
    for (int i = 0; i < 100; ++i) {
        for (int j = 0; j < 100; ++j) {
            const double z = 0.02 * std::sin(i * 0.5) * std::cos(j * 0.5);
            archive.points_.emplace_back(0.002 * i, 0.002 * j, z);
            archive.colors_.emplace_back(0.5, 0.5, 0.5);
        }
    }
    surface.cells = &cells;
    surface.source_archive = &archive;

    AdaptiveMeshParams params;
    params.voxel_size = 0.006f;
    params.min_cell_points = 10;
    params.min_cell_coverage = 0.01;
    params.poly_min_edge = 0.004;
    params.poly_max_edge = 0.25;
    params.poly_max_plane_error = 0.001;
    params.poly_max_leaf_count = 8;

    const AdaptiveMeshResult adaptive_mesh = BuildAdaptivePatchMesh(surface, params);
    EXPECT_LE(adaptive_mesh.patches.size(), 8u);
}

}  // namespace tests
}  // namespace open3d
