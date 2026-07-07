// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------
//
// Adaptive quadtree polygon mesher for planar surfaces in online SLAM freeze.
// Builds variable-size quads from source_archive points on a fitted plane.

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "open3d/geometry/PointCloud.h"
#include "open3d/geometry/TriangleMesh.h"

namespace open3d {
namespace examples {
namespace object_mesh {
namespace planar_polygon {

/// Metadata for one adaptive leaf quad (GUI / JSON).
struct PatchLeaf {
    double min_u = 0.0;
    double max_u = 0.0;
    double min_v = 0.0;
    double max_v = 0.0;
    int point_count = 0;
    double plane_rmse = 0.0;
    Eigen::Vector3d color = Eigen::Vector3d(0.7, 0.7, 0.7);
};

struct OccupiedCellView {
    int points = 0;
    Eigen::Vector3d color_sum = Eigen::Vector3d::Zero();
    double min_u = std::numeric_limits<double>::max();
    double max_u = std::numeric_limits<double>::lowest();
    double min_v = std::numeric_limits<double>::max();
    double max_v = std::numeric_limits<double>::lowest();
};

/// Inputs for adaptive meshing (view of PlaneSurfaceAtlas::Surface).
struct AdaptiveSurfaceInput {
    Eigen::Vector4d plane = Eigen::Vector4d::Zero();
    Eigen::Vector3d u = Eigen::Vector3d::UnitX();
    Eigen::Vector3d v = Eigen::Vector3d::UnitY();
    double cell_size = 0.25;
    const std::map<std::pair<int, int>, OccupiedCellView>* cells = nullptr;
    const geometry::PointCloud* source_archive = nullptr;
};

struct AdaptiveMeshParams {
    float voxel_size = 3.0f / 512.0f;
    int min_cell_points = 10;
    double min_cell_coverage = 0.2;
    double poly_min_edge = 0.006;
    double poly_max_edge = 0.25;
    double poly_max_plane_error = 0.01;
    int poly_max_leaf_count = 5000;
    bool poly_use_boundary_hull = false;
    double poly_hull_alpha = 0.012;
};

struct AdaptiveMeshResult {
    geometry::TriangleMesh mesh;
    std::vector<PatchLeaf> patches;
    std::vector<int> boundary_vertices;
    int occupied_cells = 0;
    int occupy_threshold = 0;
    Eigen::Vector3d aabb_min = Eigen::Vector3d::Zero();
    Eigen::Vector3d aabb_max = Eigen::Vector3d::Zero();
};

namespace detail {

inline Eigen::Vector3d CornerOnPlane(const AdaptiveSurfaceInput& surface,
                                     double a,
                                     double b) {
    const Eigen::Vector3d n = surface.plane.head<3>();
    Eigen::Vector3d p = a * surface.u + b * surface.v;
    p += (-(n.dot(p) + surface.plane(3))) * n;
    return p;
}

inline double ComputePlaneRmse(const std::vector<Eigen::Vector3d>& points,
                               const Eigen::Vector4d& plane) {
    if (points.empty() || plane.head<3>().norm() < 1e-9) {
        return 0.0;
    }
    Eigen::Vector4d p = plane;
    p /= p.head<3>().norm();
    const Eigen::Vector3d n = p.head<3>();
    double sum_sq = 0.0;
    for (const auto& pt : points) {
        const double d = n.dot(pt) + p(3);
        sum_sq += d * d;
    }
    return std::sqrt(sum_sq / static_cast<double>(points.size()));
}

struct UVSample {
    double u = 0.0;
    double v = 0.0;
    Eigen::Vector3d world;
    Eigen::Vector3d color;
};

inline std::vector<UVSample> ProjectArchiveToUV(
        const AdaptiveSurfaceInput& surface) {
    std::vector<UVSample> samples;
    if (surface.source_archive == nullptr ||
        surface.source_archive->points_.empty()) {
        return samples;
    }
    const geometry::PointCloud& archive = *surface.source_archive;
    const bool has_colors =
            archive.colors_.size() == archive.points_.size();
    const Eigen::Vector3d n = surface.plane.head<3>();
    samples.reserve(archive.points_.size());
    for (size_t k = 0; k < archive.points_.size(); ++k) {
        const Eigen::Vector3d& pt = archive.points_[k];
        const double d = n.dot(pt) + surface.plane(3);
        const Eigen::Vector3d on_plane = pt - d * n;
        UVSample sample;
        sample.u = surface.u.dot(on_plane);
        sample.v = surface.v.dot(on_plane);
        sample.world = pt;  // raw point for plane-error refinement
        sample.color = has_colors ? archive.colors_[k]
                                  : Eigen::Vector3d(0.7, 0.7, 0.7);
        samples.push_back(sample);
    }
    return samples;
}

inline void CollectSamplesInBox(const std::vector<UVSample>& samples,
                                double u0,
                                double u1,
                                double v0,
                                double v1,
                                std::vector<size_t>& out_indices) {
    out_indices.clear();
    for (size_t k = 0; k < samples.size(); ++k) {
        const UVSample& s = samples[k];
        if (s.u >= u0 && s.u <= u1 && s.v >= v0 && s.v <= v1) {
            out_indices.push_back(k);
        }
    }
}

/// Simple 2D point-in-polygon (ray casting) for hull boundary clip.
inline bool PointInPolygon2D(double u,
                             double v,
                             const std::vector<Eigen::Vector2d>& polygon) {
    if (polygon.size() < 3) {
        return true;
    }
    bool inside = false;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const double ui = polygon[i].x();
        const double vi = polygon[i].y();
        const double uj = polygon[j].x();
        const double vj = polygon[j].y();
        const bool intersect =
                ((vi > v) != (vj > v)) &&
                (u < (uj - ui) * (v - vi) / (vj - vi + 1e-12) + ui);
        if (intersect) {
            inside = !inside;
        }
    }
    return inside;
}

/// Coarse boundary polygon from occupied cell corners (convex-ish outline).
inline std::vector<Eigen::Vector2d> BuildOccupiedBoundaryPolygon(
        const AdaptiveSurfaceInput& surface,
        const std::map<std::pair<int, int>, const OccupiedCellView*>&
                occupied) {
    std::vector<Eigen::Vector2d> samples;
    samples.reserve(occupied.size() * 4);
    for (const auto& [cell_ij, cell_ptr] : occupied) {
        (void)cell_ij;
        const OccupiedCellView& cell = *cell_ptr;
        if (cell.max_u <= cell.min_u || cell.max_v <= cell.min_v) {
            continue;
        }
        samples.emplace_back(cell.min_u, cell.min_v);
        samples.emplace_back(cell.max_u, cell.min_v);
        samples.emplace_back(cell.max_u, cell.max_v);
        samples.emplace_back(cell.min_u, cell.max_v);
    }
    if (samples.size() < 3) {
        return samples;
    }
    // Convex hull (monotone chain) for a conservative boundary mask.
    std::sort(samples.begin(), samples.end(),
              [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
                  return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
              });
    auto cross = [](const Eigen::Vector2d& o, const Eigen::Vector2d& a,
                    const Eigen::Vector2d& b) {
        return (a.x() - o.x()) * (b.y() - o.y()) -
               (a.y() - o.y()) * (b.x() - o.x());
    };
    std::vector<Eigen::Vector2d> hull;
    for (const auto& p : samples) {
        while (hull.size() >= 2 &&
               cross(hull[hull.size() - 2], hull.back(), p) <= 0) {
            hull.pop_back();
        }
        hull.push_back(p);
    }
    const size_t lower = hull.size();
    for (auto it = samples.rbegin(); it != samples.rend(); ++it) {
        while (hull.size() > lower &&
               cross(hull[hull.size() - 2], hull.back(), *it) <= 0) {
            hull.pop_back();
        }
        hull.push_back(*it);
    }
    if (!hull.empty()) {
        hull.pop_back();
    }
    return hull;
}

struct LeafRecord {
    double min_u = 0.0;
    double max_u = 0.0;
    double min_v = 0.0;
    double max_v = 0.0;
    int point_count = 0;
    double plane_rmse = 0.0;
    Eigen::Vector3d color = Eigen::Vector3d(0.7, 0.7, 0.7);
};

inline bool SharesEdge(const LeafRecord& a,
                       const LeafRecord& b,
                       double eps,
                       int edge_a) {
    // edge_a: 0=bottom, 1=right, 2=top, 3=left
    auto overlap = [](double lo0, double hi0, double lo1, double hi1) {
        return lo0 <= hi1 + 1e-9 && lo1 <= hi0 + 1e-9;
    };
    switch (edge_a) {
        case 0:  // bottom: v = min_v
            return std::abs(a.min_v - b.max_v) <= eps &&
                   overlap(a.min_u, a.max_u, b.min_u, b.max_u);
        case 1:
            return std::abs(a.max_u - b.min_u) <= eps &&
                   overlap(a.min_v, a.max_v, b.min_v, b.max_v);
        case 2:
            return std::abs(a.max_v - b.min_v) <= eps &&
                   overlap(a.min_u, a.max_u, b.min_u, b.max_u);
        case 3:
            return std::abs(a.min_u - b.max_u) <= eps &&
                   overlap(a.min_v, a.max_v, b.min_v, b.max_v);
        default:
            return false;
    }
}

}  // namespace detail

/// Build an adaptive quad mesh on a planar surface from archived points.
inline AdaptiveMeshResult BuildAdaptivePatchMesh(
        const AdaptiveSurfaceInput& surface, const AdaptiveMeshParams& params) {
    AdaptiveMeshResult mesh_out;
    if (surface.cells == nullptr || surface.source_archive == nullptr ||
        surface.plane.head<3>().norm() < 1e-9) {
        return mesh_out;
    }

    const double voxels_per_cell =
            (surface.cell_size / std::max(1e-4f, params.voxel_size)) *
            (surface.cell_size / std::max(1e-4f, params.voxel_size));
    const int occupy_threshold = std::max(
            params.min_cell_points,
            static_cast<int>(params.min_cell_coverage * voxels_per_cell));
    mesh_out.occupy_threshold = occupy_threshold;

    std::map<std::pair<int, int>, const OccupiedCellView*> occupied;
    for (const auto& [cell_ij, cell] : *surface.cells) {
        if (cell.points >= occupy_threshold) {
            occupied.emplace(cell_ij, &cell);
        }
    }
    mesh_out.occupied_cells = static_cast<int>(occupied.size());
    if (occupied.empty()) {
        return mesh_out;
    }

    const std::vector<detail::UVSample> uv_samples =
            detail::ProjectArchiveToUV(surface);
    if (uv_samples.empty()) {
        return mesh_out;
    }

    std::vector<Eigen::Vector2d> boundary_polygon;
    if (params.poly_use_boundary_hull) {
        boundary_polygon =
                detail::BuildOccupiedBoundaryPolygon(surface, occupied);
    }

    const double poly_min_edge =
            params.poly_min_edge > 0.0
                    ? params.poly_min_edge
                    : 2.0 * static_cast<double>(params.voxel_size);
    const double poly_max_edge =
            params.poly_max_edge > 0.0 ? params.poly_max_edge
                                       : surface.cell_size;
    const double poly_max_plane_error =
            params.poly_max_plane_error > 0.0
                    ? params.poly_max_plane_error
                    : 0.01;

    std::vector<detail::LeafRecord> leaves;
    leaves.reserve(static_cast<size_t>(occupied.size()) * 4);

    struct Node {
        double u0, u1, v0, v1;
    };
    std::vector<Node> stack;

    for (const auto& [cell_ij, cell_ptr] : occupied) {
        (void)cell_ij;
        const OccupiedCellView& cell = *cell_ptr;
        if (cell.max_u <= cell.min_u || cell.max_v <= cell.min_v) {
            continue;
        }
        stack.push_back({cell.min_u, cell.max_u, cell.min_v, cell.max_v});
    }

    std::vector<size_t> indices;
    std::vector<Eigen::Vector3d> world_points;
    while (!stack.empty()) {
        if (static_cast<int>(leaves.size()) >= params.poly_max_leaf_count) {
            break;
        }
        const Node node = stack.back();
        stack.pop_back();

        detail::CollectSamplesInBox(uv_samples, node.u0, node.u1, node.v0,
                                    node.v1, indices);
        if (indices.empty()) {
            continue;
        }

        const double edge_u = node.u1 - node.u0;
        const double edge_v = node.v1 - node.v0;
        const double max_edge = std::max(edge_u, edge_v);
        const bool can_split = max_edge > poly_min_edge * 2.0;

        if (indices.size() <
                    static_cast<size_t>(params.min_cell_points) &&
            can_split) {
            continue;
        }

        world_points.clear();
        world_points.reserve(indices.size());
        Eigen::Vector3d color_sum = Eigen::Vector3d::Zero();
        double u_lo = std::numeric_limits<double>::max();
        double u_hi = std::numeric_limits<double>::lowest();
        double v_lo = u_lo, v_hi = u_hi;
        for (size_t idx : indices) {
            const detail::UVSample& s = uv_samples[idx];
            world_points.push_back(s.world);
            color_sum += s.color;
            u_lo = std::min(u_lo, s.u);
            u_hi = std::max(u_hi, s.u);
            v_lo = std::min(v_lo, s.v);
            v_hi = std::max(v_hi, s.v);
        }
        const double rmse =
                detail::ComputePlaneRmse(world_points, surface.plane);

        const bool need_split =
                can_split &&
                ((rmse > poly_max_plane_error) ||
                 (max_edge > poly_max_edge &&
                  indices.size() >
                          static_cast<size_t>(params.min_cell_points * 2)));

        if (need_split) {
            const double um = 0.5 * (node.u0 + node.u1);
            const double vm = 0.5 * (node.v0 + node.v1);
            stack.push_back({node.u0, um, node.v0, vm});
            stack.push_back({um, node.u1, node.v0, vm});
            stack.push_back({um, node.u1, vm, node.v1});
            stack.push_back({node.u0, um, vm, node.v1});
            continue;
        }

        if (params.poly_use_boundary_hull && !boundary_polygon.empty()) {
            const double cu = 0.5 * (u_lo + u_hi);
            const double cv = 0.5 * (v_lo + v_hi);
            if (!detail::PointInPolygon2D(cu, cv, boundary_polygon)) {
                continue;
            }
        }

        detail::LeafRecord leaf;
        leaf.min_u = u_lo;
        leaf.max_u = u_hi;
        leaf.min_v = v_lo;
        leaf.max_v = v_hi;
        leaf.point_count = static_cast<int>(indices.size());
        leaf.plane_rmse = rmse;
        leaf.color = (color_sum / static_cast<double>(indices.size()))
                             .cwiseMax(Eigen::Vector3d::Zero())
                             .cwiseMin(Eigen::Vector3d::Ones());
        leaves.push_back(leaf);
    }

    if (leaves.empty()) {
        return mesh_out;
    }

    const double edge_eps = poly_min_edge * 0.5;
    std::set<int> boundary;
    mesh_out.mesh.Clear();
    mesh_out.patches.reserve(leaves.size());

    for (size_t li = 0; li < leaves.size(); ++li) {
        const detail::LeafRecord& leaf = leaves[li];
        if (leaf.max_u <= leaf.min_u || leaf.max_v <= leaf.min_v) {
            continue;
        }
        const int base =
                static_cast<int>(mesh_out.mesh.vertices_.size());
        mesh_out.mesh.vertices_.push_back(
                detail::CornerOnPlane(surface, leaf.min_u, leaf.min_v));
        mesh_out.mesh.vertices_.push_back(
                detail::CornerOnPlane(surface, leaf.max_u, leaf.min_v));
        mesh_out.mesh.vertices_.push_back(
                detail::CornerOnPlane(surface, leaf.max_u, leaf.max_v));
        mesh_out.mesh.vertices_.push_back(
                detail::CornerOnPlane(surface, leaf.min_u, leaf.max_v));

        const Eigen::Vector3d color = leaf.color;
        for (int k = 0; k < 4; ++k) {
            mesh_out.mesh.vertex_colors_.push_back(color);
        }
        mesh_out.mesh.triangles_.push_back({base, base + 1, base + 2});
        mesh_out.mesh.triangles_.push_back({base, base + 2, base + 3});

        PatchLeaf patch;
        patch.min_u = leaf.min_u;
        patch.max_u = leaf.max_u;
        patch.min_v = leaf.min_v;
        patch.max_v = leaf.max_v;
        patch.point_count = leaf.point_count;
        patch.plane_rmse = leaf.plane_rmse;
        patch.color = color;
        mesh_out.patches.push_back(patch);

        auto mark_edge = [&](int edge_id, int va, int vb) {
            bool has_neighbor = false;
            for (size_t lj = 0; lj < leaves.size(); ++lj) {
                if (lj == li) {
                    continue;
                }
                if (detail::SharesEdge(leaf, leaves[lj], edge_eps, edge_id)) {
                    has_neighbor = true;
                    break;
                }
            }
            if (!has_neighbor) {
                boundary.insert(base + va);
                boundary.insert(base + vb);
            }
        };
        mark_edge(0, 0, 1);
        mark_edge(1, 1, 2);
        mark_edge(2, 2, 3);
        mark_edge(3, 3, 0);
    }

    mesh_out.boundary_vertices.assign(boundary.begin(), boundary.end());
    mesh_out.aabb_min = Eigen::Vector3d::Constant(
            std::numeric_limits<double>::max());
    mesh_out.aabb_max = Eigen::Vector3d::Constant(
            std::numeric_limits<double>::lowest());
    for (const auto& p : mesh_out.mesh.vertices_) {
        mesh_out.aabb_min = mesh_out.aabb_min.cwiseMin(p);
        mesh_out.aabb_max = mesh_out.aabb_max.cwiseMax(p);
    }
    return mesh_out;
}

}  // namespace planar_polygon
}  // namespace object_mesh
}  // namespace examples
}  // namespace open3d
