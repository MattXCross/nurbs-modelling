#include "surface_picking.h"

#include "geometry_queries.h"
#include "geometry_tolerance.h"
#include "surface_tessellation.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

namespace {

std::optional<double> intersect_triangle(
    const cad::Ray3& ray,
    cad::Point3 a,
    cad::Point3 b,
    cad::Point3 c
) noexcept {
    const cad::Vector3 edge_ab = b - a;
    const cad::Vector3 edge_ac = c - a;
    const cad::Vector3 perpendicular = cad::cross(ray.direction(), edge_ac);
    const double determinant = cad::dot(edge_ab, perpendicular);
    const double determinant_scale = cad::length(edge_ab) * cad::length(edge_ac);
    if (!std::isfinite(determinant) || !std::isfinite(determinant_scale) ||
        determinant_scale == 0.0 ||
        std::abs(determinant) <= determinant_scale * 1e-12) {
        return std::nullopt;
    }

    const double inverse = 1.0 / determinant;
    const cad::Vector3 origin_offset = ray.origin() - a;
    const double u = cad::dot(origin_offset, perpendicular) * inverse;
    constexpr double barycentric_tolerance = 1e-12;
    if (!std::isfinite(u) || u < -barycentric_tolerance ||
        u > 1.0 + barycentric_tolerance) {
        return std::nullopt;
    }

    const cad::Vector3 cross_offset = cad::cross(origin_offset, edge_ab);
    const double v = cad::dot(ray.direction(), cross_offset) * inverse;
    if (!std::isfinite(v) || v < -barycentric_tolerance ||
        u + v > 1.0 + barycentric_tolerance) {
        return std::nullopt;
    }
    const double distance = cad::dot(edge_ac, cross_offset) * inverse;
    if (!std::isfinite(distance) || distance < 0.0) {
        return std::nullopt;
    }
    return distance;
}

std::optional<double> intersect_mesh(
    const cad::Ray3& ray,
    const cad::SurfaceMesh& mesh
) noexcept {
    std::optional<double> closest;
    for (std::size_t index = 0; index + 2 < mesh.triangle_indices.size(); index += 3) {
        const std::uint32_t a = mesh.triangle_indices[index];
        const std::uint32_t b = mesh.triangle_indices[index + 1];
        const std::uint32_t c = mesh.triangle_indices[index + 2];
        if (a >= mesh.positions.size() || b >= mesh.positions.size() || c >= mesh.positions.size()) {
            continue;
        }
        const auto hit = intersect_triangle(
            ray, mesh.positions[a], mesh.positions[b], mesh.positions[c]
        );
        if (hit && (!closest || *hit < *closest)) {
            closest = *hit;
        }
    }
    return closest;
}

} // namespace

std::vector<SurfacePickHit> pick_surfaces(
    const Scene& scene,
    const cad::Ray3& ray,
    const cad::SurfaceTessellationSettings& tessellation_settings
) {
    std::vector<SurfacePickHit> hits;
    static cad::SurfaceTessellationCache mesh_cache;
    const cad::GeometryTolerance tolerance = cad::GeometryTolerance::defaults();
    for (const SceneNode& node : scene.nodes()) {
        if (!node.visible || node.surface == nullptr) {
            continue;
        }
        const auto bounds = node.surface->control_bounds();
        if (!bounds || !cad::intersect_ray_aabb(ray, *bounds, tolerance)) {
            continue;
        }
        const auto mesh = mesh_cache.get(
            *node.surface,
            node.geometry_revision,
            tessellation_settings
        );
        if (!mesh) {
            continue;
        }
        const auto distance = intersect_mesh(ray, **mesh);
        if (distance) {
            hits.push_back(SurfacePickHit{node.id, *distance, ray.at(*distance)});
        }
    }
    std::ranges::sort(hits, {}, &SurfacePickHit::distance);
    return hits;
}
