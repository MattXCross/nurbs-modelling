#include "surface_picking.h"

#include "geometry_queries.h"
#include "geometry_tolerance.h"
#include "surface_sampling.h"

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

std::optional<double> intersect_sample_grid(
    const cad::Ray3& ray,
    const NurbsSurfaceSampleGrid& samples
) noexcept {
    std::optional<double> closest;
    for (std::size_t u = 0; u + 1 < samples.u_count(); ++u) {
        for (std::size_t v = 0; v + 1 < samples.v_count(); ++v) {
            const auto p00 = samples.point(u, v);
            const auto p10 = samples.point(u + 1, v);
            const auto p01 = samples.point(u, v + 1);
            const auto p11 = samples.point(u + 1, v + 1);
            if (!p00 || !p10 || !p01 || !p11) {
                continue;
            }
            for (const auto hit : {
                     intersect_triangle(ray, *p00, *p10, *p11),
                     intersect_triangle(ray, *p00, *p11, *p01)
                 }) {
                if (hit && (!closest || *hit < *closest)) {
                    closest = *hit;
                }
            }
        }
    }
    return closest;
}

} // namespace

std::optional<cad::Ray3> make_viewport_ray(
    Vec2 viewport_position,
    int viewport_width,
    int viewport_height,
    const CameraState& camera
) noexcept {
    if (viewport_width <= 0 || viewport_height <= 0 ||
        !std::isfinite(viewport_position.x) || !std::isfinite(viewport_position.y) ||
        !std::isfinite(camera.vertical_fov_degrees) ||
        camera.vertical_fov_degrees <= 0.0f || camera.vertical_fov_degrees >= 180.0f) {
        return std::nullopt;
    }
    const auto forward = cad::normalized(camera.target - camera.position);
    if (!forward) {
        return std::nullopt;
    }
    const auto right = cad::normalized(cad::cross(*forward, camera.up));
    if (!right) {
        return std::nullopt;
    }
    const auto screen_up = cad::normalized(cad::cross(*right, *forward));
    if (!screen_up) {
        return std::nullopt;
    }

    const double normalized_x = 2.0 * static_cast<double>(viewport_position.x) /
        static_cast<double>(viewport_width) - 1.0;
    const double normalized_y = 1.0 - 2.0 * static_cast<double>(viewport_position.y) /
        static_cast<double>(viewport_height);
    const double half_height = std::tan(
        static_cast<double>(camera.vertical_fov_degrees) * std::numbers::pi / 360.0
    );
    const double aspect = static_cast<double>(viewport_width) /
        static_cast<double>(viewport_height);
    return cad::Ray3::from_origin_direction(
        camera.position,
        *forward + *right * (normalized_x * aspect * half_height) +
            *screen_up * (normalized_y * half_height)
    );
}

std::vector<SurfacePickHit> pick_surfaces(
    const Scene& scene,
    const cad::Ray3& ray,
    std::size_t segments_per_knot_span
) {
    std::vector<SurfacePickHit> hits;
    if (segments_per_knot_span == 0) {
        return hits;
    }
    const cad::GeometryTolerance tolerance = cad::GeometryTolerance::defaults();
    for (const SceneNode& node : scene.nodes()) {
        if (!node.visible || node.surface == nullptr) {
            continue;
        }
        const auto bounds = node.surface->control_bounds();
        if (!bounds || !cad::intersect_ray_aabb(ray, *bounds, tolerance)) {
            continue;
        }
        const auto samples = sample_surface_by_knot_spans(
            *node.surface,
            segments_per_knot_span
        );
        if (!samples) {
            continue;
        }
        const auto distance = intersect_sample_grid(ray, *samples);
        if (distance) {
            hits.push_back(SurfacePickHit{node.id, *distance, ray.at(*distance)});
        }
    }
    std::ranges::sort(hits, {}, &SurfacePickHit::distance);
    return hits;
}
