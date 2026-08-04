#include "viewport_math.h"

#include <cmath>
#include <limits>
#include <numbers>

namespace {
struct CameraBasis {
    cad::Vector3 forward;
    cad::Vector3 right;
    cad::Vector3 up;
};

std::optional<CameraBasis> camera_basis(const CameraState& camera) {
    const auto forward = cad::normalized(camera.target - camera.position);
    if (!forward) return std::nullopt;
    const auto right = cad::normalized(cad::cross(*forward, camera.up));
    if (!right) return std::nullopt;
    const auto up = cad::normalized(cad::cross(*right, *forward));
    if (!up) return std::nullopt;
    return CameraBasis{*forward, *right, *up};
}

bool valid_projection(const CameraState& camera) {
    if (camera.projection == ProjectionMode::orthographic) {
        return std::isfinite(camera.orthographic_vertical_size) &&
            camera.orthographic_vertical_size > 0.0;
    }
    return std::isfinite(camera.vertical_fov_degrees) &&
        camera.vertical_fov_degrees > 0.0f && camera.vertical_fov_degrees < 180.0f;
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
        !valid_projection(camera)) {
        return std::nullopt;
    }
    const auto basis = camera_basis(camera);
    if (!basis) return std::nullopt;

    const double normalized_x = 2.0 * viewport_position.x / viewport_width - 1.0;
    const double normalized_y = 1.0 - 2.0 * viewport_position.y / viewport_height;
    const double aspect = static_cast<double>(viewport_width) / viewport_height;
    if (camera.projection == ProjectionMode::orthographic) {
        const double half_height = camera.orthographic_vertical_size * 0.5;
        const cad::Point3 origin = camera.position +
            basis->right * (normalized_x * aspect * half_height) +
            basis->up * (normalized_y * half_height);
        return cad::Ray3::from_origin_direction(origin, basis->forward);
    }
    const double half_height = std::tan(
        static_cast<double>(camera.vertical_fov_degrees) * std::numbers::pi / 360.0
    );
    return cad::Ray3::from_origin_direction(
        camera.position,
        basis->forward + basis->right * (normalized_x * aspect * half_height) +
            basis->up * (normalized_y * half_height)
    );
}

std::optional<Vec2> project_to_viewport(
    cad::Point3 world_position,
    int viewport_width,
    int viewport_height,
    const CameraState& camera
) noexcept {
    if (viewport_width <= 0 || viewport_height <= 0 || !cad::is_finite(world_position) ||
        !valid_projection(camera)) return std::nullopt;
    const auto basis = camera_basis(camera);
    if (!basis) return std::nullopt;
    const cad::Vector3 offset = world_position - camera.position;
    const double depth = cad::dot(offset, basis->forward);
    if (!std::isfinite(depth) || depth <= 0.0) return std::nullopt;
    const double aspect = static_cast<double>(viewport_width) / viewport_height;
    const double half_height = camera.projection == ProjectionMode::orthographic
        ? camera.orthographic_vertical_size * 0.5
        : depth * std::tan(static_cast<double>(camera.vertical_fov_degrees) *
            std::numbers::pi / 360.0);
    if (!std::isfinite(half_height) || half_height <= 0.0) return std::nullopt;
    const double nx = cad::dot(offset, basis->right) / (half_height * aspect);
    const double ny = cad::dot(offset, basis->up) / half_height;
    return Vec2{
        static_cast<float>((nx + 1.0) * 0.5 * viewport_width),
        static_cast<float>((1.0 - ny) * 0.5 * viewport_height)
    };
}

ClipPlanes derive_clip_planes(const CameraState& camera, const cad::Aabb3& bounds) noexcept {
    const auto basis = camera_basis(camera);
    const auto minimum = bounds.minimum();
    const auto maximum = bounds.maximum();
    const double distance = cad::distance(camera.position, camera.target);
    const double fallback_scale = std::max(distance, 1.0);
    if (!basis || !minimum || !maximum) {
        return {std::max(fallback_scale * 1e-4, 1e-9), fallback_scale * 1000.0};
    }
    double minimum_depth = std::numeric_limits<double>::infinity();
    double maximum_depth = -std::numeric_limits<double>::infinity();
    for (const double x : {minimum->x, maximum->x}) {
        for (const double y : {minimum->y, maximum->y}) {
            for (const double z : {minimum->z, maximum->z}) {
                const double depth = cad::dot(cad::Point3{x, y, z} - camera.position, basis->forward);
                minimum_depth = std::min(minimum_depth, depth);
                maximum_depth = std::max(maximum_depth, depth);
            }
        }
    }
    const double span = std::max(maximum_depth - minimum_depth, fallback_scale * 1e-6);
    const double margin = span * 0.1;
    const double near_plane = std::max({minimum_depth - margin, span * 1e-6, 1e-12});
    const double far_plane = std::max(maximum_depth + margin, near_plane * 10.0);
    if (!std::isfinite(near_plane) || !std::isfinite(far_plane)) {
        return {std::max(fallback_scale * 1e-4, 1e-9), fallback_scale * 1000.0};
    }
    return {near_plane, far_plane};
}
