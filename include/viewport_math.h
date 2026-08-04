#pragma once

#include "kernel_math.h"
#include "math_types.h"
#include "orbit_camera.h"

#include <optional>

struct ClipPlanes {
    double near_plane;
    double far_plane;
};

[[nodiscard]] std::optional<cad::Ray3> make_viewport_ray(
    Vec2 viewport_position,
    int viewport_width,
    int viewport_height,
    const CameraState& camera
) noexcept;

[[nodiscard]] std::optional<Vec2> project_to_viewport(
    cad::Point3 world_position,
    int viewport_width,
    int viewport_height,
    const CameraState& camera
) noexcept;

// Convenience form for gizmo paths that already reject invalid camera/viewport state.
[[nodiscard]] inline Vec2 project_to_viewport(
    cad::Point3 world_position,
    const CameraState& camera,
    int viewport_width,
    int viewport_height
) noexcept {
    return project_to_viewport(world_position, viewport_width, viewport_height, camera)
        .value_or(Vec2{});
}

[[nodiscard]] ClipPlanes derive_clip_planes(
    const CameraState& camera,
    const cad::Aabb3& visible_bounds
) noexcept;
