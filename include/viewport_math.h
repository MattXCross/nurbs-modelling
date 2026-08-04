#pragma once

#include "kernel_math.h"
#include "math_types.h"
#include "orbit_camera.h"

#include <optional>

[[nodiscard]] std::optional<cad::Ray3> make_viewport_ray(
    Vec2 viewport_position,
    int viewport_width,
    int viewport_height,
    const CameraState& camera
) noexcept;
