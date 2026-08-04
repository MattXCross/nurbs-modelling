#pragma once

#include "entity_id.h"
#include "kernel_math.h"
#include "math_types.h"
#include "orbit_camera.h"
#include "scene.h"

#include <cstddef>
#include <optional>
#include <vector>

struct SurfacePickHit {
    EntityId entity;
    double distance{0.0};
    cad::Point3 position;
};

[[nodiscard]] std::optional<cad::Ray3> make_viewport_ray(
    Vec2 viewport_position,
    int viewport_width,
    int viewport_height,
    const CameraState& camera
) noexcept;

[[nodiscard]] std::vector<SurfacePickHit> pick_surfaces(
    const Scene& scene,
    const cad::Ray3& ray,
    std::size_t segments_per_knot_span = 8
);
