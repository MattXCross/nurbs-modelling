#pragma once

#include "entity_id.h"
#include "kernel_math.h"
#include "math_types.h"
#include "orbit_camera.h"
#include "scene.h"
#include "surface_tessellation.h"
#include "viewport_math.h"

#include <cstddef>
#include <optional>
#include <vector>

struct SurfacePickHit {
    EntityId entity;
    double distance{0.0};
    cad::Point3 position;
};

[[nodiscard]] std::vector<SurfacePickHit> pick_surfaces(
    const Scene& scene,
    const cad::Ray3& ray,
    const cad::SurfaceTessellationSettings& settings = {}
);
