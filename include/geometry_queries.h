#pragma once

#include "core.h"
#include "geometry_tolerance.h"
#include "kernel_math.h"

#include <optional>
#include <span>

namespace cad {

struct RayAabbIntersection {
    double entry_parameter;
    double exit_parameter;
    Point3 entry_point;
    Point3 exit_point;
};

struct PointRayDistance {
    double distance;
    double ray_parameter;
    Point3 closest_point;
};

struct PointPlaneDistance {
    double distance;
    double signed_distance;
    Point3 closest_point;
};

// Positive-weight rational geometry lies within this control-point convex hull.
[[nodiscard]] std::optional<Aabb3> control_hull_bounds(
    std::span<const ControlPoint> control_points
) noexcept;

// Intersects the forward half-ray. The box is expanded by model tolerance.
[[nodiscard]] std::optional<RayAabbIntersection> intersect_ray_aabb(
    const Ray3& ray,
    const Aabb3& bounds,
    const GeometryTolerance& tolerance
) noexcept;

[[nodiscard]] std::optional<PointRayDistance> distance_to_ray(
    Point3 point,
    const Ray3& ray
) noexcept;

[[nodiscard]] std::optional<PointPlaneDistance> distance_to_plane(
    Point3 point,
    const Plane& plane
) noexcept;

} // namespace cad
