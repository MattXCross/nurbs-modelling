#include "geometry_queries.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace cad {
namespace {

bool representable(long double value) {
    return std::isfinite(value) &&
        value >= std::numeric_limits<double>::lowest() &&
        value <= std::numeric_limits<double>::max();
}

std::optional<Point3> point_from_long_double(
    long double x,
    long double y,
    long double z
) {
    if (!representable(x) || !representable(y) || !representable(z)) {
        return std::nullopt;
    }
    return Point3{
        static_cast<double>(x),
        static_cast<double>(y),
        static_cast<double>(z)
    };
}

bool point_matches_ideal(Point3 actual, Point3 ideal, double absolute_tolerance) {
    constexpr double rounding_factor = 16.0 * std::numeric_limits<double>::epsilon();
    const auto coordinate_matches = [absolute_tolerance, rounding_factor](
        double value,
        double expected
    ) {
        const double rounding_tolerance = rounding_factor * std::max(1.0, std::abs(expected));
        return std::abs(value - expected) <= std::max(absolute_tolerance, rounding_tolerance);
    };
    return coordinate_matches(actual.x, ideal.x) &&
        coordinate_matches(actual.y, ideal.y) &&
        coordinate_matches(actual.z, ideal.z);
}

} // namespace

std::optional<Aabb3> control_hull_bounds(
    std::span<const ControlPoint> control_points
) noexcept {
    if (control_points.empty()) {
        return std::nullopt;
    }
    Aabb3 bounds;
    for (const ControlPoint& control_point : control_points) {
        if (!bounds.expand(control_point.position)) {
            return std::nullopt;
        }
    }
    return bounds;
}

std::optional<RayAabbIntersection> intersect_ray_aabb(
    const Ray3& ray,
    const Aabb3& bounds,
    const GeometryTolerance& tolerance
) noexcept {
    const auto minimum = bounds.minimum();
    const auto maximum = bounds.maximum();
    if (!minimum || !maximum) {
        return std::nullopt;
    }

    const Point3 origin = ray.origin();
    const Vector3 direction = ray.direction();
    const std::array<long double, 3> origins{origin.x, origin.y, origin.z};
    const std::array<long double, 3> directions{direction.x, direction.y, direction.z};
    const long double expansion = tolerance.model().absolute();
    const std::array<long double, 3> minima{
        static_cast<long double>(minimum->x) - expansion,
        static_cast<long double>(minimum->y) - expansion,
        static_cast<long double>(minimum->z) - expansion
    };
    const std::array<long double, 3> maxima{
        static_cast<long double>(maximum->x) + expansion,
        static_cast<long double>(maximum->y) + expansion,
        static_cast<long double>(maximum->z) + expansion
    };

    long double entry = 0.0L;
    long double exit = std::numeric_limits<long double>::max();
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(minima[axis]) || !std::isfinite(maxima[axis])) {
            return std::nullopt;
        }
        if (directions[axis] == 0.0L) {
            if (origins[axis] < minima[axis] || origins[axis] > maxima[axis]) {
                return std::nullopt;
            }
            continue;
        }

        long double first = (minima[axis] - origins[axis]) / directions[axis];
        long double second = (maxima[axis] - origins[axis]) / directions[axis];
        if (!std::isfinite(first) || !std::isfinite(second)) {
            return std::nullopt;
        }
        if (first > second) {
            std::swap(first, second);
        }
        entry = std::max(entry, first);
        exit = std::min(exit, second);
        if (entry > exit) {
            return std::nullopt;
        }
    }

    if (!representable(entry) || !representable(exit)) {
        return std::nullopt;
    }
    const double entry_parameter = static_cast<double>(entry);
    const double exit_parameter = static_cast<double>(exit);
    const Point3 entry_point = ray.at(entry_parameter);
    const Point3 exit_point = ray.at(exit_parameter);
    const auto ideal_entry = point_from_long_double(
        static_cast<long double>(origin.x) + entry * direction.x,
        static_cast<long double>(origin.y) + entry * direction.y,
        static_cast<long double>(origin.z) + entry * direction.z
    );
    const auto ideal_exit = point_from_long_double(
        static_cast<long double>(origin.x) + exit * direction.x,
        static_cast<long double>(origin.y) + exit * direction.y,
        static_cast<long double>(origin.z) + exit * direction.z
    );
    if (!is_finite(entry_point) || !is_finite(exit_point) || !ideal_entry || !ideal_exit ||
        !point_matches_ideal(entry_point, *ideal_entry, tolerance.model().absolute()) ||
        !point_matches_ideal(exit_point, *ideal_exit, tolerance.model().absolute())) {
        return std::nullopt;
    }
    return RayAabbIntersection{
        entry_parameter,
        exit_parameter,
        entry_point,
        exit_point
    };
}

std::optional<PointRayDistance> distance_to_ray(Point3 point, const Ray3& ray) noexcept {
    if (!is_finite(point)) {
        return std::nullopt;
    }
    const Point3 origin = ray.origin();
    const Vector3 direction = ray.direction();
    const long double offset_x =
        static_cast<long double>(point.x) - static_cast<long double>(origin.x);
    const long double offset_y =
        static_cast<long double>(point.y) - static_cast<long double>(origin.y);
    const long double offset_z =
        static_cast<long double>(point.z) - static_cast<long double>(origin.z);
    long double parameter =
        offset_x * direction.x + offset_y * direction.y + offset_z * direction.z;
    parameter = std::max(parameter, 0.0L);
    if (!representable(parameter)) {
        return std::nullopt;
    }

    const long double closest_x = static_cast<long double>(origin.x) +
        parameter * direction.x;
    const long double closest_y = static_cast<long double>(origin.y) +
        parameter * direction.y;
    const long double closest_z = static_cast<long double>(origin.z) +
        parameter * direction.z;
    const auto ideal_closest = point_from_long_double(closest_x, closest_y, closest_z);
    const double returned_parameter = static_cast<double>(parameter);
    const Point3 closest = ray.at(returned_parameter);
    if (!ideal_closest || !is_finite(closest) ||
        !point_matches_ideal(closest, *ideal_closest, 0.0)) {
        return std::nullopt;
    }

    const long double separation = std::hypot(
        static_cast<long double>(point.x) - closest.x,
        static_cast<long double>(point.y) - closest.y,
        static_cast<long double>(point.z) - closest.z
    );
    if (!representable(separation)) {
        return std::nullopt;
    }
    return PointRayDistance{
        static_cast<double>(separation),
        returned_parameter,
        closest
    };
}

std::optional<PointPlaneDistance> distance_to_plane(Point3 point, const Plane& plane) noexcept {
    if (!is_finite(point)) {
        return std::nullopt;
    }
    const Vector3 normal = plane.normal();
    const Point3 origin = plane.origin();
    const long double signed_distance =
        static_cast<long double>(normal.x) *
            (static_cast<long double>(point.x) - origin.x) +
        static_cast<long double>(normal.y) *
            (static_cast<long double>(point.y) - origin.y) +
        static_cast<long double>(normal.z) *
            (static_cast<long double>(point.z) - origin.z);
    if (!representable(signed_distance)) {
        return std::nullopt;
    }
    const auto closest = point_from_long_double(
        static_cast<long double>(point.x) - signed_distance * normal.x,
        static_cast<long double>(point.y) - signed_distance * normal.y,
        static_cast<long double>(point.z) - signed_distance * normal.z
    );
    if (!closest) {
        return std::nullopt;
    }
    return PointPlaneDistance{
        static_cast<double>(std::abs(signed_distance)),
        static_cast<double>(signed_distance),
        *closest
    };
}

} // namespace cad
