#include "geometry_queries.h"
#include "nurbs_surface.h"
#include "surface_sampling.h"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <string_view>
#include <vector>

namespace {

int failures = 0;

void expect(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

bool nearly_equal(double left, double right, double tolerance = 1e-12) {
    return std::abs(left - right) <= tolerance;
}

void expect_point(cad::Point3 actual, cad::Point3 expected, std::string_view message) {
    expect(nearly_equal(actual.x, expected.x), message);
    expect(nearly_equal(actual.y, expected.y), message);
    expect(nearly_equal(actual.z, expected.z), message);
}

cad::Aabb3 make_unit_bounds() {
    const std::array points{
        cad::Point3{-1.0, -1.0, -1.0},
        cad::Point3{1.0, 1.0, 1.0}
    };
    return *cad::Aabb3::from_points(points);
}

void test_control_hull_bounds() {
    const std::vector<ControlPoint> points{
        {{-2.0, 3.0, 1.0}, 1.0},
        {{4.0, -1.0, 5.0}, 2.0},
        {{0.0, 2.0, -3.0}, 0.5},
    };
    const auto bounds = cad::control_hull_bounds(points);
    expect(bounds.has_value(), "compute control-hull bounds");
    if (bounds) {
        expect_point(*bounds->minimum(), {-2.0, -1.0, -3.0}, "control bounds minimum");
        expect_point(*bounds->maximum(), {4.0, 3.0, 5.0}, "control bounds maximum");
    }
    expect(!cad::control_hull_bounds({}), "empty control hull has no bounds");

    auto invalid = points;
    invalid[1].position.x = std::numeric_limits<double>::quiet_NaN();
    expect(!cad::control_hull_bounds(invalid), "non-finite control hull has no bounds");

    const double tiny = std::numeric_limits<double>::denorm_min();
    const std::vector<ControlPoint> tiny_points{
        {{0.0, 0.0, 0.0}, 1.0},
        {{tiny, tiny, tiny}, 1.0}
    };
    const auto tiny_bounds = cad::control_hull_bounds(tiny_points);
    expect(tiny_bounds.has_value() && tiny_bounds->contains({tiny, tiny, tiny}),
           "very small finite bounds are retained");

    const std::vector<ControlPoint> huge_points{
        {{std::numeric_limits<double>::lowest(), 0.0, 0.0}, 1.0},
        {{std::numeric_limits<double>::max(), 0.0, 0.0}, 1.0}
    };
    const auto huge_bounds = cad::control_hull_bounds(huge_points);
    expect(huge_bounds.has_value(), "very large finite bounds are retained");
    if (huge_bounds) {
        expect(!huge_bounds->extent(), "unrepresentable huge extent remains explicit");
    }
}

void test_surface_control_bounds() {
    std::vector<ControlPoint> points{
        {{-1.0, 0.0, 2.0}, 1.0},
        {{-1.0, 3.0, -2.0}, 1.0},
        {{4.0, 0.0, 1.0}, 1.0},
        {{4.0, 3.0, 0.0}, 1.0},
    };
    auto surface = NurbsSurface::create(2, 2, std::move(points));
    expect(surface.has_value(), "create bounded surface");
    if (surface) {
        const auto bounds = (*surface)->control_bounds();
        expect(bounds.has_value(), "surface exposes control bounds");
        if (bounds) {
            expect_point(*bounds->minimum(), {-1.0, 0.0, -2.0}, "surface bounds minimum");
            expect_point(*bounds->maximum(), {4.0, 3.0, 2.0}, "surface bounds maximum");
        }
    }
}

void test_ray_aabb_intersection() {
    const cad::Aabb3 bounds = make_unit_bounds();
    const auto tolerance = cad::GeometryTolerance::create(1e-6, 1e-12, 1e-9, 1e-12);
    expect(tolerance.has_value(), "create query tolerance");
    if (!tolerance) {
        return;
    }

    const auto ray = cad::Ray3::from_origin_direction({-2.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    const auto hit = cad::intersect_ray_aabb(*ray, bounds, *tolerance);
    expect(hit.has_value(), "ray intersects bounds");
    if (hit) {
        expect(nearly_equal(hit->entry_parameter, 1.0 - 1e-6), "expanded bounds entry parameter");
        expect(nearly_equal(hit->exit_parameter, 3.0 + 1e-6), "expanded bounds exit parameter");
    }

    const auto inside_ray = cad::Ray3::from_origin_direction({}, {1.0, 0.0, 0.0});
    const auto inside_hit = cad::intersect_ray_aabb(*inside_ray, bounds, *tolerance);
    expect(inside_hit.has_value() && inside_hit->entry_parameter == 0.0,
           "ray starting inside bounds enters at zero");

    const auto parallel_miss = cad::Ray3::from_origin_direction(
        {-2.0, 2.0, 0.0},
        {1.0, 0.0, 0.0}
    );
    expect(!cad::intersect_ray_aabb(*parallel_miss, bounds, *tolerance),
           "parallel ray outside slab misses");

    const auto grazing = cad::Ray3::from_origin_direction(
        {-2.0, 1.0 + 0.5e-6, 0.0},
        {1.0, 0.0, 0.0}
    );
    expect(cad::intersect_ray_aabb(*grazing, bounds, *tolerance).has_value(),
           "model tolerance admits grazing ray");
    const auto outside_tolerance = cad::Ray3::from_origin_direction(
        {-2.0, 1.0 + 2e-6, 0.0},
        {1.0, 0.0, 0.0}
    );
    expect(!cad::intersect_ray_aabb(*outside_tolerance, bounds, *tolerance),
           "ray outside model tolerance misses");

    expect(!cad::intersect_ray_aabb(*ray, cad::Aabb3{}, *tolerance),
           "empty bounds cannot be intersected");

    const auto behind_ray = cad::Ray3::from_origin_direction({2.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    expect(!cad::intersect_ray_aabb(*behind_ray, bounds, *tolerance),
           "box entirely behind ray origin misses");

    const auto tangent_ray = cad::Ray3::from_origin_direction({-2.0, 1.0, 1.0}, {1.0, 0.0, 0.0});
    expect(cad::intersect_ray_aabb(*tangent_ray, bounds, *tolerance).has_value(),
           "exact corner tangent intersects");

    const std::array distant_box_points{cad::Point3{1.0, -1.0, -1.0}, cad::Point3{2.0, 1.0, 1.0}};
    const auto distant_ray = cad::Ray3::from_origin_direction({1e18, 0.0, 0.0}, {-1.0, 0.0, 0.0});
    expect(!cad::intersect_ray_aabb(
               *distant_ray,
               *cad::Aabb3::from_points(distant_box_points),
               *tolerance
           ),
           "reject intersection not reproducible by returned double parameter");
}

void test_point_distance_queries() {
    const auto ray = cad::Ray3::from_origin_direction({}, {1.0, 0.0, 0.0});
    const auto ahead = cad::distance_to_ray({2.0, 3.0, 0.0}, *ray);
    expect(ahead.has_value(), "distance to forward ray");
    if (ahead) {
        expect(nearly_equal(ahead->distance, 3.0), "forward ray distance");
        expect(nearly_equal(ahead->ray_parameter, 2.0), "forward ray parameter");
        expect_point(ahead->closest_point, {2.0, 0.0, 0.0}, "forward ray closest point");
    }

    const auto behind = cad::distance_to_ray({-2.0, 3.0, 0.0}, *ray);
    expect(behind.has_value(), "distance behind ray origin");
    if (behind) {
        expect(nearly_equal(behind->distance, std::sqrt(13.0)), "behind-origin distance");
        expect(behind->ray_parameter == 0.0, "behind-origin parameter clamps to zero");
        expect_point(behind->closest_point, {}, "behind-origin closest point");
    }

    const auto plane = cad::Plane::from_point_normal({0.0, 2.0, 0.0}, {0.0, 1.0, 0.0});
    const auto plane_distance = cad::distance_to_plane({1.0, 5.0, 3.0}, *plane);
    expect(plane_distance.has_value(), "distance to plane");
    if (plane_distance) {
        expect(nearly_equal(plane_distance->distance, 3.0), "absolute plane distance");
        expect(nearly_equal(plane_distance->signed_distance, 3.0), "signed plane distance");
        expect_point(plane_distance->closest_point, {1.0, 2.0, 3.0}, "plane closest point");
    }

    expect(!cad::distance_to_ray(
               {std::numeric_limits<double>::infinity(), 0.0, 0.0},
               *ray
           ),
           "ray distance rejects non-finite point");

    const auto distant_ray = cad::Ray3::from_origin_direction({1e18, 0.0, 0.0}, {-1.0, 0.0, 0.0});
    expect(!cad::distance_to_ray({100.0, 3.0, 0.0}, *distant_ray),
           "reject closest point not reproducible by returned double parameter");

    const auto extreme_plane = cad::Plane::from_point_normal(
        {1e18, 1e18, 0.0},
        {1.0, 1.0, 0.0}
    );
    const auto defining_point_distance = cad::distance_to_plane({1e18, 1e18, 0.0}, *extreme_plane);
    expect(defining_point_distance.has_value() && defining_point_distance->distance == 0.0,
           "plane defining point remains on plane at large coordinates");
}

void test_knot_span_surface_sampling() {
    std::vector<ControlPoint> points;
    for (std::size_t u = 0; u < 5; ++u) {
        for (std::size_t v = 0; v < 2; ++v) {
            points.push_back({{
                static_cast<double>(u),
                static_cast<double>(v),
                static_cast<double>(u + v)
            }, 1.0});
        }
    }
    auto surface = NurbsSurface::create(
        5,
        2,
        2,
        1,
        std::move(points),
        {0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0},
        {0.0, 0.0, 1.0, 1.0}
    );
    expect(surface.has_value(), "create custom-knot sampled surface");
    if (!surface) {
        return;
    }

    const auto grid = sample_surface_by_knot_spans(**surface, 2);
    expect(grid.has_value(), "sample surface by knot spans");
    if (grid) {
        const std::vector<double> expected_u{0.0, 0.25, 0.5, 0.75, 1.0};
        const std::vector<double> expected_v{0.0, 0.5, 1.0};
        expect(std::ranges::equal(grid->u_parameters(), expected_u),
               "U samples include each nonempty knot-span boundary once");
        expect(std::ranges::equal(grid->v_parameters(), expected_v),
               "V samples subdivide the knot span");
        expect(grid->points().size() == expected_u.size() * expected_v.size(),
               "sample grid has rectangular point count");
        expect(!grid->point(grid->u_count(), 0), "sample grid rejects invalid index");
        const auto sampled = grid->point(2, 1);
        const auto evaluated = (*surface)->evaluate(0.5, 0.5);
        expect(sampled.has_value() && evaluated.has_value(), "read sampled knot point");
        if (sampled && evaluated) {
            expect_point(*sampled, *evaluated, "sample grid uses surface evaluation");
        }

        const auto bounds = (*surface)->control_bounds();
        for (const Point3D point : grid->points()) {
            expect(bounds->contains(point), "positive-weight surface sample stays in control hull");
        }
    }

    const auto invalid = sample_surface_by_knot_spans(**surface, 0);
    expect(!invalid && invalid.error() == SurfaceSamplingError::invalid_segments_per_span,
           "reject zero segments per span");

    std::vector<ControlPoint> collapsed(4, ControlPoint{{2.0, 3.0, 4.0}, 1.0});
    auto degenerate = NurbsSurface::create(2, 2, std::move(collapsed));
    const auto degenerate_grid = sample_surface_by_knot_spans(**degenerate, 1);
    expect(degenerate_grid.has_value(), "sample geometrically degenerate surface");
    if (degenerate_grid) {
        for (const Point3D point : degenerate_grid->points()) {
            expect_point(point, {2.0, 3.0, 4.0}, "degenerate samples remain stable");
        }
    }


    const double large_start = 1e18;
    const double adjacent = std::nextafter(large_start, std::numeric_limits<double>::infinity());
    auto adjacent_span = NurbsSurface::create(
        2,
        2,
        1,
        1,
        std::vector<ControlPoint>{
            {{0.0, 0.0, 0.0}, 1.0}, {{0.0, 1.0, 0.0}, 1.0},
            {{1.0, 0.0, 0.0}, 1.0}, {{1.0, 1.0, 0.0}, 1.0}
        },
        {large_start, large_start, adjacent, adjacent},
        {0.0, 0.0, 1.0, 1.0}
    );
    expect(adjacent_span.has_value(), "create adjacent-representable knot span");
    if (adjacent_span) {
        const auto unsampleable = sample_surface_by_knot_spans(**adjacent_span, 2);
        expect(!unsampleable &&
                   unsampleable.error() == SurfaceSamplingError::numeric_range_not_supported,
               "reject knot subdivision with no representable interior parameter");
    }
}

} // namespace

int main() {
    test_control_hull_bounds();
    test_surface_control_bounds();
    test_ray_aabb_intersection();
    test_point_distance_queries();
    test_knot_span_surface_sampling();

    if (failures != 0) {
        std::cerr << failures << " test assertion(s) failed\n";
        return 1;
    }
    return 0;
}
