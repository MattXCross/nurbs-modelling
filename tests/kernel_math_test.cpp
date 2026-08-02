#include "kernel_math.h"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <string_view>

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

void expect_vector(cad::Vector3 actual, cad::Vector3 expected, std::string_view message) {
    expect(nearly_equal(actual.x, expected.x), message);
    expect(nearly_equal(actual.y, expected.y), message);
    expect(nearly_equal(actual.z, expected.z), message);
}

void test_point_and_vector_operations() {
    constexpr cad::Point3 point{1.0, 2.0, 3.0};
    constexpr cad::Vector3 offset{4.0, -2.0, 1.0};
    constexpr cad::Point3 moved = point + offset;
    static_assert(moved == cad::Point3{5.0, 0.0, 4.0});
    static_assert(moved - point == offset);
    static_assert(point - offset == cad::Point3{-3.0, 4.0, 2.0});
    static_assert(offset * 2.0 == cad::Vector3{8.0, -4.0, 2.0});
    static_assert(0.5 * offset == cad::Vector3{2.0, -1.0, 0.5});

    static_assert(cad::dot(cad::Vector3{1.0, 2.0, 3.0}, cad::Vector3{4.0, 5.0, 6.0}) == 32.0);
    static_assert(cad::cross(cad::Vector3{1.0, 0.0, 0.0}, cad::Vector3{0.0, 1.0, 0.0}) ==
                  cad::Vector3{0.0, 0.0, 1.0});
    expect(nearly_equal(cad::length(cad::Vector3{3.0, 4.0, 12.0}), 13.0), "3D length");
    expect(nearly_equal(cad::distance(point, moved), std::sqrt(21.0)), "3D distance");

    constexpr cad::Point2 point_2d{1.0, 2.0};
    constexpr cad::Vector2 vector_2d{3.0, 4.0};
    static_assert(point_2d + vector_2d == cad::Point2{4.0, 6.0});
    static_assert(cad::cross(cad::Vector2{1.0, 0.0}, cad::Vector2{0.0, 1.0}) == 1.0);
    expect(nearly_equal(cad::length(vector_2d), 5.0), "2D length");
}

void test_normalization() {
    const auto unit = cad::normalized(cad::Vector3{3.0, 0.0, 4.0});
    expect(unit.has_value(), "normalize regular vector");
    if (unit) {
        expect_vector(*unit, {0.6, 0.0, 0.8}, "normalized vector value");
        expect(nearly_equal(cad::length(*unit), 1.0), "normalized vector length");
    }

    const double high = std::numeric_limits<double>::max() / 2.0;
    const auto high_unit = cad::normalized(cad::Vector3{high, high, 0.0});
    expect(high_unit.has_value(), "normalize high-range vector without overflow");
    if (high_unit) {
        expect(nearly_equal(cad::length(*high_unit), 1.0), "high-range unit vector length");
    }

    expect(!cad::normalized(cad::Vector3{}), "reject zero vector normalization");
    const double subnormal = std::numeric_limits<double>::denorm_min();
    const auto subnormal_unit = cad::normalized(cad::Vector3{subnormal, subnormal, 0.0});
    expect(subnormal_unit.has_value(), "normalize subnormal vector");
    if (subnormal_unit) {
        expect(nearly_equal(cad::length(*subnormal_unit), 1.0), "subnormal unit vector length");
    }
    expect(
        !cad::normalized(cad::Vector3{std::numeric_limits<double>::infinity(), 0.0, 0.0}),
        "reject non-finite vector normalization"
    );
}

void test_interval() {
    const auto interval = cad::Interval::from_bounds(-2.0, 4.0);
    expect(interval.has_value(), "create valid interval");
    if (interval) {
        expect(nearly_equal(interval->length(), 6.0), "interval length");
        expect(interval->contains(-2.0) && interval->contains(4.0), "interval includes endpoints");
        expect(!interval->contains(4.1), "interval excludes outside value");
    }
    expect(!cad::Interval::from_bounds(2.0, 1.0), "reject reversed interval");
    expect(
        !cad::Interval::from_bounds(0.0, std::numeric_limits<double>::infinity()),
        "reject non-finite interval"
    );
}

void test_ray() {
    const auto ray = cad::Ray3::from_origin_direction({1.0, 2.0, 3.0}, {0.0, 0.0, 5.0});
    expect(ray.has_value(), "create ray");
    if (ray) {
        expect_vector(ray->direction(), {0.0, 0.0, 1.0}, "ray direction is normalized");
        expect_point(ray->at(4.0), {1.0, 2.0, 7.0}, "point along ray");
    }
    expect(!cad::Ray3::from_origin_direction({}, {}), "reject zero ray direction");
    expect(
        !cad::Ray3::from_origin_direction(
            {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
            {1.0, 0.0, 0.0}
        ),
        "reject non-finite ray origin"
    );
}

void test_plane() {
    const auto plane = cad::Plane::from_point_normal({0.0, 2.0, 0.0}, {0.0, 3.0, 0.0});
    expect(plane.has_value(), "create plane from point and normal");
    if (plane) {
        expect_vector(plane->normal(), {0.0, 1.0, 0.0}, "plane normal is normalized");
        expect(nearly_equal(plane->signed_distance({1.0, 5.0, 3.0}), 3.0), "plane distance");
        expect_point(plane->project({1.0, 5.0, 3.0}), {1.0, 2.0, 3.0}, "plane projection");
    }

    const auto points_plane = cad::Plane::from_points(
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0}
    );
    expect(points_plane.has_value(), "create plane from three points");
    if (points_plane) {
        expect_vector(points_plane->normal(), {0.0, 0.0, 1.0}, "three-point plane normal");
    }
    expect(
        !cad::Plane::from_points({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}),
        "reject collinear plane points"
    );
    expect(
        !cad::Plane::from_point_normal(
            {
                std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max(),
                0.0
            },
            {1.0, 1.0, 0.0}
        ),
        "reject unrepresentable plane offset"
    );
}

void test_bounds() {
    cad::Aabb3 bounds;
    expect(bounds.empty(), "new bounds are empty");
    expect(!bounds.minimum() && !bounds.maximum(), "empty bounds have no corners");
    expect(!bounds.center() && !bounds.extent(), "empty bounds have no size");
    expect(
        !bounds.expand({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}),
        "reject non-finite bounds point"
    );
    expect(bounds.expand({-1.0, 2.0, 3.0}), "expand bounds with first point");
    expect(bounds.expand({4.0, -2.0, 8.0}), "expand bounds with second point");
    expect_point(*bounds.minimum(), {-1.0, -2.0, 3.0}, "bounds minimum");
    expect_point(*bounds.maximum(), {4.0, 2.0, 8.0}, "bounds maximum");
    expect_point(*bounds.center(), {1.5, 0.0, 5.5}, "bounds center");
    expect_vector(*bounds.extent(), {5.0, 4.0, 5.0}, "bounds extent");
    expect(bounds.contains({0.0, 0.0, 4.0}), "bounds contain interior point");
    expect(!bounds.contains({0.0, 3.0, 4.0}), "bounds exclude outside point");

    cad::Aabb3 extreme_bounds;
    expect(extreme_bounds.expand({std::numeric_limits<double>::lowest(), 0.0, 0.0}),
           "expand extreme bounds minimum");
    expect(extreme_bounds.expand({std::numeric_limits<double>::max(), 0.0, 0.0}),
           "expand extreme bounds maximum");
    expect_point(*extreme_bounds.center(), {0.0, 0.0, 0.0}, "extreme bounds center is finite");
    expect(!extreme_bounds.extent(), "unrepresentable bounds extent is absent");

    constexpr std::array points{
        cad::Point3{-3.0, 1.0, 2.0},
        cad::Point3{6.0, 4.0, -1.0}
    };
    const auto from_points = cad::Aabb3::from_points(points);
    expect(from_points.has_value(), "create bounds from point span");
    expect(!cad::Aabb3::from_points(std::span<const cad::Point3>{}), "reject empty point span");
}

void test_transforms() {
    const cad::AffineTransform3 identity;
    expect_point(identity.transform_point({1.0, 2.0, 3.0}), {1.0, 2.0, 3.0}, "identity point");

    const auto translation = cad::AffineTransform3::translation({5.0, -2.0, 1.0});
    const auto scale = cad::AffineTransform3::scale({2.0, 3.0, 4.0});
    expect(translation.has_value() && scale.has_value(), "create translation and scale");
    if (translation && scale) {
        expect_point(translation->transform_point({1.0, 2.0, 3.0}), {6.0, 0.0, 4.0},
                     "translate point");
        expect_vector(translation->transform_vector({1.0, 2.0, 3.0}), {1.0, 2.0, 3.0},
                      "translation does not affect vector");
        expect_point(scale->transform_point({1.0, 2.0, 3.0}), {2.0, 6.0, 12.0}, "scale point");
        expect_point((*translation * *scale).transform_point({1.0, 1.0, 1.0}),
                     {7.0, 1.0, 5.0}, "transform composition order");
    }

    const auto rotation = cad::AffineTransform3::rotation(
        {0.0, 0.0, 1.0},
        std::numbers::pi / 2.0
    );
    expect(rotation.has_value(), "create rotation");
    if (rotation) {
        expect_point(rotation->transform_point({1.0, 0.0, 0.0}), {0.0, 1.0, 0.0}, "rotate point");
    }

    const auto mirror_plane = cad::Plane::from_point_normal({2.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    expect(mirror_plane.has_value(), "create reflection plane");
    if (mirror_plane) {
        expect_point(cad::AffineTransform3::reflection(*mirror_plane).transform_point({5.0, 1.0, 0.0}),
                     {-1.0, 1.0, 0.0}, "reflect point across offset plane");
    }

    expect(!cad::AffineTransform3::rotation({}, 1.0), "reject zero rotation axis");
    expect(
        !cad::AffineTransform3::rotation(
            {1.0, 0.0, 0.0},
            std::numeric_limits<double>::quiet_NaN()
        ),
        "reject non-finite rotation angle"
    );
    expect(
        !cad::AffineTransform3::scale(
            {1.0, std::numeric_limits<double>::infinity(), 1.0}
        ),
        "reject non-finite scale"
    );
    expect(
        !cad::AffineTransform3::translation(
            {std::numeric_limits<double>::infinity(), 0.0, 0.0}
        ),
        "reject non-finite translation"
    );
}

} // namespace

int main() {
    test_point_and_vector_operations();
    test_normalization();
    test_interval();
    test_ray();
    test_plane();
    test_bounds();
    test_transforms();

    if (failures != 0) {
        std::cerr << failures << " test assertion(s) failed\n";
        return 1;
    }
    return 0;
}
