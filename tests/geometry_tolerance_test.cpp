#include "geometry_tolerance.h"

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

void test_policy_validation() {
    const auto policy = cad::GeometryTolerance::create(0.1, 0.01, 0.05, 0.001);
    expect(policy.has_value(), "accept valid tolerance policy");
    if (policy) {
        expect(policy->model().absolute() == 0.1, "store model tolerance");
        expect(policy->parameter().absolute() == 0.01, "store parameter tolerance");
        expect(policy->angular_radians() == 0.05, "store angular tolerance");
        expect(policy->model().relative() == 0.001, "store relative tolerance");
    }

    expect(!cad::GeometryTolerance::create(0.0, 0.01, 0.05, 0.001),
           "reject zero model tolerance");
    expect(!cad::GeometryTolerance::create(0.1, -0.01, 0.05, 0.001),
           "reject negative parameter tolerance");
    expect(!cad::GeometryTolerance::create(0.1, 0.01, 0.0, 0.001),
           "reject zero angular tolerance");
    expect(!cad::GeometryTolerance::create(0.1, 0.01, 0.05, -0.001),
           "reject negative relative tolerance");
    expect(!cad::GeometryTolerance::create(0.1, 0.01, 0.05, 1.1),
           "reject relative tolerance above one");
    expect(!cad::GeometryTolerance::create(0.1, 0.01, std::numbers::pi / 2.0, 0.001),
           "reject degenerate angular tolerance");
    expect(
        !cad::GeometryTolerance::create(
            std::numeric_limits<double>::infinity(),
            0.01,
            0.05,
            0.001
        ),
        "reject non-finite tolerance"
    );
}

void test_scalar_comparison_boundaries() {
    const auto tolerance = cad::ScalarTolerance::create(0.25, 0.0);
    expect(tolerance.has_value(), "create scalar tolerance");
    if (!tolerance) {
        return;
    }

    expect(cad::approximately_equal(10.0, 10.25, *tolerance),
           "absolute comparison includes boundary");
    expect(cad::approximately_equal(10.0, 10.249, *tolerance),
           "absolute comparison accepts inside boundary");
    expect(!cad::approximately_equal(10.0, 10.251, *tolerance),
           "absolute comparison rejects outside boundary");
    expect(!cad::approximately_equal(
               std::numeric_limits<double>::infinity(),
               std::numeric_limits<double>::infinity(),
               *tolerance
           ),
           "non-finite values do not compare equal");

    const auto relative = cad::ScalarTolerance::create(0.01, 0.1);
    expect(relative.has_value(), "create relative scalar tolerance");
    if (relative) {
        constexpr double relative_boundary = 100.0 + (0.01 + 0.1 * 100.0) / (1.0 - 0.1);
        expect(cad::approximately_equal(100.0, relative_boundary - 1e-10, *relative),
               "relative comparison accepts immediately inside boundary");
        expect(!cad::approximately_equal(100.0, relative_boundary + 1e-10, *relative),
               "relative comparison rejects immediately outside boundary");
        expect(cad::near_zero(1.01, 10.0, *relative),
               "near-zero comparison includes scaled boundary");
        expect(!cad::near_zero(1.02, 10.0, *relative),
               "near-zero comparison rejects outside scaled boundary");
        expect(!cad::near_zero(0.0, -1.0, *relative),
               "near-zero comparison rejects invalid scale");
    }
}

void test_model_and_parameter_boundaries() {
    const auto policy = cad::GeometryTolerance::create(0.125, 0.0625, 0.05, 0.0);
    expect(policy.has_value(), "create boundary-test policy");
    if (!policy) {
        return;
    }

    expect(cad::points_near(cad::Point3{}, cad::Point3{0.125, 0.0, 0.0}, *policy),
           "model comparison includes boundary");
    expect(cad::points_near(cad::Point3{}, cad::Point3{0.124, 0.0, 0.0}, *policy),
           "model comparison accepts inside boundary");
    expect(!cad::points_near(cad::Point3{}, cad::Point3{0.126, 0.0, 0.0}, *policy),
           "model comparison rejects outside boundary");
    expect(!cad::points_near(cad::Point3{}, cad::Point3{0.09, 0.09, 0.0}, *policy),
           "model comparison uses Euclidean separation");
    expect(cad::points_near(
               cad::Point3{1e6, 1e6, 1e6},
               cad::Point3{1e6 + 0.0625, 1e6, 1e6},
               *policy
           ),
           "model comparison is independent of coordinate magnitude");
    const auto relative_policy = cad::GeometryTolerance::create(0.125, 0.0625, 0.05, 0.5);
    expect(relative_policy.has_value(), "create relative model policy");
    if (relative_policy) {
        expect(!cad::points_near(
                   cad::Point3{1e6, 1e6, 1e6},
                   cad::Point3{1e6 + 0.25, 1e6, 1e6},
                   *relative_policy
               ),
               "relative tolerance does not expand point coincidence at large coordinates");
    }
    expect(cad::points_near(cad::Point2{}, cad::Point2{0.1, 0.0}, *policy),
           "2D model comparison uses model tolerance");
    expect(!cad::points_near(
               cad::Point3{},
               cad::Point3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
               *policy
           ),
           "model comparison rejects non-finite point");

    expect(cad::parameters_near(0.5, 0.5625, *policy),
           "parameter comparison includes boundary");
    expect(cad::parameters_near(0.5, 0.5624, *policy),
           "parameter comparison accepts inside boundary");
    expect(!cad::parameters_near(0.5, 0.5626, *policy),
           "parameter comparison rejects outside boundary");
}

void test_angular_boundaries() {
    const auto policy = cad::GeometryTolerance::create(0.1, 0.01, 0.1, 0.0);
    expect(policy.has_value(), "create angular-test policy");
    if (!policy) {
        return;
    }

    const auto direction_at = [](double angle) {
        return cad::Vector3{std::cos(angle), std::sin(angle), 0.0};
    };
    const cad::Vector3 reference{1.0, 0.0, 0.0};
    expect(cad::same_direction(reference, direction_at(0.1 - 1e-10), *policy),
           "angular comparison accepts immediately inside boundary");
    expect(!cad::same_direction(reference, direction_at(0.1 + 1e-10), *policy),
           "angular comparison rejects immediately outside boundary");
    expect(!cad::same_direction(reference, -reference, *policy),
           "opposite vectors do not have the same direction");
    expect(cad::parallel(reference, -reference, *policy),
           "opposite vectors are parallel");
    expect(!cad::parallel(reference, {}, *policy),
           "zero vector is not parallel");
    expect(cad::same_direction(
               cad::Vector2{1.0, 0.0},
               cad::Vector2{std::cos(0.05), std::sin(0.05)},
               *policy
           ),
           "2D angular comparison uses angular tolerance");
    expect(!cad::same_direction(
               reference,
               {std::numeric_limits<double>::infinity(), 0.0, 0.0},
               *policy
           ),
           "angular comparison rejects non-finite vector");
}

void test_defaults() {
    const cad::GeometryTolerance defaults = cad::GeometryTolerance::defaults();
    expect(defaults.model().absolute() == 1e-9, "default model tolerance");
    expect(defaults.parameter().absolute() == 1e-12, "default parameter tolerance");
    expect(defaults.angular_radians() == 1e-9, "default angular tolerance");
    expect(defaults.model().relative() == 1e-12, "default relative tolerance");

    const cad::Vector3 reference{1.0, 0.0, 0.0};
    const auto direction_at = [](double angle) {
        return cad::Vector3{std::cos(angle), std::sin(angle), 0.0};
    };
    expect(cad::same_direction(reference, direction_at(0.5e-9), defaults),
           "default angular tolerance accepts smaller angle");
    expect(!cad::same_direction(reference, direction_at(2e-9), defaults),
           "default angular tolerance rejects larger angle");
}

} // namespace

int main() {
    test_policy_validation();
    test_scalar_comparison_boundaries();
    test_model_and_parameter_boundaries();
    test_angular_boundaries();
    test_defaults();

    if (failures != 0) {
        std::cerr << failures << " test assertion(s) failed\n";
        return 1;
    }
    return 0;
}
