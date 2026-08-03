#include "nurbs_surface.h"

#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

namespace {

int failures = 0;

void expect(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

bool nearly_equal(double lhs, double rhs, double tolerance = 1e-12) {
    return std::abs(lhs - rhs) <= tolerance;
}

std::vector<ControlPoint> make_grid(std::size_t u_count, std::size_t v_count) {
    std::vector<ControlPoint> points;
    points.reserve(u_count * v_count);
    for (std::size_t u = 0; u < u_count; ++u) {
        for (std::size_t v = 0; v < v_count; ++v) {
            points.push_back({{
                static_cast<double>(u),
                static_cast<double>(v),
                static_cast<double>(u + 2 * v)
            }, 1.0});
        }
    }
    return points;
}

void expect_point(
    const std::expected<Point3D, CadError>& result,
    Point3D expected,
    std::string_view message,
    double tolerance = 1e-12
) {
    expect(result.has_value(), message);
    if (!result) {
        return;
    }
    expect(nearly_equal(result->x, expected.x, tolerance), message);
    expect(nearly_equal(result->y, expected.y, tolerance), message);
    expect(nearly_equal(result->z, expected.z, tolerance), message);
}

void expect_vector(
    cad::Vector3 actual,
    cad::Vector3 expected,
    std::string_view message,
    double tolerance = 1e-12
) {
    expect(nearly_equal(actual.x, expected.x, tolerance), message);
    expect(nearly_equal(actual.y, expected.y, tolerance), message);
    expect(nearly_equal(actual.z, expected.z, tolerance), message);
}

void expect_creation_error(
    std::expected<std::unique_ptr<NurbsSurface>, NurbsSurfaceError> result,
    NurbsSurfaceErrorCode code,
    std::optional<NurbsParameterDirection> direction,
    std::size_t index,
    std::string_view message
) {
    expect(!result.has_value(), message);
    if (result) {
        return;
    }
    expect(result.error().code == code, message);
    expect(result.error().direction == direction, message);
    expect(result.error().index == index, message);
}

void test_open_uniform_knots() {
    expect(
        NurbsSurface::make_open_uniform_knots(4, 2) ==
            std::vector<double>({0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0}),
        "quadratic open-uniform knots"
    );
    expect(
        NurbsSurface::make_open_uniform_knots(5, 1) ==
            std::vector<double>({0.0, 0.0, 0.25, 0.5, 0.75, 1.0, 1.0}),
        "linear open-uniform knots"
    );
    expect(NurbsSurface::make_open_uniform_knots(0, 0).empty(), "zero control count");
    expect(NurbsSurface::make_open_uniform_knots(2, 2).empty(), "degree at control count");
}

void test_default_degrees_and_planar_evaluation() {
    auto result = NurbsSurface::create(5, 4, make_grid(5, 4));
    expect(result.has_value(), "create a default-degree surface");
    if (!result) {
        return;
    }
    expect((*result)->u_degree() == 3, "default U degree is capped at three");
    expect((*result)->v_degree() == 3, "default V degree is capped at three");

    for (const auto [u, v] : std::vector<std::pair<double, double>>{
             {0.0, 0.0}, {0.2, 0.7}, {0.5, 0.5}, {1.0, 1.0}
         }) {
        const auto point = (*result)->evaluate(u, v);
        expect(point.has_value(), "evaluate a planar surface");
        if (point) {
            expect(nearly_equal(point->z, point->x + 2.0 * point->y), "preserve plane");
        }
    }
}

void test_bilinear_endpoints_and_midpoint() {
    std::vector<ControlPoint> points{
        {{0.0, 0.0, 0.0}, 1.0},
        {{0.0, 2.0, 0.0}, 1.0},
        {{2.0, 0.0, 2.0}, 1.0},
        {{2.0, 2.0, 2.0}, 1.0},
    };
    auto result = NurbsSurface::create(2, 2, std::move(points));
    expect(result.has_value(), "create a valid bilinear surface");
    if (!result) {
        return;
    }

    expect_point((*result)->evaluate(0.0, 0.0), {0.0, 0.0, 0.0}, "lower endpoint");
    expect_point((*result)->evaluate(1.0, 1.0), {2.0, 2.0, 2.0}, "upper endpoint");
    expect_point((*result)->evaluate(0.5, 0.5), {1.0, 1.0, 1.0}, "bilinear midpoint");
}

void test_rational_weight_influence() {
    std::vector<ControlPoint> points{
        {{0.0, 0.0, 0.0}, 1.0},
        {{0.0, 1.0, 0.0}, 1.0},
        {{1.0, 0.0, 0.0}, 1.0},
        {{1.0, 1.0, 0.0}, 2.0},
    };
    auto result = NurbsSurface::create(2, 2, std::move(points));
    expect(result.has_value(), "create a rational bilinear surface");
    if (result) {
        expect_point((*result)->evaluate(0.5, 0.5), {0.6, 0.6, 0.0}, "rational midpoint");
    }
}

void test_custom_domains_and_bounds() {
    auto result = NurbsSurface::create(
        2,
        2,
        1,
        1,
        make_grid(2, 2),
        {-2.0, -2.0, 4.0, 4.0},
        {10.0, 10.0, 20.0, 20.0}
    );
    expect(result.has_value(), "create a surface with custom domains");
    if (!result) {
        return;
    }

    expect((*result)->u_domain() == std::pair(-2.0, 4.0), "custom U domain");
    expect((*result)->v_domain() == std::pair(10.0, 20.0), "custom V domain");
    expect_point((*result)->evaluate(1.0, 15.0), {0.5, 0.5, 1.5}, "custom-domain midpoint");

    for (const auto [u, v] : std::vector<std::pair<double, double>>{
             {-2.0001, 15.0}, {4.0001, 15.0}, {1.0, 9.9999}, {1.0, 20.0001},
             {std::numeric_limits<double>::quiet_NaN(), 15.0},
             {1.0, std::numeric_limits<double>::infinity()}
         }) {
        const auto point = (*result)->evaluate(u, v);
        expect(!point && point.error() == CadError::OutOfBounds, "reject out-of-domain parameter");
    }
}

void test_repeated_internal_knot() {
    auto result = NurbsSurface::create(
        5,
        2,
        2,
        1,
        make_grid(5, 2),
        {0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0},
        {0.0, 0.0, 1.0, 1.0}
    );
    expect(result.has_value(), "accept an internal knot repeated to the degree");
    if (result) {
        const auto point = (*result)->evaluate(0.5, 0.5);
        expect(point.has_value(), "evaluate exactly on a repeated knot");
        if (point) {
            expect(std::isfinite(point->x) && std::isfinite(point->y) && std::isfinite(point->z),
                   "repeated-knot result is finite");
        }
    }
}

void test_degree_ranges() {
    auto linear = NurbsSurface::create(3, 3, 1, 1, make_grid(3, 3));
    expect(linear.has_value(), "accept degree one");

    auto maximum = NurbsSurface::create(4, 3, 3, 2, make_grid(4, 3));
    expect(maximum.has_value(), "accept degree equal to control count minus one");
    if (maximum) {
        expect((*maximum)->evaluate(0.37, 0.61).has_value(), "evaluate maximum-degree surface");
    }
}

void test_control_net_validation_errors() {
    expect_creation_error(
        NurbsSurface::create(0, 2, {}),
        NurbsSurfaceErrorCode::invalid_control_net_dimensions,
        std::nullopt,
        NurbsSurfaceError::no_index,
        "reject zero control-net dimension"
    );
    expect_creation_error(
        NurbsSurface::create(2, 2, make_grid(2, 1)),
        NurbsSurfaceErrorCode::control_point_count_mismatch,
        std::nullopt,
        NurbsSurfaceError::no_index,
        "reject mismatched control-point count"
    );

    auto non_finite = make_grid(2, 2);
    non_finite[2].position.y = std::numeric_limits<double>::quiet_NaN();
    expect_creation_error(
        NurbsSurface::create(2, 2, std::move(non_finite)),
        NurbsSurfaceErrorCode::non_finite_control_point,
        std::nullopt,
        2,
        "reject non-finite control point"
    );

    auto zero_weight = make_grid(2, 2);
    zero_weight[1].weight = 0.0;
    expect_creation_error(
        NurbsSurface::create(2, 2, std::move(zero_weight)),
        NurbsSurfaceErrorCode::non_positive_weight,
        std::nullopt,
        1,
        "reject zero weight"
    );
}

void test_knot_validation_errors() {
    expect_creation_error(
        NurbsSurface::create(2, 2, 2, 1, make_grid(2, 2)),
        NurbsSurfaceErrorCode::degree_out_of_range,
        NurbsParameterDirection::u,
        NurbsSurfaceError::no_index,
        "reject out-of-range U degree"
    );
    expect_creation_error(
        NurbsSurface::create(2, 2, 1, 2, make_grid(2, 2)),
        NurbsSurfaceErrorCode::degree_out_of_range,
        NurbsParameterDirection::v,
        NurbsSurfaceError::no_index,
        "reject out-of-range V degree"
    );
    expect_creation_error(
        NurbsSurface::create(2, 2, 1, 1, make_grid(2, 2), {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0, 1.0}),
        NurbsSurfaceErrorCode::knot_count_mismatch,
        NurbsParameterDirection::u,
        NurbsSurfaceError::no_index,
        "reject incorrect knot count"
    );
    expect_creation_error(
        NurbsSurface::create(
            2, 2, 1, 1, make_grid(2, 2),
            {0.0, 0.0, std::numeric_limits<double>::infinity(), 1.0},
            {0.0, 0.0, 1.0, 1.0}
        ),
        NurbsSurfaceErrorCode::non_finite_knot,
        NurbsParameterDirection::u,
        2,
        "reject non-finite knot"
    );
    expect_creation_error(
        NurbsSurface::create(
            2, 2, 1, 1, make_grid(2, 2),
            {0.0, 0.0, 1.0, 1.0},
            {0.0, 0.5, 0.25, 1.0}
        ),
        NurbsSurfaceErrorCode::knots_not_nondecreasing,
        NurbsParameterDirection::v,
        2,
        "reject descending knots"
    );
    expect_creation_error(
        NurbsSurface::create(
            3, 2, 1, 1, make_grid(3, 2),
            {0.0, 0.0, 0.0, 1.0, 1.0},
            {0.0, 0.0, 1.0, 1.0}
        ),
        NurbsSurfaceErrorCode::knot_multiplicity_exceeded,
        NurbsParameterDirection::u,
        2,
        "reject excessive knot multiplicity"
    );
    expect_creation_error(
        NurbsSurface::create(
            4, 2, 2, 1, make_grid(4, 2),
            {-1.0, 0.0, 1.0, 1.0, 1.0, 2.0, 3.0},
            {0.0, 0.0, 1.0, 1.0}
        ),
        NurbsSurfaceErrorCode::empty_parameter_domain,
        NurbsParameterDirection::u,
        NurbsSurfaceError::no_index,
        "reject empty parameter domain"
    );
}

void test_high_dynamic_range_values() {
    constexpr double high = std::numeric_limits<double>::max() / 16.0;
    constexpr double low = std::numeric_limits<double>::min();
    std::vector<ControlPoint> points{
        {{high, 0.0, 0.0}, low},
        {{high, 1.0, 0.0}, 1.0},
        {{high, 0.0, 1.0}, 1.0},
        {{high, 1.0, 1.0}, high},
    };
    auto result = NurbsSurface::create(2, 2, std::move(points));
    expect(result.has_value(), "accept supported high dynamic-range values");
    if (result) {
        const auto point = (*result)->evaluate(0.5, 0.5);
        expect(point.has_value(), "evaluate supported high dynamic-range values");
        if (point) {
            expect(std::isfinite(point->x) && std::isfinite(point->y) && std::isfinite(point->z),
                   "high dynamic-range result is finite");
        }
    }
}

void test_platform_dependent_range_guards() {
    if constexpr (std::numeric_limits<long double>::max() ==
                  std::numeric_limits<double>::max()) {
        auto large_weight = make_grid(2, 2);
        large_weight[0].weight = std::numeric_limits<double>::max();
        expect_creation_error(
            NurbsSurface::create(2, 2, std::move(large_weight)),
            NurbsSurfaceErrorCode::numeric_range_not_supported,
            std::nullopt,
            0,
            "reject weight outside accumulator range"
        );

        expect_creation_error(
            NurbsSurface::create(
                2,
                2,
                1,
                1,
                make_grid(2, 2),
                {
                    std::numeric_limits<double>::lowest(),
                    std::numeric_limits<double>::lowest(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max()
                },
                {0.0, 0.0, 1.0, 1.0}
            ),
            NurbsSurfaceErrorCode::knot_range_not_finite,
            NurbsParameterDirection::u,
            NurbsSurfaceError::no_index,
            "reject knot range outside accumulator range"
        );
    }
}

void test_planar_surface_derivatives_and_normals() {
    std::vector<ControlPoint> points{
        {{0.0, 0.0, 0.0}, 1.0},
        {{0.0, 2.0, 0.0}, 1.0},
        {{2.0, 0.0, 2.0}, 1.0},
        {{2.0, 2.0, 2.0}, 1.0},
    };
    auto result = NurbsSurface::create(2, 2, std::move(points));
    expect(result.has_value(), "create planar derivative surface");
    if (!result) {
        return;
    }

    const auto derivatives = (*result)->evaluate_derivatives(0.35, 0.65);
    expect(derivatives.has_value(), "evaluate planar surface derivatives");
    if (derivatives) {
        expect_vector(derivatives->u, {2.0, 0.0, 2.0}, "planar U derivative");
        expect_vector(derivatives->v, {0.0, 2.0, 0.0}, "planar V derivative");
        expect_vector(derivatives->uu, {}, "planar UU derivative");
        expect_vector(derivatives->uv, {}, "planar UV derivative");
        expect_vector(derivatives->vv, {}, "planar VV derivative");
        expect_point(
            (*result)->evaluate(0.35, 0.65),
            derivatives->position,
            "differential position matches point evaluation"
        );
    }

    const cad::GeometryTolerance tolerance = cad::GeometryTolerance::defaults();
    const cad::Vector3 expected_normal{-std::sqrt(0.5), 0.0, std::sqrt(0.5)};
    for (const auto [u, v] : std::vector<std::pair<double, double>>{
             {0.0, 0.0}, {0.2, 0.7}, {0.5, 0.5}, {1.0, 1.0}
         }) {
        const auto normal = (*result)->normal(u, v, tolerance);
        expect(normal.has_value(), "evaluate stable planar normal");
        if (normal) {
            expect_vector(*normal, expected_normal, "planar normal orientation");
            expect(nearly_equal(cad::length(*normal), 1.0), "planar normal has unit length");
        }
    }
}

void test_rational_derivatives_against_finite_differences() {
    std::vector<ControlPoint> points{
        {{0.0, 0.0, 0.0}, 1.0}, {{0.0, 1.0, 1.0}, 0.8}, {{0.0, 2.0, 0.0}, 1.0},
        {{1.0, 0.0, 1.0}, 1.2}, {{1.0, 1.0, 2.0}, 2.0}, {{1.0, 2.0, 1.0}, 1.1},
        {{2.0, 0.0, 0.0}, 1.0}, {{2.0, 1.0, 1.0}, 0.9}, {{2.0, 2.0, 0.0}, 1.0},
    };
    auto result = NurbsSurface::create(3, 3, 2, 2, std::move(points));
    expect(result.has_value(), "create rational derivative surface");
    if (!result) {
        return;
    }

    constexpr double u = 0.43;
    constexpr double v = 0.57;
    constexpr double step = 1e-4;
    const auto derivatives = (*result)->evaluate_derivatives(u, v);
    expect(derivatives.has_value(), "evaluate rational surface derivatives");
    if (!derivatives) {
        return;
    }

    const auto point = [&](double sample_u, double sample_v) {
        return (*result)->evaluate(sample_u, sample_v).value();
    };
    const Point3D center = point(u, v);
    const Point3D u_plus = point(u + step, v);
    const Point3D u_minus = point(u - step, v);
    const Point3D v_plus = point(u, v + step);
    const Point3D v_minus = point(u, v - step);
    const cad::Vector3 finite_u = (u_plus - u_minus) / (2.0 * step);
    const cad::Vector3 finite_v = (v_plus - v_minus) / (2.0 * step);
    const cad::Vector3 finite_uu =
        ((u_plus - center) + (u_minus - center)) / (step * step);
    const cad::Vector3 finite_vv =
        ((v_plus - center) + (v_minus - center)) / (step * step);
    const Point3D plus_plus = point(u + step, v + step);
    const Point3D plus_minus = point(u + step, v - step);
    const Point3D minus_plus = point(u - step, v + step);
    const Point3D minus_minus = point(u - step, v - step);
    const cad::Vector3 finite_uv =
        ((plus_plus - plus_minus) - (minus_plus - minus_minus)) / (4.0 * step * step);

    expect_vector(derivatives->u, finite_u, "rational U finite difference", 1e-6);
    expect_vector(derivatives->v, finite_v, "rational V finite difference", 1e-6);
    expect_vector(derivatives->uu, finite_uu, "rational UU finite difference", 1e-5);
    expect_vector(derivatives->uv, finite_uv, "rational UV finite difference", 1e-5);
    expect_vector(derivatives->vv, finite_vv, "rational VV finite difference", 1e-5);

    const auto normal = (*result)->normal(u, v, cad::GeometryTolerance::defaults());
    expect(normal.has_value(), "evaluate rational surface normal");
    if (normal) {
        expect(nearly_equal(cad::length(*normal), 1.0), "rational normal has unit length");
        expect(nearly_equal(cad::dot(*normal, derivatives->u), 0.0, 1e-12),
               "normal is perpendicular to U tangent");
        expect(nearly_equal(cad::dot(*normal, derivatives->v), 0.0, 1e-12),
               "normal is perpendicular to V tangent");
    }
}

void test_singular_normal_and_derivative_errors() {
    std::vector<ControlPoint> points(4, ControlPoint{{1.0, 2.0, 3.0}, 1.0});
    auto result = NurbsSurface::create(2, 2, std::move(points));
    expect(result.has_value(), "create geometrically singular surface");
    if (!result) {
        return;
    }

    const auto derivatives = (*result)->evaluate_derivatives(0.5, 0.5);
    expect(derivatives.has_value(), "singular surface still has differential result");
    if (derivatives) {
        expect_vector(derivatives->u, {}, "singular U derivative is zero");
        expect_vector(derivatives->v, {}, "singular V derivative is zero");
    }
    const auto normal = (*result)->normal(0.5, 0.5, cad::GeometryTolerance::defaults());
    expect(!normal && normal.error() == CadError::DegenerateSurface,
           "singular surface normal reports degeneracy");

    const auto outside = (*result)->evaluate_derivatives(-0.1, 0.5);
    expect(!outside && outside.error() == CadError::OutOfBounds,
           "derivatives reject out-of-domain parameter");
    const auto non_finite = (*result)->evaluate_derivatives(
        std::numeric_limits<double>::quiet_NaN(),
        0.5
    );
    expect(!non_finite && non_finite.error() == CadError::OutOfBounds,
           "derivatives reject non-finite parameter");
}

void test_one_sided_repeated_knot_derivatives() {
    std::vector<ControlPoint> points;
    for (const double x : {0.0, 1.0, 2.0, 4.0, 5.0}) {
        points.push_back({{x, 0.0, 0.0}, 1.0});
        points.push_back({{x, 1.0, 0.0}, 1.0});
    }
    auto result = NurbsSurface::create(
        5,
        2,
        2,
        1,
        std::move(points),
        {0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0},
        {0.0, 0.0, 1.0, 1.0}
    );
    expect(result.has_value(), "create repeated-knot derivative surface");
    if (!result) {
        return;
    }

    const auto derivatives = (*result)->evaluate_derivatives(0.5, 0.5);
    expect(derivatives.has_value(), "evaluate one-sided repeated-knot derivatives");
    if (derivatives) {
        expect_vector(derivatives->u, {8.0, 0.0, 0.0},
                      "internal knot uses right-hand U derivative");
        expect_vector(derivatives->uu, {-8.0, 0.0, 0.0},
                      "internal knot uses right-hand UU derivative");
    }
}

void test_normal_independent_of_second_derivative_range() {
    constexpr double domain = 1e-200;
    std::vector<ControlPoint> points{
        {{0.0, 0.0, 0.0}, 1.0}, {{0.0, 1.0, 0.0}, 1.0},
        {{1.0, 0.0, 0.0}, 1.0}, {{1.0, 1.0, 0.0}, 1.0},
        {{0.0, 0.0, 0.0}, 1.0}, {{0.0, 1.0, 0.0}, 1.0},
    };
    auto result = NurbsSurface::create(
        3,
        2,
        2,
        1,
        std::move(points),
        {0.0, 0.0, 0.0, domain, domain, domain},
        {0.0, 0.0, 1.0, 1.0}
    );
    expect(result.has_value(), "create high-curvature parameterized surface");
    if (!result) {
        return;
    }

    const auto derivatives = (*result)->evaluate_derivatives(domain * 0.25, 0.5);
    expect(!derivatives && derivatives.error() == CadError::DegenerateSurface,
           "unrepresentable second derivative is rejected");
    const auto normal = (*result)->normal(
        domain * 0.25,
        0.5,
        cad::GeometryTolerance::defaults()
    );
    expect(normal.has_value(), "normal does not depend on second derivative range");
    if (normal) {
        expect_vector(*normal, {0.0, 0.0, 1.0}, "high-curvature surface normal");
    }
}

void test_normal_angular_singularity_threshold() {
    const auto tolerance = cad::GeometryTolerance::create(1e-9, 1e-12, 0.01, 1e-12);
    expect(tolerance.has_value(), "create normal angular tolerance");
    if (!tolerance) {
        return;
    }

    const auto make_surface = [](double angle) {
        const cad::Vector3 v_direction{std::cos(angle), std::sin(angle), 0.0};
        return NurbsSurface::create(
            2,
            2,
            std::vector<ControlPoint>{
                {{0.0, 0.0, 0.0}, 1.0},
                {{v_direction.x, v_direction.y, 0.0}, 1.0},
                {{1.0, 0.0, 0.0}, 1.0},
                {{1.0 + v_direction.x, v_direction.y, 0.0}, 1.0},
            }
        );
    };

    auto below = make_surface(0.01 - 1e-5);
    auto above = make_surface(0.01 + 1e-5);
    expect(below.has_value() && above.has_value(), "create angular-threshold surfaces");
    if (below) {
        const auto normal = (*below)->normal(0.5, 0.5, *tolerance);
        expect(!normal && normal.error() == CadError::DegenerateSurface,
               "normal rejects tangents below angular threshold");
    }
    if (above) {
        const auto normal = (*above)->normal(0.5, 0.5, *tolerance);
        expect(normal.has_value(), "normal accepts tangents above angular threshold");
    }
}

} // namespace

int main() {
    test_open_uniform_knots();
    test_default_degrees_and_planar_evaluation();
    test_bilinear_endpoints_and_midpoint();
    test_rational_weight_influence();
    test_custom_domains_and_bounds();
    test_repeated_internal_knot();
    test_degree_ranges();
    test_control_net_validation_errors();
    test_knot_validation_errors();
    test_high_dynamic_range_values();
    test_platform_dependent_range_guards();
    test_planar_surface_derivatives_and_normals();
    test_rational_derivatives_against_finite_differences();
    test_singular_normal_and_derivative_errors();
    test_one_sided_repeated_knot_derivatives();
    test_normal_independent_of_second_derivative_range();
    test_normal_angular_singularity_threshold();

    if (failures != 0) {
        std::cerr << failures << " test assertion(s) failed\n";
        return 1;
    }
    return 0;
}
