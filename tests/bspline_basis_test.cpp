#include "bspline_basis.h"

#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <numeric>
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

bool nearly_equal(long double left, long double right, long double tolerance = 1e-14L) {
    return std::abs(left - right) <= tolerance;
}

long double sum(std::span<const long double> values) {
    return std::accumulate(values.begin(), values.end(), 0.0L);
}

void test_quadratic_bernstein_values_and_derivatives() {
    const std::vector<double> knots{0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    const auto result = cad::evaluate_bspline_basis(3, 2, knots, 0.25, 2);
    expect(result.has_value(), "evaluate quadratic Bernstein basis");
    if (!result) {
        return;
    }

    expect(result->first_control_point() == 0, "quadratic Bernstein first control point");
    expect(result->degree() == 2, "quadratic Bernstein degree");
    expect(result->derivative_order() == 2, "quadratic Bernstein derivative order");

    const auto values = result->derivative(0);
    const auto first = result->derivative(1);
    const auto second = result->derivative(2);
    expect(values.size() == 3 && first.size() == 3 && second.size() == 3,
           "quadratic derivative row sizes");
    expect(nearly_equal(values[0], 0.5625L), "quadratic first basis value");
    expect(nearly_equal(values[1], 0.375L), "quadratic second basis value");
    expect(nearly_equal(values[2], 0.0625L), "quadratic third basis value");
    expect(nearly_equal(first[0], -1.5L), "quadratic first derivative zero");
    expect(nearly_equal(first[1], 1.0L), "quadratic first derivative one");
    expect(nearly_equal(first[2], 0.5L), "quadratic first derivative two");
    expect(nearly_equal(second[0], 2.0L), "quadratic second derivative zero");
    expect(nearly_equal(second[1], -4.0L), "quadratic second derivative one");
    expect(nearly_equal(second[2], 2.0L), "quadratic second derivative two");
    expect(result->derivative(3).empty(), "unrequested derivative row is empty");
}

void test_partition_of_unity_and_derivative_sums() {
    const std::vector<double> knots{0.0, 0.0, 0.0, 0.25, 0.5, 0.75, 1.0, 1.0, 1.0};
    for (const double parameter : {0.0, 0.1, 0.25, 0.49, 0.5, 0.9, 1.0}) {
        const auto result = cad::evaluate_bspline_basis(6, 2, knots, parameter, 2);
        expect(result.has_value(), "evaluate partition-of-unity sample");
        if (!result) {
            continue;
        }
        expect(nearly_equal(sum(result->derivative(0)), 1.0L), "basis values sum to one");
        expect(nearly_equal(sum(result->derivative(1)), 0.0L),
               "first basis derivatives sum to zero");
        expect(nearly_equal(sum(result->derivative(2)), 0.0L),
               "second basis derivatives sum to zero");
        for (const long double value : result->derivative(0)) {
            expect(value >= 0.0L && value <= 1.0L, "basis value is in unit interval");
        }
    }
}

void test_local_support_index() {
    const std::vector<double> knots{0.0, 0.0, 0.0, 0.25, 0.5, 0.75, 1.0, 1.0, 1.0};
    const auto result = cad::evaluate_bspline_basis(6, 2, knots, 0.6);
    expect(result.has_value(), "evaluate local basis support");
    if (result) {
        expect(result->first_control_point() == 2, "local support starts at expected control point");
        expect(result->derivative(0).size() == 3, "only degree plus one values are returned");
    }
}

void test_domain_endpoints() {
    const std::vector<double> knots{0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0};
    const auto start = cad::evaluate_bspline_basis(4, 2, knots, 0.0);
    const auto end = cad::evaluate_bspline_basis(4, 2, knots, 1.0);
    expect(start.has_value() && end.has_value(), "evaluate both domain endpoints");
    if (start) {
        expect(start->first_control_point() == 0, "start endpoint support index");
        expect(nearly_equal(start->derivative(0)[0], 1.0L), "start endpoint interpolates first point");
        expect(nearly_equal(sum(start->derivative(0)), 1.0L), "start endpoint partition of unity");
    }
    if (end) {
        expect(end->first_control_point() == 1, "end endpoint support index");
        expect(nearly_equal(end->derivative(0)[2], 1.0L), "end endpoint interpolates last point");
        expect(nearly_equal(sum(end->derivative(0)), 1.0L), "end endpoint partition of unity");
    }
}

void test_repeated_internal_knot() {
    const std::vector<double> knots{0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0};
    const auto result = cad::evaluate_bspline_basis(5, 2, knots, 0.5, 2);
    expect(result.has_value(), "evaluate at repeated internal knot");
    if (!result) {
        return;
    }
    expect(nearly_equal(sum(result->derivative(0)), 1.0L),
           "repeated-knot basis values sum to one");
    expect(nearly_equal(sum(result->derivative(1)), 0.0L),
           "repeated-knot first derivatives sum to zero");
    expect(nearly_equal(result->derivative(0)[0], 1.0L),
           "repeated-knot right-hand basis value");
    expect(nearly_equal(result->derivative(1)[0], -4.0L),
           "repeated-knot right-hand first derivative");
    expect(nearly_equal(result->derivative(1)[1], 4.0L),
           "repeated-knot adjacent first derivative");
    expect(nearly_equal(result->derivative(2)[0], 8.0L),
           "repeated-knot right-hand second derivative");
    for (std::size_t order = 0; order <= 2; ++order) {
        for (const long double value : result->derivative(order)) {
            expect(std::isfinite(value), "repeated-knot basis result is finite");
        }
    }
}

void test_supported_degree_range() {
    const std::vector<double> quintic_knots{
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0
    };
    auto quintic = cad::evaluate_bspline_basis(6, 5, quintic_knots, 0.3, 2);
    expect(quintic.has_value(), "evaluate degree above fixed workspace range");
    if (quintic) {
        expect(nearly_equal(sum(quintic->derivative(0)), 1.0L), "quintic partition of unity");
        expect(nearly_equal(sum(quintic->derivative(1)), 0.0L), "quintic derivative sum");

        auto moved = std::move(*quintic);
        expect(nearly_equal(sum(moved.derivative(0)), 1.0L), "dynamic evaluation survives move");
        expect(quintic->derivative(0).size() == 1, "moved-from evaluation remains valid");
    }

    const std::vector<double> constant_knots{0.0, 0.5, 1.0, 1.5};
    const auto constant = cad::evaluate_bspline_basis(3, 0, constant_knots, 0.75, 2);
    expect(constant.has_value(), "evaluate degree-zero basis");
    if (constant) {
        expect(constant->first_control_point() == 1, "degree-zero support index");
        expect(nearly_equal(constant->derivative(0)[0], 1.0L), "degree-zero basis value");
        expect(nearly_equal(constant->derivative(1)[0], 0.0L), "degree-zero first derivative");
        expect(nearly_equal(constant->derivative(2)[0], 0.0L), "degree-zero second derivative");
    }
}

void test_validation_errors() {
    const std::vector<double> linear_knots{0.0, 0.0, 1.0, 1.0};
    expect(
        cad::evaluate_bspline_basis(0, 0, {}, 0.0).error() ==
            cad::BSplineBasisError::invalid_control_count,
        "reject zero control count"
    );
    expect(
        cad::evaluate_bspline_basis(2, 2, linear_knots, 0.5).error() ==
            cad::BSplineBasisError::degree_out_of_range,
        "reject out-of-range degree"
    );
    expect(
        cad::evaluate_bspline_basis(2, 1, std::span(linear_knots).first(3), 0.5).error() ==
            cad::BSplineBasisError::knot_count_mismatch,
        "reject wrong knot count"
    );
    expect(
        cad::evaluate_bspline_basis(
            2,
            1,
            std::vector<double>{0.0, 0.0, std::numeric_limits<double>::infinity(), 1.0},
            0.5
        ).error() == cad::BSplineBasisError::non_finite_knot,
        "reject non-finite knot"
    );
    expect(
        cad::evaluate_bspline_basis(2, 1, std::vector<double>{0.0, 0.5, 0.25, 1.0}, 0.5)
                .error() == cad::BSplineBasisError::knots_not_nondecreasing,
        "reject descending knots"
    );
    expect(
        cad::evaluate_bspline_basis(3, 1, std::vector<double>{0.0, 0.0, 0.0, 1.0, 1.0}, 0.5)
                .error() == cad::BSplineBasisError::knot_multiplicity_exceeded,
        "reject excessive multiplicity"
    );
    expect(
        cad::evaluate_bspline_basis(
            4,
            2,
            std::vector<double>{-1.0, 0.0, 1.0, 1.0, 1.0, 2.0, 3.0},
            1.0
        ).error() == cad::BSplineBasisError::empty_parameter_domain,
        "reject empty domain"
    );
    expect(
        cad::evaluate_bspline_basis(2, 1, linear_knots, -0.1).error() ==
            cad::BSplineBasisError::parameter_out_of_domain,
        "reject out-of-domain parameter"
    );
    expect(
        cad::evaluate_bspline_basis(
            2,
            1,
            linear_knots,
            0.5,
            std::numeric_limits<std::size_t>::max()
        ).error() == cad::BSplineBasisError::numeric_range_not_supported,
        "reject unsupported derivative storage size"
    );
}

} // namespace

int main() {
    test_quadratic_bernstein_values_and_derivatives();
    test_partition_of_unity_and_derivative_sums();
    test_local_support_index();
    test_domain_endpoints();
    test_repeated_internal_knot();
    test_supported_degree_range();
    test_validation_errors();

    if (failures != 0) {
        std::cerr << failures << " test assertion(s) failed\n";
        return 1;
    }
    return 0;
}
