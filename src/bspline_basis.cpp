#include "bspline_basis.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <utility>
#include <vector>

namespace cad {
namespace {

std::expected<void, BSplineBasisError> validate_inputs(
    std::size_t control_count,
    std::size_t degree,
    std::span<const double> knots,
    double parameter,
    std::size_t derivative_order
) {
    if (control_count == 0) {
        return std::unexpected(BSplineBasisError::invalid_control_count);
    }
    if (degree >= control_count ||
        control_count > std::numeric_limits<std::size_t>::max() - degree - 1) {
        return std::unexpected(BSplineBasisError::degree_out_of_range);
    }
    if (knots.size() != control_count + degree + 1) {
        return std::unexpected(BSplineBasisError::knot_count_mismatch);
    }
    const std::size_t count = degree + 1;
    const std::size_t vector_max = std::vector<long double>{}.max_size();
    const auto ptrdiff_max = static_cast<std::size_t>(
        std::numeric_limits<std::ptrdiff_t>::max()
    );
    const bool result_count_overflows =
        derivative_order == std::numeric_limits<std::size_t>::max() ||
        derivative_order + 1 > std::numeric_limits<std::size_t>::max() / count;
    const std::size_t result_count = result_count_overflows
        ? 0
        : (derivative_order + 1) * count;
    if (control_count > ptrdiff_max - 1 || degree > ptrdiff_max ||
        result_count_overflows ||
        (derivative_order > 0 && count > std::numeric_limits<std::size_t>::max() / count) ||
        (derivative_order > 0 && count > std::numeric_limits<std::size_t>::max() / 2) ||
        result_count > vector_max ||
        (derivative_order > 0 && count * count > vector_max) ||
        (derivative_order > 0 && 2 * count > vector_max)) {
        return std::unexpected(BSplineBasisError::numeric_range_not_supported);
    }

    std::size_t multiplicity = 1;
    for (std::size_t index = 0; index < knots.size(); ++index) {
        if (!std::isfinite(knots[index])) {
            return std::unexpected(BSplineBasisError::non_finite_knot);
        }
        if (index > 0) {
            if (knots[index] < knots[index - 1]) {
                return std::unexpected(BSplineBasisError::knots_not_nondecreasing);
            }
            multiplicity = knots[index] == knots[index - 1] ? multiplicity + 1 : 1;
            if (multiplicity > degree + 1) {
                return std::unexpected(BSplineBasisError::knot_multiplicity_exceeded);
            }
        }
    }

    const long double knot_range =
        static_cast<long double>(knots.back()) - static_cast<long double>(knots.front());
    if (!std::isfinite(knot_range)) {
        return std::unexpected(BSplineBasisError::numeric_range_not_supported);
    }

    const double domain_start = knots[degree];
    const double domain_end = knots[control_count];
    if (domain_start >= domain_end) {
        return std::unexpected(BSplineBasisError::empty_parameter_domain);
    }
    if (!std::isfinite(parameter) || parameter < domain_start || parameter > domain_end) {
        return std::unexpected(BSplineBasisError::parameter_out_of_domain);
    }
    return {};
}

std::size_t find_span(
    double parameter,
    std::size_t control_count,
    std::size_t degree,
    std::span<const double> knots
) {
    if (parameter == knots[control_count]) {
        return control_count - 1;
    }
    const auto first_larger = std::upper_bound(
        knots.begin() + static_cast<std::ptrdiff_t>(degree),
        knots.begin() + static_cast<std::ptrdiff_t>(control_count + 1),
        parameter
    );
    return static_cast<std::size_t>(std::distance(knots.begin(), first_larger) - 1);
}

void evaluate_values(
    std::size_t span,
    double parameter,
    std::size_t degree,
    std::span<const double> knots,
    std::span<long double> values
) {
    constexpr std::size_t fixed_count = 4;
    std::array<long double, fixed_count> fixed_left{};
    std::array<long double, fixed_count> fixed_right{};
    std::vector<long double> dynamic_left;
    std::vector<long double> dynamic_right;
    if (degree + 1 > fixed_count) {
        dynamic_left.resize(degree + 1);
        dynamic_right.resize(degree + 1);
    }
    auto left = dynamic_left.empty()
        ? std::span<long double>(fixed_left).first(degree + 1)
        : std::span<long double>(dynamic_left);
    auto right = dynamic_right.empty()
        ? std::span<long double>(fixed_right).first(degree + 1)
        : std::span<long double>(dynamic_right);

    values[0] = 1.0L;
    for (std::size_t j = 1; j <= degree; ++j) {
        left[j] = static_cast<long double>(parameter) - knots[span + 1 - j];
        right[j] = static_cast<long double>(knots[span + j]) - parameter;
        long double saved = 0.0L;
        for (std::size_t r = 0; r < j; ++r) {
            const long double denominator = right[r + 1] + left[j - r];
            const long double term = denominator == 0.0L ? 0.0L : values[r] / denominator;
            values[r] = saved + right[r + 1] * term;
            saved = left[j - r] * term;
        }
        values[j] = saved;
    }
}

void evaluate_derivatives(
    std::size_t span,
    double parameter,
    std::size_t degree,
    std::span<const double> knots,
    std::size_t derivative_order,
    std::span<long double> result
) {
    const std::size_t count = degree + 1;
    const std::size_t computed_order = std::min(degree, derivative_order);

    constexpr std::size_t fixed_degree_count = 4;
    constexpr std::size_t fixed_matrix_count = fixed_degree_count * fixed_degree_count;
    std::array<long double, fixed_matrix_count> fixed_ndu{};
    std::array<long double, fixed_degree_count * 2> fixed_a{};
    std::array<long double, fixed_degree_count> fixed_left{};
    std::array<long double, fixed_degree_count> fixed_right{};
    std::vector<long double> dynamic_ndu;
    std::vector<long double> dynamic_a;
    std::vector<long double> dynamic_left;
    std::vector<long double> dynamic_right;
    if (count > fixed_degree_count) {
        dynamic_ndu.resize(count * count);
        dynamic_a.resize(2 * count);
        dynamic_left.resize(count);
        dynamic_right.resize(count);
    }

    auto ndu = dynamic_ndu.empty()
        ? std::span<long double>(fixed_ndu).first(count * count)
        : std::span<long double>(dynamic_ndu);
    auto a = dynamic_a.empty()
        ? std::span<long double>(fixed_a).first(2 * count)
        : std::span<long double>(dynamic_a);
    auto left = dynamic_left.empty()
        ? std::span<long double>(fixed_left).first(count)
        : std::span<long double>(dynamic_left);
    auto right = dynamic_right.empty()
        ? std::span<long double>(fixed_right).first(count)
        : std::span<long double>(dynamic_right);
    const auto matrix = [count](std::span<long double> values, std::size_t row, std::size_t column)
        -> long double& {
        return values[row * count + column];
    };

    matrix(ndu, 0, 0) = 1.0L;
    for (std::size_t j = 1; j <= degree; ++j) {
        left[j] = static_cast<long double>(parameter) - knots[span + 1 - j];
        right[j] = static_cast<long double>(knots[span + j]) - parameter;
        long double saved = 0.0L;
        for (std::size_t r = 0; r < j; ++r) {
            matrix(ndu, j, r) = right[r + 1] + left[j - r];
            const long double denominator = matrix(ndu, j, r);
            const long double term = denominator == 0.0L
                ? 0.0L
                : matrix(ndu, r, j - 1) / denominator;
            matrix(ndu, r, j) = saved + right[r + 1] * term;
            saved = left[j - r] * term;
        }
        matrix(ndu, j, j) = saved;
    }
    for (std::size_t j = 0; j <= degree; ++j) {
        result[j] = matrix(ndu, j, degree);
    }

    for (std::size_t r = 0; r <= degree; ++r) {
        std::size_t previous = 0;
        std::size_t current = 1;
        matrix(a, previous, 0) = 1.0L;
        for (std::size_t k = 1; k <= computed_order; ++k) {
            std::fill_n(a.begin() + static_cast<std::ptrdiff_t>(current * count), count, 0.0L);
            long double derivative = 0.0L;
            const std::ptrdiff_t rk = static_cast<std::ptrdiff_t>(r) -
                static_cast<std::ptrdiff_t>(k);
            const std::ptrdiff_t pk = static_cast<std::ptrdiff_t>(degree) -
                static_cast<std::ptrdiff_t>(k);

            if (r >= k) {
                const long double denominator = matrix(
                    ndu,
                    static_cast<std::size_t>(pk + 1),
                    static_cast<std::size_t>(rk)
                );
                matrix(a, current, 0) = denominator == 0.0L
                    ? 0.0L
                    : matrix(a, previous, 0) / denominator;
                derivative = matrix(a, current, 0) * matrix(
                    ndu,
                    static_cast<std::size_t>(rk),
                    static_cast<std::size_t>(pk)
                );
            }

            const std::ptrdiff_t first = rk >= -1 ? 1 : -rk;
            const std::ptrdiff_t last =
                static_cast<std::ptrdiff_t>(r) - 1 <= pk
                    ? static_cast<std::ptrdiff_t>(k) - 1
                    : static_cast<std::ptrdiff_t>(degree - r);
            for (std::ptrdiff_t j = first; j <= last; ++j) {
                const long double denominator = matrix(
                    ndu,
                    static_cast<std::size_t>(pk + 1),
                    static_cast<std::size_t>(rk + j)
                );
                matrix(a, current, static_cast<std::size_t>(j)) = denominator == 0.0L
                    ? 0.0L
                    : (matrix(a, previous, static_cast<std::size_t>(j)) -
                       matrix(a, previous, static_cast<std::size_t>(j - 1))) / denominator;
                derivative += matrix(a, current, static_cast<std::size_t>(j)) * matrix(
                    ndu,
                    static_cast<std::size_t>(rk + j),
                    static_cast<std::size_t>(pk)
                );
            }

            if (r <= static_cast<std::size_t>(pk)) {
                const long double denominator = matrix(ndu, static_cast<std::size_t>(pk + 1), r);
                matrix(a, current, k) = denominator == 0.0L
                    ? 0.0L
                    : -matrix(a, previous, k - 1) / denominator;
                derivative += matrix(a, current, k) * matrix(ndu, r, static_cast<std::size_t>(pk));
            }
            result[k * count + r] = derivative;
            std::swap(previous, current);
        }
    }

    long double factor = static_cast<long double>(degree);
    for (std::size_t order = 1; order <= computed_order; ++order) {
        for (std::size_t j = 0; j <= degree; ++j) {
            result[order * count + j] *= factor;
        }
        factor *= static_cast<long double>(degree - order);
    }
}

} // namespace

BSplineBasisEvaluation::BSplineBasisEvaluation(
    std::size_t first_control_point,
    std::size_t degree,
    std::size_t derivative_order
)
    : m_first_control_point(first_control_point),
      m_degree(degree),
      m_derivative_order(derivative_order),
      m_value_count((degree + 1) * (derivative_order + 1)),
      m_dynamic_values(m_value_count > fixed_capacity ? m_value_count : 0, 0.0L) {}

BSplineBasisEvaluation::BSplineBasisEvaluation(BSplineBasisEvaluation&& other) noexcept
    : m_first_control_point(other.m_first_control_point),
      m_degree(other.m_degree),
      m_derivative_order(other.m_derivative_order),
      m_value_count(other.m_value_count),
      m_fixed_values(other.m_fixed_values),
      m_dynamic_values(std::move(other.m_dynamic_values)) {
    other.reset_moved_from();
}

BSplineBasisEvaluation& BSplineBasisEvaluation::operator=(
    BSplineBasisEvaluation&& other
) noexcept {
    if (this == &other) {
        return *this;
    }
    m_first_control_point = other.m_first_control_point;
    m_degree = other.m_degree;
    m_derivative_order = other.m_derivative_order;
    m_value_count = other.m_value_count;
    m_fixed_values = other.m_fixed_values;
    m_dynamic_values = std::move(other.m_dynamic_values);
    other.reset_moved_from();
    return *this;
}

void BSplineBasisEvaluation::reset_moved_from() noexcept {
    m_first_control_point = 0;
    m_degree = 0;
    m_derivative_order = 0;
    m_value_count = 1;
    m_fixed_values.fill(0.0L);
    m_dynamic_values.clear();
}

std::expected<BSplineBasisEvaluation, BSplineBasisError> evaluate_bspline_basis(
    std::size_t control_count,
    std::size_t degree,
    std::span<const double> knots,
    double parameter,
    std::size_t derivative_order
) {
    if (const auto validation = validate_inputs(
            control_count,
            degree,
            knots,
            parameter,
            derivative_order
        ); !validation) {
        return std::unexpected(validation.error());
    }

    const std::size_t span = find_span(parameter, control_count, degree, knots);
    BSplineBasisEvaluation result(span - degree, degree, derivative_order);
    if (derivative_order == 0) {
        evaluate_values(span, parameter, degree, knots, result.mutable_values());
    } else {
        evaluate_derivatives(
            span,
            parameter,
            degree,
            knots,
            derivative_order,
            result.mutable_values()
        );
    }
    if (!std::ranges::all_of(result.values(), [](long double value) {
            return std::isfinite(value);
        })) {
        return std::unexpected(BSplineBasisError::numeric_range_not_supported);
    }
    return result;
}

} // namespace cad
