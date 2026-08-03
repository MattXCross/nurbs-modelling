#pragma once

#include <array>
#include <cstddef>
#include <expected>
#include <span>
#include <vector>

namespace cad {

enum class BSplineBasisError {
    invalid_control_count,
    degree_out_of_range,
    knot_count_mismatch,
    non_finite_knot,
    knots_not_nondecreasing,
    knot_multiplicity_exceeded,
    empty_parameter_domain,
    parameter_out_of_domain,
    numeric_range_not_supported
};

class BSplineBasisEvaluation;

[[nodiscard]] std::expected<BSplineBasisEvaluation, BSplineBasisError> evaluate_bspline_basis(
    std::size_t control_count,
    std::size_t degree,
    std::span<const double> knots,
    double parameter,
    std::size_t derivative_order = 0
);

// At an internal knot, derivatives use the right-hand span. At the domain end,
// they use the left-hand span. If continuity is insufficient, the returned
// derivative is therefore one-sided rather than a claim of two-sided existence.
class BSplineBasisEvaluation {
public:
    BSplineBasisEvaluation(const BSplineBasisEvaluation&) = default;
    BSplineBasisEvaluation& operator=(const BSplineBasisEvaluation&) = default;
    BSplineBasisEvaluation(BSplineBasisEvaluation&& other) noexcept;
    BSplineBasisEvaluation& operator=(BSplineBasisEvaluation&& other) noexcept;

    [[nodiscard]] std::size_t first_control_point() const noexcept {
        return m_first_control_point;
    }
    [[nodiscard]] std::size_t degree() const noexcept { return m_degree; }
    [[nodiscard]] std::size_t derivative_order() const noexcept { return m_derivative_order; }

    // The returned view is invalidated when this evaluation is moved, assigned, or destroyed.
    [[nodiscard]] std::span<const long double> derivative(std::size_t order) const noexcept {
        if (order > m_derivative_order) {
            return {};
        }
        return values().subspan(order * (m_degree + 1), m_degree + 1);
    }

private:
    friend std::expected<BSplineBasisEvaluation, BSplineBasisError> evaluate_bspline_basis(
        std::size_t,
        std::size_t,
        std::span<const double>,
        double,
        std::size_t
    );

    BSplineBasisEvaluation(
        std::size_t first_control_point,
        std::size_t degree,
        std::size_t derivative_order
    );

    [[nodiscard]] std::span<long double> mutable_values() noexcept {
        if (m_dynamic_values.empty()) {
            return std::span<long double>(m_fixed_values).first(m_value_count);
        }
        return m_dynamic_values;
    }

    [[nodiscard]] std::span<const long double> values() const noexcept {
        if (m_dynamic_values.empty()) {
            return std::span<const long double>(m_fixed_values).first(m_value_count);
        }
        return m_dynamic_values;
    }

    void reset_moved_from() noexcept;

    static constexpr std::size_t fixed_capacity = 12;
    std::size_t m_first_control_point;
    std::size_t m_degree;
    std::size_t m_derivative_order;
    std::size_t m_value_count;
    std::array<long double, fixed_capacity> m_fixed_values{};
    std::vector<long double> m_dynamic_values;
};

} // namespace cad
