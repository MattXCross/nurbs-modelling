#include "nurbs_surface.h"
#include "core.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <expected>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace {

std::optional<NurbsSurfaceError> validate_control_net(
    size_t u_count,
    size_t v_count,
    const std::vector<ControlPoint>& points
) {
    if (u_count == 0 || v_count == 0 ||
        u_count > std::numeric_limits<size_t>::max() / v_count) {
        return NurbsSurfaceError{NurbsSurfaceErrorCode::invalid_control_net_dimensions};
    }

    if (points.size() != u_count * v_count) {
        return NurbsSurfaceError{NurbsSurfaceErrorCode::control_point_count_mismatch};
    }

    long double maximum_weight = 0.0L;
    long double maximum_homogeneous_coordinate = 0.0L;
    for (size_t index = 0; index < points.size(); ++index) {
        const ControlPoint& point = points[index];
        if (!std::isfinite(point.position.x) ||
            !std::isfinite(point.position.y) ||
            !std::isfinite(point.position.z) ||
            !std::isfinite(point.weight)) {
            return NurbsSurfaceError{
                NurbsSurfaceErrorCode::non_finite_control_point,
                std::nullopt,
                index
            };
        }
        if (point.weight <= 0.0) {
            return NurbsSurfaceError{
                NurbsSurfaceErrorCode::non_positive_weight,
                std::nullopt,
                index
            };
        }

        maximum_weight = std::max(maximum_weight, static_cast<long double>(point.weight));
        for (const double coordinate : {
                 point.position.x,
                 point.position.y,
                 point.position.z
             }) {
            const long double homogeneous_coordinate =
                static_cast<long double>(coordinate) *
                static_cast<long double>(point.weight);
            if (!std::isfinite(homogeneous_coordinate)) {
                return NurbsSurfaceError{
                    NurbsSurfaceErrorCode::numeric_range_not_supported,
                    std::nullopt,
                    index
                };
            }
            maximum_homogeneous_coordinate = std::max(
                maximum_homogeneous_coordinate,
                std::abs(homogeneous_coordinate)
            );
        }
    }

    const long double accumulator_limit = std::numeric_limits<long double>::max() /
        static_cast<long double>(points.size());
    if (maximum_weight > accumulator_limit ||
        maximum_homogeneous_coordinate > accumulator_limit) {
        return NurbsSurfaceError{NurbsSurfaceErrorCode::numeric_range_not_supported};
    }

    return std::nullopt;
}

std::optional<NurbsSurfaceError> validate_knots(
    const std::vector<double>& knots,
    size_t control_count,
    size_t degree,
    NurbsParameterDirection direction
) {
    if (degree >= control_count ||
        control_count > std::numeric_limits<size_t>::max() - degree - 1) {
        return NurbsSurfaceError{
            NurbsSurfaceErrorCode::degree_out_of_range,
            direction
        };
    }

    if (knots.size() != control_count + degree + 1) {
        return NurbsSurfaceError{
            NurbsSurfaceErrorCode::knot_count_mismatch,
            direction
        };
    }

    for (size_t index = 0; index < knots.size(); ++index) {
        if (!std::isfinite(knots[index])) {
            return NurbsSurfaceError{
                NurbsSurfaceErrorCode::non_finite_knot,
                direction,
                index
            };
        }
        if (index > 0 && knots[index] < knots[index - 1]) {
            return NurbsSurfaceError{
                NurbsSurfaceErrorCode::knots_not_nondecreasing,
                direction,
                index
            };
        }
    }

    if (degree > 0 && !std::isfinite(
            static_cast<long double>(knots[control_count + degree - 1]) -
            static_cast<long double>(knots[1])
        )) {
        return NurbsSurfaceError{
            NurbsSurfaceErrorCode::knot_range_not_finite,
            direction
        };
    }

    size_t multiplicity = 1;
    for (size_t index = 1; index < knots.size(); ++index) {
        multiplicity = knots[index] == knots[index - 1] ? multiplicity + 1 : 1;
        if (multiplicity > degree + 1) {
            return NurbsSurfaceError{
                NurbsSurfaceErrorCode::knot_multiplicity_exceeded,
                direction,
                index
            };
        }
    }

    if (knots[degree] >= knots[control_count]) {
        return NurbsSurfaceError{
            NurbsSurfaceErrorCode::empty_parameter_domain,
            direction
        };
    }

    return std::nullopt;
}

size_t find_span(
    double parameter,
    size_t control_count,
    size_t degree,
    const std::vector<double>& knots
) {
    if (parameter == knots[control_count]) {
        return control_count - 1;
    }

    const auto first_larger = std::upper_bound(
        knots.begin() + static_cast<std::ptrdiff_t>(degree),
        knots.begin() + static_cast<std::ptrdiff_t>(control_count + 1),
        parameter
    );
    return static_cast<size_t>(std::distance(knots.begin(), first_larger) - 1);
}

struct BasisWorkspace {
    explicit BasisWorkspace(size_t degree)
        : dynamic(degree > 3 ? 3 * (degree + 1) : 0), value_count(degree + 1) {}

    [[nodiscard]] std::span<long double> values() {
        auto storage = dynamic.empty()
            ? std::span<long double>(fixed)
            : std::span<long double>(dynamic);
        return storage.first(value_count);
    }

    [[nodiscard]] std::span<long double> left() {
        auto storage = dynamic.empty()
            ? std::span<long double>(fixed)
            : std::span<long double>(dynamic);
        return storage.subspan(value_count, value_count);
    }

    [[nodiscard]] std::span<long double> right() {
        auto storage = dynamic.empty()
            ? std::span<long double>(fixed)
            : std::span<long double>(dynamic);
        return storage.subspan(2 * value_count, value_count);
    }

    std::array<long double, 12> fixed{};
    std::vector<long double> dynamic;
    size_t value_count;
};

std::span<long double> basis_functions(
    size_t span,
    double parameter,
    size_t degree,
    const std::vector<double>& knots,
    BasisWorkspace& workspace
) {
    auto basis = workspace.values();
    auto left = workspace.left();
    auto right = workspace.right();
    basis[0] = 1.0L;

    for (size_t j = 1; j <= degree; ++j) {
        left[j] = static_cast<long double>(parameter) -
            static_cast<long double>(knots[span + 1 - j]);
        right[j] = static_cast<long double>(knots[span + j]) -
            static_cast<long double>(parameter);
        long double saved = 0.0L;

        for (size_t r = 0; r < j; ++r) {
            const long double denominator = right[r + 1] + left[j - r];
            const long double term = denominator == 0.0L
                ? 0.0L
                : basis[r] / denominator;
            basis[r] = saved + right[r + 1] * term;
            saved = left[j - r] * term;
        }

        basis[j] = saved;
    }

    return basis;
}

} // namespace

std::expected<std::unique_ptr<NurbsSurface>, NurbsSurfaceError> NurbsSurface::create(
    size_t u_count,
    size_t v_count,
    std::vector<ControlPoint> points
) {
    if (const auto error = validate_control_net(u_count, v_count, points)) {
        return std::unexpected(*error);
    }

    const size_t u_degree = std::min<size_t>(3, u_count - 1);
    const size_t v_degree = std::min<size_t>(3, v_count - 1);
    return create(
        u_count,
        v_count,
        u_degree,
        v_degree,
        std::move(points),
        make_open_uniform_knots(u_count, u_degree),
        make_open_uniform_knots(v_count, v_degree)
    );
}

std::expected<std::unique_ptr<NurbsSurface>, NurbsSurfaceError> NurbsSurface::create(
    size_t u_count,
    size_t v_count,
    size_t u_degree,
    size_t v_degree,
    std::vector<ControlPoint> points,
    std::vector<double> u_knots,
    std::vector<double> v_knots
) {
    if (const auto error = validate_control_net(u_count, v_count, points)) {
        return std::unexpected(*error);
    }
    if (const auto error = validate_knots(
            u_knots,
            u_count,
            u_degree,
            NurbsParameterDirection::u
        )) {
        return std::unexpected(*error);
    }
    if (const auto error = validate_knots(
            v_knots,
            v_count,
            v_degree,
            NurbsParameterDirection::v
        )) {
        return std::unexpected(*error);
    }

    return std::unique_ptr<NurbsSurface>(new NurbsSurface(
        ValidatedTag{},
        u_count,
        v_count,
        u_degree,
        v_degree,
        std::move(points),
        std::move(u_knots),
        std::move(v_knots)
    ));
}

NurbsSurface::NurbsSurface(
    ValidatedTag,
    size_t u_count,
    size_t v_count,
    size_t u_degree,
    size_t v_degree,
    std::vector<ControlPoint> points,
    std::vector<double> u_knots,
    std::vector<double> v_knots
)
    : m_u_count(u_count),
      m_v_count(v_count),
      m_u_degree(u_degree),
      m_v_degree(v_degree),
      m_control_points(std::move(points)),
      m_u_knots(std::move(u_knots)),
      m_v_knots(std::move(v_knots)) {}

std::vector<double> NurbsSurface::make_open_uniform_knots(size_t control_count, size_t degree) {
    if (control_count == 0 || degree >= control_count ||
        control_count > std::numeric_limits<size_t>::max() - degree - 1) {
        return {};
    }

    std::vector<double> knots(control_count + degree + 1, 1.0);
    std::fill_n(knots.begin(), degree + 1, 0.0);

    const size_t span_count = control_count - degree;
    for (size_t i = degree + 1; i < control_count; ++i) {
        knots[i] = static_cast<double>(i - degree) / static_cast<double>(span_count);
    }

    return knots;
}

std::expected<Point3D, CadError> NurbsSurface::evaluate(double u, double v) const {
    if (validate_knots(
            m_u_knots,
            m_u_count,
            m_u_degree,
            NurbsParameterDirection::u
        ).has_value() ||
        validate_knots(
            m_v_knots,
            m_v_count,
            m_v_degree,
            NurbsParameterDirection::v
        ).has_value() ||
        m_u_count > std::numeric_limits<size_t>::max() / m_v_count ||
        m_control_points.size() != m_u_count * m_v_count ||
        !std::ranges::all_of(m_control_points, [](const ControlPoint& point) {
            return point.weight > 0.0 && std::isfinite(point.weight) &&
                   std::isfinite(point.position.x) && std::isfinite(point.position.y) &&
                   std::isfinite(point.position.z);
        })) {
        return std::unexpected(CadError::DegenerateSurface);
    }

    const double u_start = m_u_knots[m_u_degree];
    const double u_end = m_u_knots[m_u_count];
    const double v_start = m_v_knots[m_v_degree];
    const double v_end = m_v_knots[m_v_count];
    if (!std::isfinite(u) || !std::isfinite(v) ||
        u < u_start || u > u_end || v < v_start || v > v_end) {
        return std::unexpected(CadError::OutOfBounds);
    }

    const size_t u_span = find_span(u, m_u_count, m_u_degree, m_u_knots);
    const size_t v_span = find_span(v, m_v_count, m_v_degree, m_v_knots);
    BasisWorkspace u_workspace(m_u_degree);
    BasisWorkspace v_workspace(m_v_degree);
    const auto u_basis = basis_functions(u_span, u, m_u_degree, m_u_knots, u_workspace);
    const auto v_basis = basis_functions(v_span, v, m_v_degree, m_v_knots, v_workspace);
    const size_t first_u = u_span - m_u_degree;
    const size_t first_v = v_span - m_v_degree;
    const auto net = control_net_2d();

    long double numerator_x = 0.0L;
    long double numerator_y = 0.0L;
    long double numerator_z = 0.0L;
    long double denominator = 0.0L;
    for (size_t i = 0; i <= m_u_degree; ++i) {
        for (size_t j = 0; j <= m_v_degree; ++j) {
            const auto& control_point = net[first_u + i, first_v + j];
            const long double coefficient =
                u_basis[i] * v_basis[j] *
                static_cast<long double>(control_point.weight);
            if (!std::isfinite(coefficient) || coefficient < 0.0) {
                return std::unexpected(CadError::DegenerateSurface);
            }
            numerator_x += static_cast<long double>(control_point.position.x) * coefficient;
            numerator_y += static_cast<long double>(control_point.position.y) * coefficient;
            numerator_z += static_cast<long double>(control_point.position.z) * coefficient;
            denominator += coefficient;
        }
    }

    if (!std::isfinite(numerator_x) || !std::isfinite(numerator_y) ||
        !std::isfinite(numerator_z) || !std::isfinite(denominator) ||
        denominator <= 0.0L) {
        return std::unexpected(CadError::DegenerateSurface);
    }

    const long double result_x = numerator_x / denominator;
    const long double result_y = numerator_y / denominator;
    const long double result_z = numerator_z / denominator;
    if (!std::isfinite(result_x) || !std::isfinite(result_y) ||
        !std::isfinite(result_z) ||
        result_x < std::numeric_limits<double>::lowest() ||
        result_x > std::numeric_limits<double>::max() ||
        result_y < std::numeric_limits<double>::lowest() ||
        result_y > std::numeric_limits<double>::max() ||
        result_z < std::numeric_limits<double>::lowest() ||
        result_z > std::numeric_limits<double>::max()) {
        return std::unexpected(CadError::DegenerateSurface);
    }
    return Point3D{
        static_cast<double>(result_x),
        static_cast<double>(result_y),
        static_cast<double>(result_z)
    };
}
