#include "nurbs_surface.h"
#include "core.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <expected>
#include <limits>
#include <span>
#include <vector>

namespace {

bool valid_knots(const std::vector<double>& knots, size_t control_count, size_t degree) {
    if (control_count == 0 || degree >= control_count ||
        control_count > std::numeric_limits<size_t>::max() - degree - 1 ||
        knots.size() != control_count + degree + 1) {
        return false;
    }

    if (!std::ranges::all_of(knots, [](double knot) { return std::isfinite(knot); }) ||
        !std::ranges::is_sorted(knots)) {
        return false;
    }

    return knots[degree] < knots[control_count];
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

    [[nodiscard]] std::span<double> values() {
        auto storage = dynamic.empty() ? std::span<double>(fixed) : std::span<double>(dynamic);
        return storage.first(value_count);
    }

    [[nodiscard]] std::span<double> left() {
        auto storage = dynamic.empty() ? std::span<double>(fixed) : std::span<double>(dynamic);
        return storage.subspan(value_count, value_count);
    }

    [[nodiscard]] std::span<double> right() {
        auto storage = dynamic.empty() ? std::span<double>(fixed) : std::span<double>(dynamic);
        return storage.subspan(2 * value_count, value_count);
    }

    std::array<double, 12> fixed{};
    std::vector<double> dynamic;
    size_t value_count;
};

std::span<double> basis_functions(
    size_t span,
    double parameter,
    size_t degree,
    const std::vector<double>& knots,
    BasisWorkspace& workspace
) {
    auto basis = workspace.values();
    auto left = workspace.left();
    auto right = workspace.right();
    basis[0] = 1.0;

    for (size_t j = 1; j <= degree; ++j) {
        left[j] = parameter - knots[span + 1 - j];
        right[j] = knots[span + j] - parameter;
        double saved = 0.0;

        for (size_t r = 0; r < j; ++r) {
            const double denominator = right[r + 1] + left[j - r];
            const double term = denominator == 0.0 ? 0.0 : basis[r] / denominator;
            basis[r] = saved + right[r + 1] * term;
            saved = left[j - r] * term;
        }

        basis[j] = saved;
    }

    return basis;
}

} // namespace

NurbsSurface::NurbsSurface(size_t u_count, size_t v_count, std::vector<ControlPoint> points)
    : NurbsSurface(
          u_count,
          v_count,
          u_count == 0 ? 0 : std::min<size_t>(3, u_count - 1),
          v_count == 0 ? 0 : std::min<size_t>(3, v_count - 1),
          std::move(points),
          make_open_uniform_knots(
              u_count,
              u_count == 0 ? 0 : std::min<size_t>(3, u_count - 1)
          ),
          make_open_uniform_knots(
              v_count,
              v_count == 0 ? 0 : std::min<size_t>(3, v_count - 1)
          )
      ) {}

NurbsSurface::NurbsSurface(
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
    if (!valid_knots(m_u_knots, m_u_count, m_u_degree) ||
        !valid_knots(m_v_knots, m_v_count, m_v_degree) ||
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

    Point3D numerator{};
    double denominator = 0.0;
    for (size_t i = 0; i <= m_u_degree; ++i) {
        for (size_t j = 0; j <= m_v_degree; ++j) {
            const auto& control_point = net[first_u + i, first_v + j];
            const double coefficient = u_basis[i] * v_basis[j] * control_point.weight;
            numerator = numerator + control_point.position * coefficient;
            denominator += coefficient;
        }
    }

    if (!std::isfinite(denominator) || denominator <= 0.0) {
        return std::unexpected(CadError::DegenerateSurface);
    }

    return numerator * (1.0 / denominator);
}
