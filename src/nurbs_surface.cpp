#include "nurbs_surface.h"
#include "bspline_basis.h"
#include "core.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <expected>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace {

struct LongVector3 {
    long double x{0.0L};
    long double y{0.0L};
    long double z{0.0L};
};

struct HomogeneousDerivative {
    LongVector3 xyz;
    long double weight{0.0L};
};

LongVector3 operator-(LongVector3 left, LongVector3 right) {
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

LongVector3 operator*(LongVector3 vector, long double scalar) {
    return {vector.x * scalar, vector.y * scalar, vector.z * scalar};
}

LongVector3 operator/(LongVector3 vector, long double scalar) {
    return {vector.x / scalar, vector.y / scalar, vector.z / scalar};
}

bool finite_and_representable(LongVector3 vector) {
    constexpr long double lowest = std::numeric_limits<double>::lowest();
    constexpr long double highest = std::numeric_limits<double>::max();
    return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z) &&
        vector.x >= lowest && vector.x <= highest &&
        vector.y >= lowest && vector.y <= highest &&
        vector.z >= lowest && vector.z <= highest;
}

cad::Vector3 to_vector3(LongVector3 vector) {
    return {
        static_cast<double>(vector.x),
        static_cast<double>(vector.y),
        static_cast<double>(vector.z)
    };
}

std::optional<NurbsSurfaceError> validate_control_point(
    const ControlPoint& point,
    size_t index,
    long double accumulator_limit
) {
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

    if (static_cast<long double>(point.weight) > accumulator_limit) {
        return NurbsSurfaceError{
            NurbsSurfaceErrorCode::numeric_range_not_supported,
            std::nullopt,
            index
        };
    }
    for (const double coordinate : {
             point.position.x,
             point.position.y,
             point.position.z
         }) {
        const long double homogeneous_coordinate =
            static_cast<long double>(coordinate) *
            static_cast<long double>(point.weight);
        if (!std::isfinite(homogeneous_coordinate) ||
            std::abs(homogeneous_coordinate) > accumulator_limit) {
            return NurbsSurfaceError{
                NurbsSurfaceErrorCode::numeric_range_not_supported,
                std::nullopt,
                index
            };
        }
    }

    return std::nullopt;
}

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

    const long double accumulator_limit = std::numeric_limits<long double>::max() /
        static_cast<long double>(points.size());
    for (size_t index = 0; index < points.size(); ++index) {
        if (const auto error = validate_control_point(
                points[index],
                index,
                accumulator_limit
            )) {
            return error;
        }
    }

    return std::nullopt;
}

bool same_control_point(const ControlPoint& left, const ControlPoint& right) {
    return left.position.x == right.position.x &&
        left.position.y == right.position.y &&
        left.position.z == right.position.z &&
        left.weight == right.weight;
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
    std::vector<ControlPoint> points
) {
    if (const auto error = validate_control_net(u_count, v_count, points)) {
        return std::unexpected(*error);
    }
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

std::expected<bool, NurbsSurfaceError> NurbsSurface::set_control_point(
    size_t u,
    size_t v,
    ControlPoint point
) {
    if (u >= m_u_count || v >= m_v_count) {
        return std::unexpected(NurbsSurfaceError{
            NurbsSurfaceErrorCode::control_point_out_of_range
        });
    }

    ControlPoint& current = m_control_points[u * m_v_count + v];
    if (same_control_point(current, point)) {
        return false;
    }

    const size_t index = u * m_v_count + v;
    const long double accumulator_limit = std::numeric_limits<long double>::max() /
        static_cast<long double>(m_control_points.size());
    if (const auto error = validate_control_point(point, index, accumulator_limit)) {
        return std::unexpected(*error);
    }

    current = point;
    return true;
}

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
    if (m_u_count == 0 || m_v_count == 0 ||
        m_u_count > std::numeric_limits<size_t>::max() / m_v_count ||
        m_control_points.size() != m_u_count * m_v_count ||
        !std::ranges::all_of(m_control_points, [](const ControlPoint& point) {
            return point.weight > 0.0 && std::isfinite(point.weight) &&
                   std::isfinite(point.position.x) && std::isfinite(point.position.y) &&
                   std::isfinite(point.position.z);
        })) {
        return std::unexpected(CadError::DegenerateSurface);
    }

    const auto u_evaluation = cad::evaluate_bspline_basis(
        m_u_count,
        m_u_degree,
        m_u_knots,
        u
    );
    if (!u_evaluation) {
        return std::unexpected(
            u_evaluation.error() == cad::BSplineBasisError::parameter_out_of_domain
                ? CadError::OutOfBounds
                : CadError::DegenerateSurface
        );
    }
    const auto v_evaluation = cad::evaluate_bspline_basis(
        m_v_count,
        m_v_degree,
        m_v_knots,
        v
    );
    if (!v_evaluation) {
        return std::unexpected(
            v_evaluation.error() == cad::BSplineBasisError::parameter_out_of_domain
                ? CadError::OutOfBounds
                : CadError::DegenerateSurface
        );
    }
    const auto u_basis = u_evaluation->derivative(0);
    const auto v_basis = v_evaluation->derivative(0);
    const size_t first_u = u_evaluation->first_control_point();
    const size_t first_v = v_evaluation->first_control_point();
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

std::expected<NurbsSurfaceDerivatives, CadError> NurbsSurface::evaluate_derivatives(
    double u,
    double v
) const {
    return evaluate_derivatives_impl(u, v, 2);
}

std::expected<NurbsSurfaceDerivatives, CadError> NurbsSurface::evaluate_derivatives_impl(
    double u,
    double v,
    size_t derivative_order
) const {
    if (m_u_count == 0 || m_v_count == 0 ||
        m_u_count > std::numeric_limits<size_t>::max() / m_v_count ||
        m_control_points.size() != m_u_count * m_v_count ||
        !std::ranges::all_of(m_control_points, [](const ControlPoint& point) {
            return point.weight > 0.0 && std::isfinite(point.weight) &&
                std::isfinite(point.position.x) && std::isfinite(point.position.y) &&
                std::isfinite(point.position.z);
        })) {
        return std::unexpected(CadError::DegenerateSurface);
    }

    const auto u_evaluation = cad::evaluate_bspline_basis(
        m_u_count,
        m_u_degree,
        m_u_knots,
        u,
        derivative_order
    );
    if (!u_evaluation) {
        return std::unexpected(
            u_evaluation.error() == cad::BSplineBasisError::parameter_out_of_domain
                ? CadError::OutOfBounds
                : CadError::DegenerateSurface
        );
    }
    const auto v_evaluation = cad::evaluate_bspline_basis(
        m_v_count,
        m_v_degree,
        m_v_knots,
        v,
        derivative_order
    );
    if (!v_evaluation) {
        return std::unexpected(
            v_evaluation.error() == cad::BSplineBasisError::parameter_out_of_domain
                ? CadError::OutOfBounds
                : CadError::DegenerateSurface
        );
    }

    std::array<HomogeneousDerivative, 9> homogeneous{};
    const auto derivative = [&homogeneous](std::size_t u_order, std::size_t v_order)
        -> HomogeneousDerivative& {
        return homogeneous[u_order * 3 + v_order];
    };
    const auto net = control_net_2d();
    const std::size_t first_u = u_evaluation->first_control_point();
    const std::size_t first_v = v_evaluation->first_control_point();
    for (std::size_t u_order = 0; u_order <= derivative_order; ++u_order) {
        const auto u_basis = u_evaluation->derivative(u_order);
        for (std::size_t v_order = 0; v_order + u_order <= derivative_order; ++v_order) {
            const auto v_basis = v_evaluation->derivative(v_order);
            HomogeneousDerivative& value = derivative(u_order, v_order);
            for (std::size_t i = 0; i <= m_u_degree; ++i) {
                for (std::size_t j = 0; j <= m_v_degree; ++j) {
                    const ControlPoint& point = net[first_u + i, first_v + j];
                    const long double coefficient =
                        u_basis[i] * v_basis[j] * static_cast<long double>(point.weight);
                    value.xyz.x += static_cast<long double>(point.position.x) * coefficient;
                    value.xyz.y += static_cast<long double>(point.position.y) * coefficient;
                    value.xyz.z += static_cast<long double>(point.position.z) * coefficient;
                    value.weight += coefficient;
                }
            }
            if (!std::isfinite(value.xyz.x) || !std::isfinite(value.xyz.y) ||
                !std::isfinite(value.xyz.z) || !std::isfinite(value.weight)) {
                return std::unexpected(CadError::DegenerateSurface);
            }
        }
    }

    const HomogeneousDerivative& h00 = derivative(0, 0);
    const HomogeneousDerivative& h10 = derivative(1, 0);
    const HomogeneousDerivative& h01 = derivative(0, 1);
    const HomogeneousDerivative& h20 = derivative(2, 0);
    const HomogeneousDerivative& h11 = derivative(1, 1);
    const HomogeneousDerivative& h02 = derivative(0, 2);
    if (!std::isfinite(h00.weight) || h00.weight <= 0.0L) {
        return std::unexpected(CadError::DegenerateSurface);
    }

    const LongVector3 position = h00.xyz / h00.weight;
    const LongVector3 derivative_u = (h10.xyz - position * h10.weight) / h00.weight;
    const LongVector3 derivative_v = (h01.xyz - position * h01.weight) / h00.weight;
    LongVector3 derivative_uu{};
    LongVector3 derivative_uv{};
    LongVector3 derivative_vv{};
    if (derivative_order >= 2) {
        derivative_uu = (
            h20.xyz - derivative_u * (2.0L * h10.weight) - position * h20.weight
        ) / h00.weight;
        derivative_uv = (
            h11.xyz - derivative_v * h10.weight - derivative_u * h01.weight -
            position * h11.weight
        ) / h00.weight;
        derivative_vv = (
            h02.xyz - derivative_v * (2.0L * h01.weight) - position * h02.weight
        ) / h00.weight;
    }

    if (!finite_and_representable(position) || !finite_and_representable(derivative_u) ||
        !finite_and_representable(derivative_v) ||
        (derivative_order >= 2 &&
         (!finite_and_representable(derivative_uu) ||
          !finite_and_representable(derivative_uv) ||
          !finite_and_representable(derivative_vv)))) {
        return std::unexpected(CadError::DegenerateSurface);
    }

    return NurbsSurfaceDerivatives{
        .position = {
            static_cast<double>(position.x),
            static_cast<double>(position.y),
            static_cast<double>(position.z)
        },
        .u = to_vector3(derivative_u),
        .v = to_vector3(derivative_v),
        .uu = to_vector3(derivative_uu),
        .uv = to_vector3(derivative_uv),
        .vv = to_vector3(derivative_vv)
    };
}

std::expected<cad::Vector3, CadError> NurbsSurface::normal(
    double u,
    double v,
    const cad::GeometryTolerance& tolerance
) const {
    const auto surface = evaluate_derivatives_impl(u, v, 1);
    if (!surface) {
        return std::unexpected(surface.error());
    }

    const auto unit_u = cad::normalized(surface->u);
    const auto unit_v = cad::normalized(surface->v);
    if (!unit_u || !unit_v) {
        return std::unexpected(CadError::DegenerateSurface);
    }
    const cad::Vector3 cross_product = cad::cross(*unit_u, *unit_v);
    const double collinearity_angle = std::atan2(
        cad::length(cross_product),
        std::abs(std::clamp(cad::dot(*unit_u, *unit_v), -1.0, 1.0))
    );
    if (!std::isfinite(collinearity_angle) ||
        collinearity_angle <= tolerance.angular_radians()) {
        return std::unexpected(CadError::DegenerateSurface);
    }
    const auto result = cad::normalized(cross_product);
    if (!result) {
        return std::unexpected(CadError::DegenerateSurface);
    }
    return *result;
}
