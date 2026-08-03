#pragma once

#include <cstddef>
#include <vector>
#include <expected>
#include <memory>
#include <mdspan>
#include <optional>
#include <ranges>
#include <array>
#include <utility>

#include "core.h"
#include "geometry_tolerance.h"

class Scene;

enum class NurbsSurfaceErrorCode {
    invalid_control_net_dimensions,
    control_point_count_mismatch,
    degree_out_of_range,
    knot_count_mismatch,
    non_finite_knot,
    knots_not_nondecreasing,
    knot_range_not_finite,
    knot_multiplicity_exceeded,
    empty_parameter_domain,
    non_finite_control_point,
    non_positive_weight,
    numeric_range_not_supported,
    control_point_out_of_range
};

enum class NurbsParameterDirection { u, v };

struct NurbsSurfaceError {
    static constexpr size_t no_index = static_cast<size_t>(-1);

    NurbsSurfaceErrorCode code;
    std::optional<NurbsParameterDirection> direction{};
    size_t index{no_index};
};

struct NurbsSurfaceDerivatives {
    Point3D position;
    cad::Vector3 u;
    cad::Vector3 v;
    cad::Vector3 uu;
    cad::Vector3 uv;
    cad::Vector3 vv;
};

class NurbsSurface {
private:
    friend class Scene;

    struct ValidatedTag {};

    NurbsSurface(
        ValidatedTag,
        size_t u_count,
        size_t v_count,
        size_t u_degree,
        size_t v_degree,
        std::vector<ControlPoint> points,
        std::vector<double> u_knots,
        std::vector<double> v_knots
    );

    [[nodiscard]] std::expected<bool, NurbsSurfaceError> set_control_point(
        size_t u,
        size_t v,
        ControlPoint point
    );
    [[nodiscard]] std::expected<NurbsSurfaceDerivatives, CadError> evaluate_derivatives_impl(
        double u,
        double v,
        size_t derivative_order
    ) const;

    size_t m_u_count{0};
    size_t m_v_count{0};
    size_t m_u_degree{0};
    size_t m_v_degree{0};
    std::vector<ControlPoint> m_control_points;
    std::vector<double> m_u_knots;
    std::vector<double> m_v_knots;

public:
    [[nodiscard]] static std::expected<std::unique_ptr<NurbsSurface>, NurbsSurfaceError> create(
        size_t u_count,
        size_t v_count,
        std::vector<ControlPoint> points
    );

    [[nodiscard]] static std::expected<std::unique_ptr<NurbsSurface>, NurbsSurfaceError> create(
        size_t u_count,
        size_t v_count,
        size_t u_degree,
        size_t v_degree,
        std::vector<ControlPoint> points
    );

    [[nodiscard]] static std::expected<std::unique_ptr<NurbsSurface>, NurbsSurfaceError> create(
        size_t u_count,
        size_t v_count,
        size_t u_degree,
        size_t v_degree,
        std::vector<ControlPoint> points,
        std::vector<double> u_knots,
        std::vector<double> v_knots
    );

    ~NurbsSurface() = default;
    NurbsSurface(NurbsSurface&&) = delete;
    NurbsSurface& operator=(NurbsSurface&&) = delete;

    NurbsSurface(const NurbsSurface&) = delete;
    NurbsSurface& operator=(const NurbsSurface&) = delete;

    [[nodiscard]] auto control_net_2d() const {
        return std::mdspan(m_control_points.data(), m_u_count, m_v_count);
    }

    [[nodiscard]] std::expected<Point3D, CadError> evaluate(double u, double v) const;
    // At internal knots derivatives are right-sided; at the domain end they are left-sided.
    [[nodiscard]] std::expected<NurbsSurfaceDerivatives, CadError> evaluate_derivatives(
        double u,
        double v
    ) const;
    [[nodiscard]] std::expected<cad::Vector3, CadError> normal(
        double u,
        double v,
        const cad::GeometryTolerance& tolerance
    ) const;

    [[nodiscard]] static std::vector<double> make_open_uniform_knots(
        size_t control_count,
        size_t degree
    );

    [[nodiscard]] auto homogeneous_view() const {
        return m_control_points | std::views::transform([](const ControlPoint& control_point) {
            return std::array<double, 4>{
                control_point.position.x * control_point.weight,
                control_point.position.y * control_point.weight,
                control_point.position.z * control_point.weight,
                control_point.weight
            };
        });
    }

    [[nodiscard]] size_t u_count() const { return m_u_count; }
    [[nodiscard]] size_t v_count() const { return m_v_count; }
    [[nodiscard]] size_t u_degree() const { return m_u_degree; }
    [[nodiscard]] size_t v_degree() const { return m_v_degree; }
    [[nodiscard]] const std::vector<double>& u_knots() const { return m_u_knots; }
    [[nodiscard]] const std::vector<double>& v_knots() const { return m_v_knots; }
    [[nodiscard]] std::optional<std::pair<double, double>> u_domain() const {
        if (m_u_degree >= m_u_knots.size() || m_u_count >= m_u_knots.size()) {
            return std::nullopt;
        }
        return std::pair{m_u_knots[m_u_degree], m_u_knots[m_u_count]};
    }
    [[nodiscard]] std::optional<std::pair<double, double>> v_domain() const {
        if (m_v_degree >= m_v_knots.size() || m_v_count >= m_v_knots.size()) {
            return std::nullopt;
        }
        return std::pair{m_v_knots[m_v_degree], m_v_knots[m_v_count]};
    }
};
