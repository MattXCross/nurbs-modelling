#pragma once

#include <cstddef>
#include <vector>
#include <expected>
#include <mdspan>
#include <optional>
#include <ranges>
#include <array>
#include <utility>

#include "core.h"

class NurbsSurface {
private:
    size_t m_u_count{0};
    size_t m_v_count{0};
    size_t m_u_degree{0};
    size_t m_v_degree{0};
    std::vector<ControlPoint> m_control_points;
    std::vector<double> m_u_knots;
    std::vector<double> m_v_knots;

public:
    NurbsSurface(size_t u_count, size_t v_count, std::vector<ControlPoint> points);
    NurbsSurface(
        size_t u_count,
        size_t v_count,
        size_t u_degree,
        size_t v_degree,
        std::vector<ControlPoint> points,
        std::vector<double> u_knots,
        std::vector<double> v_knots
    );

    ~NurbsSurface() = default;
    NurbsSurface(NurbsSurface&&) noexcept = default;
    NurbsSurface& operator=(NurbsSurface&&) noexcept = default;

    NurbsSurface(const NurbsSurface&) = default;
    NurbsSurface& operator=(const NurbsSurface&) = default;

    [[nodiscard]] auto control_net_2d() {
        return std::mdspan(m_control_points.data(), m_u_count, m_v_count);
    }

    [[nodiscard]] auto control_net_2d() const {
        return std::mdspan(m_control_points.data(), m_u_count, m_v_count);
    }

    [[nodiscard]] std::expected<Point3D, CadError> evaluate(double u, double v) const;

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

    template<typename Self>
    Self&& translate(this Self&& self, Point3D delta) {
        for (auto& control_point : self.m_control_points) {
            control_point.position = control_point.position + delta;
        }

        return std::forward<Self>(self);
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
