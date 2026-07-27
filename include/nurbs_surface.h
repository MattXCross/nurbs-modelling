#pragma once

#include <cstddef>
#include <vector>
#include <expected>
#include <mdspan>
#include <ranges>
#include <array>
#include <utility>

#include "core.h"
#include "gpu_buffer.h"

class NurbsSurface {
private:
    size_t m_u_count{0};
    size_t m_v_count{0};
    std::vector<ControlPoint> m_control_points;
    GpuVertexBuffer m_gpu_vbo;

public:
    NurbsSurface(size_t u_count, size_t v_count, std::vector<ControlPoint> points);

    ~NurbsSurface() = default;
    NurbsSurface(NurbsSurface&&) noexcept = default;
    NurbsSurface& operator=(NurbsSurface&&) noexcept = default;

    NurbsSurface(const NurbsSurface&) = delete;
    NurbsSurface& operator=(const NurbsSurface&) = delete;

    [[nodiscard]] auto control_net_2d() const {
        return std::mdspan(m_control_points.data(), m_u_count, m_v_count);
    }

    [[nodiscard]] std::expected<Point3D, CadError> evaluate(double u, double v) const;

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
            control_point.position + delta;
        }

        return std::forward<Self>(self);
    }

    [[nodiscard]] size_t u_count() const { return m_u_count; }
    [[nodiscard]] size_t v_count() const { return m_v_count; }
};