#include "nurbs_surface.h"
#include "core.h"
#include <cstddef>
#include <expected>
#include <vector>

NurbsSurface::NurbsSurface(size_t u_count, size_t v_count, std::vector<ControlPoint> points)
    : m_u_count(u_count), m_v_count(v_count), 
      m_control_points(std::move(points)),
      m_gpu_vbo(u_count * v_count * sizeof(ControlPoint)) {}

std::expected<Point3D, CadError> NurbsSurface::evaluate(double u, double v) const {
    if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0) {
        return std::unexpected(CadError::OutOfBounds);
    }

    if (m_control_points.empty()) {
        return std::unexpected(CadError::DegenerateSurface);
    }

    auto net = control_net_2d();
    size_t last_u = m_u_count - 1;
    size_t last_v = m_v_count - 1;

    double u_idx = u * last_u;
    double v_idx = v * last_v;

    size_t u0 = static_cast<size_t>(u_idx);
    size_t v0 = static_cast<size_t>(v_idx);
    size_t u1 = (u0 + 1 < m_u_count) ? u0 + 1 : u0;
    size_t v1 = (v0 + 1 < m_v_count) ? v0 + 1 : v0;

    double fu = u_idx - u0;
    double fv = v_idx - v0;

    const auto& cp00 = net[u0, v0];
    const auto& cp10 = net[u1, v0];
    const auto& cp01 = net[u0, v1];
    const auto& cp11 = net[u1, v1];

    double b00 = (1.0 - fu) * (1.0 - fv);
    double b10 = fu * (1.0 - fv);
    double b01 = (1.0 - fu) * fv;
    double b11 = fu * fv;

    double weight_sum = 
        b00 * cp00.weight + 
        b10 * cp10.weight + 
        b01 * cp01.weight + 
        b11 * cp11.weight;

    if (weight_sum <= 0.0) {
        return std::unexpected(CadError::DegenerateSurface);
    }

    Point3D weighted_point_sum = 
        (cp00.position * (b00 * cp00.weight)) +
        (cp10.position * (b10 * cp10.weight)) +
        (cp01.position * (b01 * cp01.weight)) +
        (cp11.position * (b11 * cp11.weight));

    return weighted_point_sum * (1.0 / weight_sum);
}