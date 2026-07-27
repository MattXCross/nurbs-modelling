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

    Point3D p00 = net[u0, v0].position;
    Point3D p10 = net[u1, v0].position;
    Point3D p01 = net[u0, v1].position;
    Point3D p11 = net[u1, v1].position;

    Point3D p0 = p00 * (1.0 - fu) + p10 * fu;
    Point3D p1 = p01 * (1.0 - fu) + p11 * fu;

    return p0 * (1.0 - fv) + p1 * fv;
}