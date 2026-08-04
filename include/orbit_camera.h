#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

#include "kernel_math.h"
#include "math_types.h"

enum class ProjectionMode { perspective, orthographic };
enum class StandardView { front, back, top, bottom, left, right };

struct CameraState {
    cad::Point3 position{};
    cad::Point3 target{};
    cad::Vector3 up{0.0, 1.0, 0.0};
    float vertical_fov_degrees{45.0f};
    ProjectionMode projection{ProjectionMode::perspective};
    double orthographic_vertical_size{10.0};
};

class OrbitCameraController {
public:
    OrbitCameraController(cad::Point3 init_position, cad::Point3 init_target, float fov = 45.0f) {
        m_camera.position = cad::is_finite(init_position) ? init_position : cad::Point3{0.0, 0.0, 10.0};
        m_camera.target = cad::is_finite(init_target) ? init_target : cad::Point3{};
        if (std::isfinite(fov) && fov > 1.0f && fov < 179.0f) {
            m_camera.vertical_fov_degrees = fov;
        }

        const cad::Vector3 offset = m_camera.position - m_camera.target;
        m_distance = cad::length(offset);
        if (!std::isfinite(m_distance) || m_distance < m_min_distance) {
            m_distance = 1.0;
            update_position();
        } else {
            m_yaw_rad = std::atan2(offset.x, offset.z);
            m_pitch_rad = std::asin(std::clamp(offset.y / m_distance, -1.0, 1.0));
        }
        m_camera.orthographic_vertical_size = perspective_vertical_size();
    }

    void orbit(Vec2 mouse_delta) {
        if (!std::isfinite(mouse_delta.x) || !std::isfinite(mouse_delta.y)) return;
        m_yaw_rad -= mouse_delta.x * m_orbit_sensitivity;
        m_pitch_rad = std::clamp(
            m_pitch_rad + mouse_delta.y * m_orbit_sensitivity,
            -m_pitch_limit_rad,
            m_pitch_limit_rad
        );
        m_camera.up = {0.0, 1.0, 0.0};
        update_position();
    }

    void pan(Vec2 mouse_delta, float viewport_height) {
        if (!std::isfinite(mouse_delta.x) || !std::isfinite(mouse_delta.y)) return;
        const auto [forward, right, screen_up] = basis();
        const double safe_height = std::max(static_cast<double>(viewport_height), 1.0);
        const double pan_scale = vertical_size_at_target() / safe_height;
        m_camera.target = m_camera.target +
            right * (-static_cast<double>(mouse_delta.x) * pan_scale) +
            screen_up * (static_cast<double>(mouse_delta.y) * pan_scale);
        update_position();
    }

    void zoom(float wheel_movement) {
        zoom(wheel_movement, {}, 1, 1);
    }

    void zoom(float wheel_movement, Vec2 cursor, int viewport_width, int viewport_height) {
        if (!std::isfinite(wheel_movement) || wheel_movement == 0.0f) return;
        const double clamped_steps = std::clamp(static_cast<double>(wheel_movement), -8.0, 8.0);
        const double factor = 1.0 - 0.1 * clamped_steps;
        const auto before = target_plane_point(cursor, viewport_width, viewport_height);

        if (m_camera.projection == ProjectionMode::perspective) {
            m_distance = std::clamp(m_distance * factor, m_min_distance, m_max_distance);
        } else {
            m_camera.orthographic_vertical_size = std::clamp(
                m_camera.orthographic_vertical_size * factor,
                m_min_orthographic_size,
                m_max_orthographic_size
            );
        }
        update_position();

        const auto after = target_plane_point(cursor, viewport_width, viewport_height);
        if (before && after) {
            m_camera.target = m_camera.target + (*before - *after);
            update_position();
        }
    }

    [[nodiscard]] bool frame_bounds(const cad::Aabb3& bounds, double aspect_ratio) {
        const auto minimum = bounds.minimum();
        const auto maximum = bounds.maximum();
        const auto center = bounds.center();
        if (!minimum || !maximum || !center || !cad::is_finite(*center) ||
            !std::isfinite(aspect_ratio) || aspect_ratio <= 0.0) {
            return false;
        }

        const auto [forward, right, screen_up] = basis();
        double half_width = 0.0;
        double half_height = 0.0;
        double required_distance = 0.0;
        const double tangent = std::tan(
            static_cast<double>(m_camera.vertical_fov_degrees) * std::numbers::pi / 360.0
        );
        if (!std::isfinite(tangent) || tangent <= 0.0) return false;

        for (const cad::Point3 corner : corners(*minimum, *maximum)) {
            const cad::Vector3 offset = corner - *center;
            const double x = std::abs(cad::dot(offset, right));
            const double y = std::abs(cad::dot(offset, screen_up));
            const double z = cad::dot(offset, forward);
            half_width = std::max(half_width, x);
            half_height = std::max(half_height, y);
            required_distance = std::max({
                required_distance,
                x * m_fit_margin / (tangent * aspect_ratio) - z,
                y * m_fit_margin / tangent - z,
                -z + m_min_distance
            });
        }

        const double model_size = std::max({half_width, half_height, required_distance});
        if (!std::isfinite(model_size)) return false;
        m_camera.target = *center;
        if (m_camera.projection == ProjectionMode::perspective) {
            m_distance = std::clamp(
                required_distance > m_min_distance ? required_distance : 1.0,
                m_min_distance,
                m_max_distance
            );
        } else {
            double size = 2.0 * m_fit_margin * std::max(half_height, half_width / aspect_ratio);
            if (size <= m_min_orthographic_size) size = 1.0;
            m_camera.orthographic_vertical_size = std::clamp(
                size, m_min_orthographic_size, m_max_orthographic_size
            );
            m_distance = std::clamp(
                std::max(m_distance, model_size * 2.0), m_min_distance, m_max_distance
            );
        }
        update_position();
        return true;
    }

    void set_standard_view(StandardView view) {
        cad::Vector3 offset;
        switch (view) {
            case StandardView::front: offset = {0.0, 0.0, 1.0}; m_camera.up = {0.0, 1.0, 0.0}; break;
            case StandardView::back: offset = {0.0, 0.0, -1.0}; m_camera.up = {0.0, 1.0, 0.0}; break;
            case StandardView::top: offset = {0.0, 1.0, 0.0}; m_camera.up = {0.0, 0.0, -1.0}; break;
            case StandardView::bottom: offset = {0.0, -1.0, 0.0}; m_camera.up = {0.0, 0.0, 1.0}; break;
            case StandardView::left: offset = {-1.0, 0.0, 0.0}; m_camera.up = {0.0, 1.0, 0.0}; break;
            case StandardView::right: offset = {1.0, 0.0, 0.0}; m_camera.up = {0.0, 1.0, 0.0}; break;
        }
        m_yaw_rad = std::atan2(offset.x, offset.z);
        m_pitch_rad = std::asin(offset.y);
        m_camera.position = m_camera.target + offset * m_distance;
    }

    void set_projection(ProjectionMode projection) {
        if (m_camera.projection == projection) return;
        if (projection == ProjectionMode::orthographic) {
            m_camera.orthographic_vertical_size = perspective_vertical_size();
        } else {
            const double tangent = std::tan(
                static_cast<double>(m_camera.vertical_fov_degrees) * std::numbers::pi / 360.0
            );
            m_distance = std::clamp(
                m_camera.orthographic_vertical_size / (2.0 * tangent),
                m_min_distance,
                m_max_distance
            );
            update_position();
        }
        m_camera.projection = projection;
    }

    [[nodiscard]] const CameraState& camera() const { return m_camera; }

private:
    struct Basis {
        cad::Vector3 forward;
        cad::Vector3 right;
        cad::Vector3 up;
    };

    [[nodiscard]] Basis basis() const {
        const cad::Vector3 forward = cad::normalized(m_camera.target - m_camera.position)
            .value_or(cad::Vector3{0.0, 0.0, -1.0});
        const cad::Vector3 right = cad::normalized(cad::cross(forward, m_camera.up))
            .value_or(cad::Vector3{1.0, 0.0, 0.0});
        return {forward, right, cad::normalized(cad::cross(right, forward)).value_or(m_camera.up)};
    }

    [[nodiscard]] double perspective_vertical_size() const {
        return 2.0 * m_distance * std::tan(
            static_cast<double>(m_camera.vertical_fov_degrees) * std::numbers::pi / 360.0
        );
    }

    [[nodiscard]] double vertical_size_at_target() const {
        return m_camera.projection == ProjectionMode::orthographic
            ? m_camera.orthographic_vertical_size : perspective_vertical_size();
    }

    [[nodiscard]] std::optional<cad::Point3> target_plane_point(
        Vec2 cursor, int width, int height
    ) const {
        if (width <= 0 || height <= 0 || !std::isfinite(cursor.x) || !std::isfinite(cursor.y)) {
            return std::nullopt;
        }
        const auto [forward, right, screen_up] = basis();
        const double nx = 2.0 * static_cast<double>(cursor.x) / width - 1.0;
        const double ny = 1.0 - 2.0 * static_cast<double>(cursor.y) / height;
        const double half_height = vertical_size_at_target() * 0.5;
        const double aspect = static_cast<double>(width) / height;
        return m_camera.target + right * (nx * aspect * half_height) + screen_up * (ny * half_height);
    }

    static std::array<cad::Point3, 8> corners(cad::Point3 minimum, cad::Point3 maximum) {
        return {{{minimum.x, minimum.y, minimum.z}, {maximum.x, minimum.y, minimum.z},
                 {minimum.x, maximum.y, minimum.z}, {maximum.x, maximum.y, minimum.z},
                 {minimum.x, minimum.y, maximum.z}, {maximum.x, minimum.y, maximum.z},
                 {minimum.x, maximum.y, maximum.z}, {maximum.x, maximum.y, maximum.z}}};
    }

    void update_position() {
        const double horizontal_distance = m_distance * std::cos(m_pitch_rad);
        m_camera.position = {
            m_camera.target.x + horizontal_distance * std::sin(m_yaw_rad),
            m_camera.target.y + m_distance * std::sin(m_pitch_rad),
            m_camera.target.z + horizontal_distance * std::cos(m_yaw_rad)
        };
    }

    CameraState m_camera{};
    double m_yaw_rad{0.0};
    double m_pitch_rad{0.0};
    double m_distance{10.0};
    double m_orbit_sensitivity{0.005};
    double m_pitch_limit_rad{1.5};
    double m_min_distance{1e-12};
    double m_max_distance{1e15};
    double m_min_orthographic_size{1e-12};
    double m_max_orthographic_size{1e15};
    double m_fit_margin{1.1};
};
