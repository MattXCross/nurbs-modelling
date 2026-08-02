#pragma once

#include <algorithm>
#include <cmath>

#include "kernel_math.h"
#include "math_types.h"

struct CameraState {
    cad::Point3 position{};
    cad::Point3 target{};
    cad::Vector3 up{0.0, 1.0, 0.0};
    float vertical_fov_degrees{45.0f};
};

class OrbitCameraController {
private:
    CameraState m_camera{};
    double m_yaw_rad{0.0};
    double m_pitch_rad{0.0};
    double m_distance{10.0};

    double m_orbit_sensitivity{0.005};
    double m_pitch_limit_rad{1.5};
    double m_min_distance{1.0};
    double m_max_distance{100000.0};

public:
    OrbitCameraController(cad::Point3 init_position, cad::Point3 init_target, float fov = 45.0f) {
        m_camera.position = init_position;
        m_camera.target = init_target;
        m_camera.vertical_fov_degrees = fov;

        const cad::Vector3 offset = init_position - init_target;
        m_distance = cad::length(offset);
        if (m_distance < m_min_distance) {
            m_distance = m_min_distance;
            m_yaw_rad = 0.0;
            m_pitch_rad = 0.0;
            update_position();
            return;
        }
        m_yaw_rad = std::atan2(offset.x, offset.z);
        m_pitch_rad = std::asin(offset.y / m_distance);
    }  

    void orbit(Vec2 mouse_delta) {
        m_yaw_rad -= mouse_delta.x * m_orbit_sensitivity;
        m_pitch_rad += mouse_delta.y * m_orbit_sensitivity;
        m_pitch_rad = std::clamp(m_pitch_rad, -m_pitch_limit_rad, m_pitch_limit_rad);
        update_position();
    }

    void pan(Vec2 mouse_delta, float viewport_height) {
        const cad::Vector3 forward_dir =
            cad::normalized(m_camera.target - m_camera.position).value_or(cad::Vector3{});
        const cad::Vector3 right_dir =
            cad::normalized(cad::cross(forward_dir, m_camera.up)).value_or(cad::Vector3{});
        const cad::Vector3 screen_up_dir =
            cad::normalized(cad::cross(right_dir, forward_dir)).value_or(cad::Vector3{});

        constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
        const double safe_height = std::max(static_cast<double>(viewport_height), 1.0);
        const double pan_scale = 2.0 * m_distance *
            std::tan(static_cast<double>(m_camera.vertical_fov_degrees) *
                     degrees_to_radians * 0.5) / safe_height;

        m_camera.target = m_camera.target +
            right_dir * (-static_cast<double>(mouse_delta.x) * pan_scale) +
            screen_up_dir * (static_cast<double>(mouse_delta.y) * pan_scale);
        update_position();
    }

    void zoom(float wheel_movement) {
        if (wheel_movement == 0.0f) return;

        m_distance *= (1.0 - static_cast<double>(wheel_movement) * 0.1);
        m_distance = std::clamp(m_distance, m_min_distance, m_max_distance);
        update_position();
    }

    void update_position() {
        const double horizontal_distance = m_distance * std::cos(m_pitch_rad);
        m_camera.position = cad::Point3{
            m_camera.target.x + horizontal_distance * std::sin(m_yaw_rad),
            m_camera.target.y + m_distance * std::sin(m_pitch_rad),
            m_camera.target.z + horizontal_distance * std::cos(m_yaw_rad)
        };
    }

    [[nodiscard]] const CameraState& camera() const { return m_camera; }
};
