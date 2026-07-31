#pragma once

#include <algorithm>
#include <cmath>

#include "raylib.h"

class OrbitCameraController {
private:
    Camera3D m_raylib_camera{};
    float m_yaw_rad{0.0f};
    float m_pitch_rad{0.0f};
    float m_distance{10.0f};

    float m_orbit_sensitivity{0.005f};
    float m_pitch_limit_rad{1.5f};
    float m_min_distance{1.0f};
    float m_max_distance{100000.0f};

public:
    OrbitCameraController(Vector3 init_position, Vector3 init_target, float fov = 45.0f) {
        m_raylib_camera.position = init_position;
        m_raylib_camera.target = init_target;
        m_raylib_camera.up = Vector3{0.0f, 1.0f, 0.0f};
        m_raylib_camera.fovy = fov;
        m_raylib_camera.projection = CAMERA_PERSPECTIVE;

        Vector3 offset{
            init_position.x - init_target.x,
            init_position.y - init_target.y,
            init_position.z - init_target.z
        };
        m_distance = std::sqrt(offset.x * offset.x + offset.y * offset.y + offset.z * offset.z);
        if (m_distance < m_min_distance) {
            m_distance = m_min_distance;
            m_yaw_rad = 0.0f;
            m_pitch_rad = 0.0f;
            update_position();
            return;
        }
        m_yaw_rad = std::atan2(offset.x, offset.z);
        m_pitch_rad = std::asin(offset.y / m_distance);
    }  

    void orbit(Vector2 mouse_delta) {
        m_yaw_rad -= mouse_delta.x * m_orbit_sensitivity;
        m_pitch_rad += mouse_delta.y * m_orbit_sensitivity;
        m_pitch_rad = std::clamp(m_pitch_rad, -m_pitch_limit_rad, m_pitch_limit_rad);
        update_position();
    }

    void pan(Vector2 mouse_delta, float screen_height) {
        const Vector3 forward_dir = normalized(Vector3{
            m_raylib_camera.target.x - m_raylib_camera.position.x,
            m_raylib_camera.target.y - m_raylib_camera.position.y,
            m_raylib_camera.target.z - m_raylib_camera.position.z
        });
        const Vector3 right_dir = normalized(cross(forward_dir, m_raylib_camera.up));
        const Vector3 screen_up_dir = normalized(cross(right_dir, forward_dir));

        float pan_scale = 2.0f * m_distance * std::tan(m_raylib_camera.fovy * DEG2RAD * 0.5f) / screen_height;

        m_raylib_camera.target.x +=
            right_dir.x * -mouse_delta.x * pan_scale + screen_up_dir.x * mouse_delta.y * pan_scale;
        m_raylib_camera.target.y +=
            right_dir.y * -mouse_delta.x * pan_scale + screen_up_dir.y * mouse_delta.y * pan_scale;
        m_raylib_camera.target.z +=
            right_dir.z * -mouse_delta.x * pan_scale + screen_up_dir.z * mouse_delta.y * pan_scale;
        update_position();
    }

    void zoom(float wheel_movement) {
        if (wheel_movement == 0.0f) return;

        m_distance *= (1.0f - wheel_movement * 0.1f);
        m_distance = std::clamp(m_distance, m_min_distance, m_max_distance);
        update_position();
    }

    void update_position() {
        float horizontal_distance = m_distance * std::cos(m_pitch_rad);
        m_raylib_camera.position = Vector3{
            m_raylib_camera.target.x + horizontal_distance * std::sin(m_yaw_rad),
            m_raylib_camera.target.y + m_distance * std::sin(m_pitch_rad),
            m_raylib_camera.target.z + horizontal_distance * std::cos(m_yaw_rad)
        };
    }

    [[nodiscard]] const Camera3D& raw_camera() const { return m_raylib_camera; }

private:
    [[nodiscard]] static Vector3 cross(Vector3 left, Vector3 right) {
        return {
            left.y * right.z - left.z * right.y,
            left.z * right.x - left.x * right.z,
            left.x * right.y - left.y * right.x
        };
    }

    [[nodiscard]] static Vector3 normalized(Vector3 vector) {
        const float length = std::sqrt(
            vector.x * vector.x + vector.y * vector.y + vector.z * vector.z
        );
        if (length == 0.0f) {
            return {};
        }
        return {vector.x / length, vector.y / length, vector.z / length};
    }
};