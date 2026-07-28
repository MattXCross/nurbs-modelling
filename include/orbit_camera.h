#pragma once

#include <algorithm>
#include <cmath>

#include "raylib.h"
#include "raymath.h"

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

        Vector3 offset = Vector3Subtract(init_position, init_target);
        m_distance = Vector3Length(offset);
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
        Vector3 forward_dir = Vector3Normalize(Vector3Subtract(m_raylib_camera.target, m_raylib_camera.position));
        Vector3 right_dir = Vector3Normalize(Vector3CrossProduct(forward_dir, m_raylib_camera.up));
        Vector3 screen_up_dir = Vector3Normalize(Vector3CrossProduct(right_dir, forward_dir));

        float pan_scale = 2.0f * m_distance * std::tan(m_raylib_camera.fovy * DEG2RAD * 0.5f) / screen_height;

        Vector3 horizontal_pan = Vector3Scale(right_dir, -mouse_delta.x * pan_scale);
        Vector3 vertical_pan = Vector3Scale(screen_up_dir, mouse_delta.y * pan_scale);

        m_raylib_camera.target = Vector3Add(m_raylib_camera.target, Vector3Add(horizontal_pan, vertical_pan));
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
};