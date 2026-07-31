#pragma once

#include <algorithm>
#include <cmath>

#include "math_types.h"

struct CameraState {
    Vec3 position{};
    Vec3 target{};
    Vec3 up{0.0f, 1.0f, 0.0f};
    float vertical_fov_degrees{45.0f};
};

class OrbitCameraController {
private:
    CameraState m_camera{};
    float m_yaw_rad{0.0f};
    float m_pitch_rad{0.0f};
    float m_distance{10.0f};

    float m_orbit_sensitivity{0.005f};
    float m_pitch_limit_rad{1.5f};
    float m_min_distance{1.0f};
    float m_max_distance{100000.0f};

public:
    OrbitCameraController(Vec3 init_position, Vec3 init_target, float fov = 45.0f) {
        m_camera.position = init_position;
        m_camera.target = init_target;
        m_camera.vertical_fov_degrees = fov;

        Vec3 offset{
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

    void orbit(Vec2 mouse_delta) {
        m_yaw_rad -= mouse_delta.x * m_orbit_sensitivity;
        m_pitch_rad += mouse_delta.y * m_orbit_sensitivity;
        m_pitch_rad = std::clamp(m_pitch_rad, -m_pitch_limit_rad, m_pitch_limit_rad);
        update_position();
    }

    void pan(Vec2 mouse_delta, float viewport_height) {
        const Vec3 forward_dir = normalized(Vec3{
            m_camera.target.x - m_camera.position.x,
            m_camera.target.y - m_camera.position.y,
            m_camera.target.z - m_camera.position.z
        });
        const Vec3 right_dir = normalized(cross(forward_dir, m_camera.up));
        const Vec3 screen_up_dir = normalized(cross(right_dir, forward_dir));

        constexpr float degrees_to_radians = 3.14159265358979323846f / 180.0f;
        const float safe_height = std::max(viewport_height, 1.0f);
        const float pan_scale = 2.0f * m_distance *
            std::tan(m_camera.vertical_fov_degrees * degrees_to_radians * 0.5f) / safe_height;

        m_camera.target.x +=
            right_dir.x * -mouse_delta.x * pan_scale + screen_up_dir.x * mouse_delta.y * pan_scale;
        m_camera.target.y +=
            right_dir.y * -mouse_delta.x * pan_scale + screen_up_dir.y * mouse_delta.y * pan_scale;
        m_camera.target.z +=
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
        m_camera.position = Vec3{
            m_camera.target.x + horizontal_distance * std::sin(m_yaw_rad),
            m_camera.target.y + m_distance * std::sin(m_pitch_rad),
            m_camera.target.z + horizontal_distance * std::cos(m_yaw_rad)
        };
    }

    [[nodiscard]] const CameraState& camera() const { return m_camera; }

private:
    [[nodiscard]] static Vec3 cross(Vec3 left, Vec3 right) {
        return {
            left.y * right.z - left.z * right.y,
            left.z * right.x - left.x * right.z,
            left.x * right.y - left.y * right.x
        };
    }

    [[nodiscard]] static Vec3 normalized(Vec3 vector) {
        const float length = std::sqrt(
            vector.x * vector.x + vector.y * vector.y + vector.z * vector.z
        );
        if (length == 0.0f) {
            return {};
        }
        return {vector.x / length, vector.y / length, vector.z / length};
    }
};
