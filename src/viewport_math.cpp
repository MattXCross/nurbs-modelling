#include "viewport_math.h"

#include <cmath>
#include <numbers>

std::optional<cad::Ray3> make_viewport_ray(
    Vec2 viewport_position,
    int viewport_width,
    int viewport_height,
    const CameraState& camera
) noexcept {
    if (viewport_width <= 0 || viewport_height <= 0 ||
        !std::isfinite(viewport_position.x) || !std::isfinite(viewport_position.y) ||
        !std::isfinite(camera.vertical_fov_degrees) ||
        camera.vertical_fov_degrees <= 0.0f || camera.vertical_fov_degrees >= 180.0f) {
        return std::nullopt;
    }
    const auto forward = cad::normalized(camera.target - camera.position);
    if (!forward) return std::nullopt;
    const auto right = cad::normalized(cad::cross(*forward, camera.up));
    if (!right) return std::nullopt;
    const auto screen_up = cad::normalized(cad::cross(*right, *forward));
    if (!screen_up) return std::nullopt;

    const double normalized_x = 2.0 * viewport_position.x / viewport_width - 1.0;
    const double normalized_y = 1.0 - 2.0 * viewport_position.y / viewport_height;
    const double half_height = std::tan(
        static_cast<double>(camera.vertical_fov_degrees) * std::numbers::pi / 360.0
    );
    const double aspect = static_cast<double>(viewport_width) / viewport_height;
    return cad::Ray3::from_origin_direction(
        camera.position,
        *forward + *right * (normalized_x * aspect * half_height) +
            *screen_up * (normalized_y * half_height)
    );
}
