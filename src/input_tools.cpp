#include "input_tools.h"

#include "core.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {

Vector3 to_raylib(const Point3D& point) {
    return {
        static_cast<float>(point.x),
        static_cast<float>(point.y),
        static_cast<float>(point.z)
    };
}

Vector3 normalized(Vector3 vector) {
    const float length = std::sqrt(
        vector.x * vector.x + vector.y * vector.y + vector.z * vector.z
    );
    if (length == 0.0f) {
        return {};
    }
    return {vector.x / length, vector.y / length, vector.z / length};
}

} // namespace

ControlPointSelectionTool::ControlPointSelectionTool(
    SelectionHandler on_selection,
    ClearHandler on_clear,
    float hit_radius
)
    : m_on_selection(std::move(on_selection)),
      m_on_clear(std::move(on_clear)),
      m_hit_radius(std::max(0.0f, hit_radius)) {}

void ControlPointSelectionTool::process_input(
    const InputFrameSnapshot& input,
    OrbitCameraController& camera_controller,
    Scene& scene
) {
    if (!input.left_mouse_pressed) {
        return;
    }

    if (input.screen_width <= 0 || input.screen_height <= 0) {
        return;
    }

    NurbsSurface* selected_surface = nullptr;
    ControlPoint* selected_point = nullptr;
    size_t selected_u = 0;
    size_t selected_v = 0;
    const float hit_radius_squared = m_hit_radius * m_hit_radius;
    float closest_screen_distance_squared = std::numeric_limits<float>::max();
    float closest_depth = std::numeric_limits<float>::max();

    const Camera3D& camera = camera_controller.raw_camera();
    const Vector3 camera_forward = normalized(Vector3{
        camera.target.x - camera.position.x,
        camera.target.y - camera.position.y,
        camera.target.z - camera.position.z
    });

    for (const auto& node : scene.nodes()) {
        if (!node.visible || !node.surface) {
            continue;
        }

        auto control_net = node.surface->control_net_2d();
        for (size_t u = 0; u < control_net.extent(0); ++u) {
            for (size_t v = 0; v < control_net.extent(1); ++v) {
                ControlPoint& point = control_net[u, v];
                const Vector3 world_position = to_raylib(point.position);
                const float depth =
                    (world_position.x - camera.position.x) * camera_forward.x +
                    (world_position.y - camera.position.y) * camera_forward.y +
                    (world_position.z - camera.position.z) * camera_forward.z;
                if (depth <= 0.0f) {
                    continue;
                }

                const Vector2 screen_position = GetWorldToScreenEx(
                    world_position,
                    camera,
                    input.screen_width,
                    input.screen_height
                );
                const float delta_x = screen_position.x - input.mouse_position.x;
                const float delta_y = screen_position.y - input.mouse_position.y;
                const float screen_distance_squared = delta_x * delta_x + delta_y * delta_y;
                if (screen_distance_squared > hit_radius_squared) {
                    continue;
                }

                const bool closer_in_depth = depth < closest_depth;
                const bool same_depth = std::abs(depth - closest_depth) <= 0.001f;

                if (!closer_in_depth &&
                    !(same_depth && screen_distance_squared < closest_screen_distance_squared)) {
                    continue;
                }

                selected_surface = node.surface.get();
                selected_point = &point;
                selected_u = u;
                selected_v = v;
                closest_screen_distance_squared = screen_distance_squared;
                closest_depth = depth;
            }
        }
    }

    if (selected_point != nullptr) {
        if (m_on_selection) {
            m_on_selection(*selected_surface, selected_u, selected_v, *selected_point);
        }
    } else if (m_on_clear) {
        m_on_clear();
    }
}