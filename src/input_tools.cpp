#include "input_tools.h"

#include "kernel_math.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace {

cad::Point3 at_render_precision(cad::Point3 point) {
    return {
        static_cast<double>(static_cast<float>(point.x)),
        static_cast<double>(static_cast<float>(point.y)),
        static_cast<double>(static_cast<float>(point.z))
    };
}

cad::Vector3 at_render_precision(cad::Vector3 vector) {
    return {
        static_cast<double>(static_cast<float>(vector.x)),
        static_cast<double>(static_cast<float>(vector.y)),
        static_cast<double>(static_cast<float>(vector.z))
    };
}

Vec2 project_to_viewport(
    cad::Point3 world_position,
    const CameraState& camera,
    int viewport_width,
    int viewport_height
) {
    constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
    const cad::Point3 camera_position = at_render_precision(camera.position);
    const cad::Point3 camera_target = at_render_precision(camera.target);
    const cad::Vector3 camera_up = at_render_precision(camera.up);
    world_position = at_render_precision(world_position);
    const cad::Vector3 forward =
        cad::normalized(camera_target - camera_position).value_or(cad::Vector3{});
    const cad::Vector3 right =
        cad::normalized(cad::cross(forward, camera_up)).value_or(cad::Vector3{});
    const cad::Vector3 screen_up =
        cad::normalized(cad::cross(right, forward)).value_or(cad::Vector3{});
    const cad::Vector3 offset = world_position - camera_position;
    const double depth = cad::dot(offset, forward);
    const double half_height = depth *
        std::tan(static_cast<double>(camera.vertical_fov_degrees) *
                 degrees_to_radians * 0.5);
    const double aspect = static_cast<double>(viewport_width) /
        static_cast<double>(viewport_height);
    const double normalized_x = cad::dot(offset, right) / (half_height * aspect);
    const double normalized_y = cad::dot(offset, screen_up) / half_height;

    return {
        static_cast<float>((normalized_x + 1.0) * 0.5 * static_cast<double>(viewport_width)),
        static_cast<float>((1.0 - normalized_y) * 0.5 * static_cast<double>(viewport_height))
    };
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

    std::optional<ControlPointSelection> selected_point;
    const float hit_radius_squared = m_hit_radius * m_hit_radius;
    float closest_screen_distance_squared = std::numeric_limits<float>::max();
    double closest_depth = std::numeric_limits<double>::max();

    const CameraState& camera = camera_controller.camera();
    const cad::Point3 camera_position = at_render_precision(camera.position);
    const cad::Point3 camera_target = at_render_precision(camera.target);
    const cad::Vector3 camera_forward =
        cad::normalized(camera_target - camera_position).value_or(cad::Vector3{});

    for (const auto& node : scene.nodes()) {
        if (!node.visible || !node.surface) {
            continue;
        }

        const auto control_net = node.surface->control_net_2d();
        for (size_t u = 0; u < control_net.extent(0); ++u) {
            for (size_t v = 0; v < control_net.extent(1); ++v) {
                const ControlPoint& point = control_net[u, v];
                const cad::Point3 world_position = at_render_precision(point.position);
                const double depth = cad::dot(world_position - camera_position, camera_forward);
                if (depth <= 0.0) {
                    continue;
                }

                const Vec2 screen_position = project_to_viewport(
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
                const bool same_depth = std::abs(depth - closest_depth) <= 0.001;

                if (!closer_in_depth &&
                    !(same_depth && screen_distance_squared < closest_screen_distance_squared)) {
                    continue;
                }

                selected_point = ControlPointSelection{node.id, u, v};
                closest_screen_distance_squared = screen_distance_squared;
                closest_depth = depth;
            }
        }
    }

    if (selected_point.has_value()) {
        if (m_on_selection) {
            m_on_selection(*selected_point);
        }
    } else if (m_on_clear) {
        m_on_clear();
    }
}
