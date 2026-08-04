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
    EnabledHandler enabled,
    SelectionHandler on_selection,
    RectangleHandler on_rectangle,
    ClearHandler on_clear,
    float hit_radius
)
    : m_enabled(std::move(enabled)),
      m_on_selection(std::move(on_selection)),
      m_on_rectangle(std::move(on_rectangle)),
      m_on_clear(std::move(on_clear)),
       m_hit_radius(std::max(0.0f, hit_radius)) {}

SurfaceSelectionTool::SurfaceSelectionTool(
    EnabledHandler enabled,
    SelectionHandler on_selection,
    HoverHandler on_hover,
    ClearHandler on_clear
)
    : m_enabled(std::move(enabled)),
      m_on_selection(std::move(on_selection)),
      m_on_hover(std::move(on_hover)),
      m_on_clear(std::move(on_clear)) {}

void SurfaceSelectionTool::process_input(
    const InputFrameSnapshot& input,
    OrbitCameraController& camera_controller,
    Scene& scene
) {
    if (!m_enabled || !m_enabled()) {
        return;
    }
    const auto ray = make_viewport_ray(
        input.mouse_position,
        input.screen_width,
        input.screen_height,
        camera_controller.camera()
    );
    const std::vector<SurfacePickHit> hits = ray ? pick_surfaces(scene, *ray) :
        std::vector<SurfacePickHit>{};
    if (m_on_hover) {
        m_on_hover(hits.empty() ? std::nullopt : std::optional{hits.front().entity});
    }
    if (!input.left_mouse_pressed) {
        return;
    }
    if (hits.empty()) {
        m_last_hits.clear();
        m_has_last_click = false;
        if (m_on_clear) {
            m_on_clear();
        }
        return;
    }

    std::vector<EntityId> entities;
    entities.reserve(hits.size());
    for (const SurfacePickHit& hit : hits) {
        entities.push_back(hit.entity);
    }
    const float delta_x = input.mouse_position.x - m_last_click_position.x;
    const float delta_y = input.mouse_position.y - m_last_click_position.y;
    const bool same_location = m_has_last_click && delta_x * delta_x + delta_y * delta_y <= 16.0f;
    if (same_location && entities == m_last_hits) {
        m_cycle_index = (m_cycle_index + 1) % entities.size();
    } else {
        m_cycle_index = 0;
    }
    m_last_hits = entities;
    m_last_click_position = input.mouse_position;
    m_has_last_click = true;
    if (m_on_selection) {
        m_on_selection(EntitySelection{entities[m_cycle_index]});
    }
}

void ControlPointSelectionTool::process_input(
    const InputFrameSnapshot& input,
    OrbitCameraController& camera_controller,
    Scene& scene
) {
    if (!m_enabled || !m_enabled()) {
        m_selecting = false;
        return;
    }

    if (input.left_mouse_pressed) {
        m_press_position = input.mouse_position;
        m_selecting = true;
        return;
    }
    if (!input.left_mouse_released || !m_selecting) {
        return;
    }
    m_selecting = false;

    if (input.screen_width <= 0 || input.screen_height <= 0) {
        return;
    }

    const float drag_x = input.mouse_position.x - m_press_position.x;
    const float drag_y = input.mouse_position.y - m_press_position.y;
    if (drag_x * drag_x + drag_y * drag_y > 16.0f) {
        const float minimum_x = std::min(m_press_position.x, input.mouse_position.x);
        const float minimum_y = std::min(m_press_position.y, input.mouse_position.y);
        const Rect rectangle{
            minimum_x,
            minimum_y,
            std::abs(drag_x),
            std::abs(drag_y)
        };
        std::vector<ControlPointSelection> selected_points;
        const CameraState& camera = camera_controller.camera();
        const cad::Vector3 camera_forward = cad::normalized(
            at_render_precision(camera.target) - at_render_precision(camera.position)
        ).value_or(cad::Vector3{});
        for (const SceneNode& node : scene.nodes()) {
            if (!node.visible || node.surface == nullptr) {
                continue;
            }
            const auto net = node.surface->control_net_2d();
            for (std::size_t u = 0; u < net.extent(0); ++u) {
                for (std::size_t v = 0; v < net.extent(1); ++v) {
                    const cad::Point3 position = at_render_precision(net[u, v].position);
                    if (cad::dot(position - at_render_precision(camera.position), camera_forward) > 0.0 &&
                        rectangle.contains(project_to_viewport(
                            position,
                            camera,
                            input.screen_width,
                            input.screen_height
                        ))) {
                        selected_points.push_back(ControlPointSelection{node.id, u, v});
                    }
                }
            }
        }
        if (m_on_rectangle) {
            m_on_rectangle(std::move(selected_points), input.modifiers);
        }
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
            m_on_selection(*selected_point, input.modifiers);
        }
    } else if (!input.modifiers.shift && !input.modifiers.ctrl && m_on_clear) {
        m_on_clear();
    }
}
