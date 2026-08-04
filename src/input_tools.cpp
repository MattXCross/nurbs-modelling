#include "input_tools.h"

#include "kernel_math.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <tuple>
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

double gizmo_world_scale(
    cad::Point3 pivot,
    const CameraState& camera,
    int viewport_height
) {
    if (viewport_height <= 0) {
        return 0.0;
    }
    const double depth = cad::distance(camera.position, pivot);
    return depth * 2.0 * std::tan(
        static_cast<double>(camera.vertical_fov_degrees) * std::numbers::pi / 360.0
    ) * 80.0 / static_cast<double>(viewport_height);
}

double point_segment_distance(Vec2 point, Vec2 start, Vec2 end) {
    const double x = static_cast<double>(end.x - start.x);
    const double y = static_cast<double>(end.y - start.y);
    const double length_squared = x * x + y * y;
    if (length_squared == 0.0) {
        return std::hypot(point.x - start.x, point.y - start.y);
    }
    const double parameter = std::clamp(
        (static_cast<double>(point.x - start.x) * x +
         static_cast<double>(point.y - start.y) * y) / length_squared,
        0.0,
        1.0
    );
    return std::hypot(
        static_cast<double>(point.x - start.x) - parameter * x,
        static_cast<double>(point.y - start.y) - parameter * y
    );
}

std::optional<cad::Point3> ray_plane_point(
    const cad::Ray3& ray,
    cad::Point3 plane_point,
    cad::Vector3 plane_normal
) {
    const double denominator = cad::dot(ray.direction(), plane_normal);
    if (!std::isfinite(denominator) || std::abs(denominator) <= 1e-12) {
        return std::nullopt;
    }
    const double parameter = cad::dot(plane_point - ray.origin(), plane_normal) / denominator;
    if (!std::isfinite(parameter) || parameter < 0.0) {
        return std::nullopt;
    }
    return ray.at(parameter);
}

cad::Vector3 axis_vector(TranslationConstraint constraint, const TransformFrame& frame) {
    switch (constraint) {
        case TranslationConstraint::x: return frame.x;
        case TranslationConstraint::y: return frame.y;
        case TranslationConstraint::z: return frame.z;
        default: return {};
    }
}

cad::Vector3 plane_normal(
    TranslationConstraint constraint,
    const TransformFrame& frame,
    const CameraState& camera
) {
    switch (constraint) {
        case TranslationConstraint::xy: return frame.z;
        case TranslationConstraint::xz: return frame.y;
        case TranslationConstraint::yz: return frame.x;
        case TranslationConstraint::screen:
            return cad::normalized(camera.target - camera.position).value_or(cad::Vector3{});
        default: return {};
    }
}

cad::Vector3 snapped(cad::Vector3 delta, double increment) {
    if (!std::isfinite(increment) || increment <= 0.0) {
        return delta;
    }
    const auto snap = [increment](double value) {
        return std::round(value / increment) * increment;
    };
    return {snap(delta.x), snap(delta.y), snap(delta.z)};
}

cad::Vector3 grid_snapped(cad::Point3 pivot, cad::Vector3 delta, double increment) {
    if (!std::isfinite(increment) || increment <= 0.0) {
        return delta;
    }
    const auto snap = [increment](double value) {
        return std::round(value / increment) * increment;
    };
    const cad::Point3 target = pivot + delta;
    return {
        snap(target.x) - pivot.x,
        snap(target.y) - pivot.y,
        snap(target.z) - pivot.z
    };
}

cad::Vector3 increment_snapped(
    cad::Vector3 delta,
    TranslationConstraint constraint,
    const TransformFrame& frame,
    double increment
) {
    const auto scalar = [increment](double value) {
        return std::round(value / increment) * increment;
    };
    switch (constraint) {
        case TranslationConstraint::x: return frame.x * scalar(cad::dot(delta, frame.x));
        case TranslationConstraint::y: return frame.y * scalar(cad::dot(delta, frame.y));
        case TranslationConstraint::z: return frame.z * scalar(cad::dot(delta, frame.z));
        case TranslationConstraint::xy:
            return frame.x * scalar(cad::dot(delta, frame.x)) +
                frame.y * scalar(cad::dot(delta, frame.y));
        case TranslationConstraint::xz:
            return frame.x * scalar(cad::dot(delta, frame.x)) +
                frame.z * scalar(cad::dot(delta, frame.z));
        case TranslationConstraint::yz:
            return frame.y * scalar(cad::dot(delta, frame.y)) +
                frame.z * scalar(cad::dot(delta, frame.z));
        case TranslationConstraint::screen: return snapped(delta, increment);
    }
    return delta;
}

std::pair<cad::Vector3, cad::Vector3> ring_basis(cad::Vector3 normal) {
    const cad::Vector3 reference = std::abs(normal.y) < 0.9
        ? cad::Vector3{0.0, 1.0, 0.0}
        : cad::Vector3{1.0, 0.0, 0.0};
    const cad::Vector3 first = cad::normalized(cad::cross(normal, reference))
        .value_or(cad::Vector3{1.0, 0.0, 0.0});
    const cad::Vector3 second = cad::cross(normal, first);
    return {first, second};
}

double ring_screen_distance(
    Vec2 mouse,
    cad::Point3 pivot,
    cad::Vector3 normal,
    double radius,
    const CameraState& camera,
    int width,
    int height
) {
    const auto [first, second] = ring_basis(normal);
    double closest = std::numeric_limits<double>::max();
    constexpr std::size_t segments = 64;
    Vec2 previous = project_to_viewport(
        pivot + first * radius,
        camera,
        width,
        height
    );
    for (std::size_t index = 1; index <= segments; ++index) {
        const double angle = 2.0 * std::numbers::pi * static_cast<double>(index) /
            static_cast<double>(segments);
        const Vec2 current = project_to_viewport(
            pivot + (first * std::cos(angle) + second * std::sin(angle)) * radius,
            camera,
            width,
            height
        );
        closest = std::min(closest, point_segment_distance(mouse, previous, current));
        previous = current;
    }
    return closest;
}

} // namespace

TranslationTool::TranslationTool(
    ActiveHandler active,
    PivotHandler pivot,
    BeginHandler begin,
    PreviewHandler preview,
    FinishHandler finish,
    FinishHandler cancel,
    double snap_increment
)
    : m_active(std::move(active)),
      m_pivot(std::move(pivot)),
      m_begin(std::move(begin)),
      m_preview(std::move(preview)),
      m_finish(std::move(finish)),
      m_cancel(std::move(cancel)),
      m_snap_increment(snap_increment) {}

void TranslationTool::process_input(
    const InputFrameSnapshot& input,
    OrbitCameraController& camera_controller,
    Scene&
) {
    if (input.escape_pressed && m_active && m_active()) {
        m_dragging = false;
        if (m_cancel) {
            m_cancel();
        }
        return;
    }

    const CameraState& camera = camera_controller.camera();
    if (input.left_mouse_pressed && m_active && !m_active() && m_pivot && m_begin) {
        const auto frame = m_pivot();
        if (!frame || input.screen_width <= 0 || input.screen_height <= 0) {
            return;
        }
        const cad::Point3 pivot = frame->pivot;
        const double scale = gizmo_world_scale(pivot, camera, input.screen_height);
        if (!std::isfinite(scale) || scale <= 0.0) {
            return;
        }
        const Vec2 center = project_to_viewport(
            pivot,
            camera,
            input.screen_width,
            input.screen_height
        );
        std::optional<TranslationConstraint> hit;
        if (std::hypot(
                input.mouse_position.x - center.x,
                input.mouse_position.y - center.y
            ) <= 9.0) {
            hit = TranslationConstraint::screen;
        }
        const auto projected = [&](cad::Vector3 offset) {
            return project_to_viewport(
                pivot + offset * scale,
                camera,
                input.screen_width,
                input.screen_height
            );
        };
        if (!hit) {
            for (const auto& [constraint, first, second] : {
                     std::tuple{TranslationConstraint::xy, frame->x, frame->y},
                     std::tuple{TranslationConstraint::xz, frame->x, frame->z},
                     std::tuple{TranslationConstraint::yz, frame->y, frame->z}
                 }) {
                const Vec2 handle = projected((first + second) * 0.28);
                if (std::hypot(
                        input.mouse_position.x - handle.x,
                        input.mouse_position.y - handle.y
                    ) <= 9.0) {
                    hit = constraint;
                    break;
                }
            }
        }
        if (!hit) {
            for (const TranslationConstraint constraint : {
                     TranslationConstraint::x,
                     TranslationConstraint::y,
                     TranslationConstraint::z
                 }) {
                const cad::Vector3 axis = axis_vector(constraint, *frame);
                if (point_segment_distance(
                        input.mouse_position,
                        projected(axis * 0.4),
                        projected(axis)
                    ) <= 8.0) {
                    hit = constraint;
                    break;
                }
            }
        }
        if (!hit || !m_begin(*hit)) {
            return;
        }

        m_constraint = *hit;
        m_start_pivot = pivot;
        m_frame = *frame;
        m_start_mouse = input.mouse_position;
        m_gizmo_scale = scale;
        m_dragging = true;
        const cad::Vector3 axis = axis_vector(*hit, m_frame);
        if (axis != cad::Vector3{}) {
            const Vec2 end = projected(axis);
            const double x = end.x - center.x;
            const double y = end.y - center.y;
            m_axis_screen_length = std::hypot(x, y);
            if (m_axis_screen_length <= 1e-6) {
                m_cancel();
                m_dragging = false;
                return;
            }
            m_axis_screen_direction = {
                static_cast<float>(x / m_axis_screen_length),
                static_cast<float>(y / m_axis_screen_length)
            };
        } else {
            m_plane_normal = plane_normal(*hit, m_frame, camera);
            const auto ray = make_viewport_ray(
                input.mouse_position,
                input.screen_width,
                input.screen_height,
                camera
            );
            const auto point = ray ? ray_plane_point(*ray, pivot, m_plane_normal) : std::nullopt;
            if (!point) {
                m_cancel();
                m_dragging = false;
                return;
            }
            m_start_plane_point = *point;
        }
        return;
    }

    if (!m_dragging || !m_active || !m_active()) {
        return;
    }
    if (input.left_mouse_released) {
        m_dragging = false;
        if (m_finish) {
            m_finish();
        }
        return;
    }

    cad::Vector3 delta;
    const cad::Vector3 axis = axis_vector(m_constraint, m_frame);
    if (axis != cad::Vector3{}) {
        const double pixels =
            static_cast<double>(input.mouse_position.x - m_start_mouse.x) *
                m_axis_screen_direction.x +
            static_cast<double>(input.mouse_position.y - m_start_mouse.y) *
                m_axis_screen_direction.y;
        delta = axis * (pixels * m_gizmo_scale / m_axis_screen_length);
    } else {
        const auto ray = make_viewport_ray(
            input.mouse_position,
            input.screen_width,
            input.screen_height,
            camera
        );
        const auto point = ray ? ray_plane_point(*ray, m_start_pivot, m_plane_normal) : std::nullopt;
        if (!point) {
            return;
        }
        delta = *point - m_start_plane_point;
        switch (m_constraint) {
            case TranslationConstraint::xy:
                delta = m_frame.x * cad::dot(delta, m_frame.x) +
                    m_frame.y * cad::dot(delta, m_frame.y);
                break;
            case TranslationConstraint::xz:
                delta = m_frame.x * cad::dot(delta, m_frame.x) +
                    m_frame.z * cad::dot(delta, m_frame.z);
                break;
            case TranslationConstraint::yz:
                delta = m_frame.y * cad::dot(delta, m_frame.y) +
                    m_frame.z * cad::dot(delta, m_frame.z);
                break;
            default: break;
        }
    }
    if (input.modifiers.ctrl) {
        delta = increment_snapped(delta, m_constraint, m_frame, m_snap_increment);
    } else if (input.modifiers.shift) {
        delta = grid_snapped(m_start_pivot, delta, m_snap_increment);
        switch (m_constraint) {
            case TranslationConstraint::x:
                delta = m_frame.x * cad::dot(delta, m_frame.x); break;
            case TranslationConstraint::y:
                delta = m_frame.y * cad::dot(delta, m_frame.y); break;
            case TranslationConstraint::z:
                delta = m_frame.z * cad::dot(delta, m_frame.z); break;
            case TranslationConstraint::xy:
                delta = m_frame.x * cad::dot(delta, m_frame.x) +
                    m_frame.y * cad::dot(delta, m_frame.y); break;
            case TranslationConstraint::xz:
                delta = m_frame.x * cad::dot(delta, m_frame.x) +
                    m_frame.z * cad::dot(delta, m_frame.z); break;
            case TranslationConstraint::yz:
                delta = m_frame.y * cad::dot(delta, m_frame.y) +
                    m_frame.z * cad::dot(delta, m_frame.z); break;
            default: break;
        }
    }
    if (m_preview) {
        (void)m_preview(delta);
    }
}

RotationTool::RotationTool(
    ActiveHandler active,
    PivotHandler pivot,
    BeginHandler begin,
    PreviewHandler preview,
    FinishHandler finish,
    FinishHandler cancel
)
    : m_active(std::move(active)),
      m_pivot(std::move(pivot)),
      m_begin(std::move(begin)),
      m_preview(std::move(preview)),
      m_finish(std::move(finish)),
      m_cancel(std::move(cancel)) {}

void RotationTool::process_input(
    const InputFrameSnapshot& input,
    OrbitCameraController& camera_controller,
    Scene&
) {
    if (input.escape_pressed && m_active && m_active()) {
        m_dragging = false;
        m_cancel();
        return;
    }
    const CameraState& camera = camera_controller.camera();
    if (input.left_mouse_pressed && m_active && !m_active() && m_pivot) {
        const auto frame = m_pivot();
        if (!frame || input.screen_width <= 0 || input.screen_height <= 0) {
            return;
        }
        const cad::Point3 pivot = frame->pivot;
        const double scale = gizmo_world_scale(pivot, camera, input.screen_height);
        const auto camera_axis = cad::normalized(camera.target - camera.position);
        if (!camera_axis || scale <= 0.0) {
            return;
        }
        std::optional<std::pair<RotationConstraint, cad::Vector3>> hit;
        double closest = 9.0;
        for (const auto& [constraint, axis, radius] : {
                 std::tuple{RotationConstraint::x, frame->x, scale},
                 std::tuple{RotationConstraint::y, frame->y, scale},
                 std::tuple{RotationConstraint::z, frame->z, scale},
                 std::tuple{RotationConstraint::screen, *camera_axis, scale * 1.18}
             }) {
            const double distance = ring_screen_distance(
                input.mouse_position,
                pivot,
                axis,
                radius,
                camera,
                input.screen_width,
                input.screen_height
            );
            if (distance < closest) {
                closest = distance;
                hit = std::pair{constraint, axis};
            }
        }
        if (!hit || !m_begin(hit->first, hit->second)) {
            return;
        }
        m_center = project_to_viewport(
            pivot,
            camera,
            input.screen_width,
            input.screen_height
        );
        m_start_angle = std::atan2(
            input.mouse_position.y - m_center.y,
            input.mouse_position.x - m_center.x
        );
        m_dragging = true;
        return;
    }
    if (!m_dragging || !m_active || !m_active()) {
        return;
    }
    if (input.left_mouse_released) {
        m_dragging = false;
        m_finish();
        return;
    }
    double angle = std::atan2(
        input.mouse_position.y - m_center.y,
        input.mouse_position.x - m_center.x
    ) - m_start_angle;
    if (input.modifiers.ctrl) {
        constexpr double increment = 15.0 * std::numbers::pi / 180.0;
        angle = std::round(angle / increment) * increment;
    }
    (void)m_preview(angle);
}

ScaleTool::ScaleTool(
    ActiveHandler active,
    PivotHandler pivot,
    BeginHandler begin,
    PreviewHandler preview,
    FinishHandler finish,
    FinishHandler cancel
)
    : m_active(std::move(active)),
      m_pivot(std::move(pivot)),
      m_begin(std::move(begin)),
      m_preview(std::move(preview)),
      m_finish(std::move(finish)),
      m_cancel(std::move(cancel)) {}

void ScaleTool::process_input(
    const InputFrameSnapshot& input,
    OrbitCameraController& camera_controller,
    Scene&
) {
    if (input.escape_pressed && m_active && m_active()) {
        m_dragging = false;
        m_cancel();
        return;
    }
    const CameraState& camera = camera_controller.camera();
    if (input.left_mouse_pressed && m_active && !m_active() && m_pivot) {
        const auto frame = m_pivot();
        if (!frame || input.screen_width <= 0 || input.screen_height <= 0) {
            return;
        }
        const cad::Point3 pivot = frame->pivot;
        const double scale = gizmo_world_scale(pivot, camera, input.screen_height);
        const Vec2 center = project_to_viewport(pivot, camera, input.screen_width, input.screen_height);
        std::optional<ScaleConstraint> hit;
        if (std::hypot(input.mouse_position.x - center.x, input.mouse_position.y - center.y) <= 9.0) {
            hit = ScaleConstraint::uniform;
        }
        if (!hit) {
            for (const auto& [constraint, axis] : {
                     std::pair{ScaleConstraint::x, frame->x},
                     std::pair{ScaleConstraint::y, frame->y},
                     std::pair{ScaleConstraint::z, frame->z}
                 }) {
                const Vec2 end = project_to_viewport(
                    pivot + axis * scale,
                    camera,
                    input.screen_width,
                    input.screen_height
                );
                if (point_segment_distance(input.mouse_position, center, end) <= 8.0) {
                    hit = constraint;
                    const double x = end.x - center.x;
                    const double y = end.y - center.y;
                    m_axis_screen_length = std::hypot(x, y);
                    if (m_axis_screen_length > 1e-6) {
                        m_axis_screen_direction = {
                            static_cast<float>(x / m_axis_screen_length),
                            static_cast<float>(y / m_axis_screen_length)
                        };
                    }
                    break;
                }
            }
        }
        if (!hit || m_axis_screen_length <= 1e-6 || !m_begin(*hit)) {
            return;
        }
        m_constraint = *hit;
        m_frame = *frame;
        m_start_mouse = input.mouse_position;
        m_dragging = true;
        return;
    }
    if (!m_dragging || !m_active || !m_active()) {
        return;
    }
    if (input.left_mouse_released) {
        m_dragging = false;
        m_finish();
        return;
    }
    double pixels = 0.0;
    if (m_constraint == ScaleConstraint::uniform) {
        pixels = static_cast<double>(input.mouse_position.x - m_start_mouse.x) -
            static_cast<double>(input.mouse_position.y - m_start_mouse.y);
    } else {
        pixels = static_cast<double>(input.mouse_position.x - m_start_mouse.x) *
                m_axis_screen_direction.x +
            static_cast<double>(input.mouse_position.y - m_start_mouse.y) *
                m_axis_screen_direction.y;
    }
    double factor = std::max(0.001, 1.0 + pixels / 80.0);
    if (input.modifiers.ctrl) {
        factor = std::max(0.1, std::round(factor * 10.0) / 10.0);
    }
    (void)m_preview(factor);
}

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
