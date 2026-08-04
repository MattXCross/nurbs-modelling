#include "transform_gizmos.h"

#include "viewport_math.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <tuple>
#include <utility>

namespace {

constexpr GizmoColor red{230, 41, 55, 255};
constexpr GizmoColor green{0, 228, 48, 255};
constexpr GizmoColor blue{0, 121, 241, 255};
constexpr GizmoColor yellow{253, 249, 0, 255};
constexpr GizmoColor light_gray{200, 200, 200, 255};
constexpr GizmoColor plane_xy{253, 249, 0, 166};
constexpr GizmoColor plane_xz{255, 0, 255, 166};
constexpr GizmoColor plane_yz{102, 191, 255, 166};

double gizmo_world_scale(cad::Point3 pivot, const CameraState& camera, int viewport_height) {
    if (viewport_height <= 0) {
        return 0.0;
    }
    const double vertical_size = camera.projection == ProjectionMode::orthographic
        ? camera.orthographic_vertical_size
        : cad::distance(camera.position, pivot) * 2.0 * std::tan(
            static_cast<double>(camera.vertical_fov_degrees) * std::numbers::pi / 360.0
        );
    return vertical_size * 80.0 / static_cast<double>(viewport_height);
}

double point_segment_distance(Vec2 point, Vec2 start, Vec2 end) {
    const double x = end.x - start.x;
    const double y = end.y - start.y;
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
    return {snap(target.x) - pivot.x, snap(target.y) - pivot.y, snap(target.z) - pivot.z};
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
    return {first, cad::cross(normal, first)};
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
    Vec2 previous = project_to_viewport(pivot + first * radius, camera, width, height);
    for (std::size_t index = 1; index <= segments; ++index) {
        const double angle = 2.0 * std::numbers::pi * index / segments;
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

GizmoColor active_color(bool active, GizmoColor normal) {
    return active ? yellow : normal;
}

} // namespace

TranslationGizmo::TranslationGizmo(
    ActiveHandler active,
    FrameHandler frame,
    BeginHandler begin,
    PreviewHandler preview,
    FinishHandler finish,
    FinishHandler cancel,
    double snap_increment
)
    : m_active(std::move(active)),
      m_frame_provider(std::move(frame)),
      m_begin(std::move(begin)),
      m_preview(std::move(preview)),
      m_finish(std::move(finish)),
      m_cancel(std::move(cancel)),
      m_snap_increment(snap_increment) {}

bool TranslationGizmo::process_input(
    const InputFrameSnapshot& input,
    const CameraState& camera
) {
    const std::optional<TranslationConstraint> active = m_active ? m_active() : std::nullopt;
    if (input.escape_pressed && active) {
        m_dragging = false;
        if (m_cancel) m_cancel();
        return true;
    }
    if (input.left_mouse_pressed && !active && m_frame_provider && m_begin) {
        const auto frame = m_frame_provider();
        if (!frame || input.screen_width <= 0 || input.screen_height <= 0) return false;
        const cad::Point3 pivot = frame->pivot;
        const double scale = gizmo_world_scale(pivot, camera, input.screen_height);
        if (!std::isfinite(scale) || scale <= 0.0) return false;
        const Vec2 center = project_to_viewport(
            pivot, camera, input.screen_width, input.screen_height
        );
        const auto projected = [&](cad::Vector3 offset) {
            return project_to_viewport(
                pivot + offset * scale, camera, input.screen_width, input.screen_height
            );
        };
        std::optional<TranslationConstraint> hit;
        if (std::hypot(input.mouse_position.x - center.x, input.mouse_position.y - center.y) <= 9.0) {
            hit = TranslationConstraint::screen;
        }
        if (!hit) {
            for (const auto& [constraint, first, second] : {
                     std::tuple{TranslationConstraint::xy, frame->x, frame->y},
                     std::tuple{TranslationConstraint::xz, frame->x, frame->z},
                     std::tuple{TranslationConstraint::yz, frame->y, frame->z}
                 }) {
                const Vec2 handle = projected((first + second) * 0.28);
                if (std::hypot(input.mouse_position.x - handle.x,
                               input.mouse_position.y - handle.y) <= 9.0) {
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
                        input.mouse_position, projected(axis * 0.4), projected(axis)
                    ) <= 8.0) {
                    hit = constraint;
                    break;
                }
            }
        }
        if (!hit || !m_begin(*hit)) return false;

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
                return true;
            }
            m_axis_screen_direction = {
                static_cast<float>(x / m_axis_screen_length),
                static_cast<float>(y / m_axis_screen_length)
            };
        } else {
            m_plane_normal = plane_normal(*hit, m_frame, camera);
            const auto ray = make_viewport_ray(
                input.mouse_position, input.screen_width, input.screen_height, camera
            );
            const auto point = ray
                ? ray_plane_point(*ray, pivot, m_plane_normal)
                : std::nullopt;
            if (!point) {
                m_cancel();
                m_dragging = false;
                return true;
            }
            m_start_plane_point = *point;
        }
        return true;
    }
    if (!m_dragging || !active) return false;
    if (input.left_mouse_released) {
        m_dragging = false;
        if (m_finish) m_finish();
        return true;
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
            input.mouse_position, input.screen_width, input.screen_height, camera
        );
        const auto point = ray
            ? ray_plane_point(*ray, m_start_pivot, m_plane_normal)
            : std::nullopt;
        if (!point) return true;
        delta = *point - m_start_plane_point;
        switch (m_constraint) {
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
    if (m_preview) (void)m_preview(delta);
    return true;
}

void TranslationGizmo::append_draw_data(
    GizmoDrawList& draw_list,
    const CameraState& camera,
    int viewport_height
) const {
    const auto frame = m_frame_provider ? m_frame_provider() : std::nullopt;
    if (!frame) return;
    const double scale = gizmo_world_scale(frame->pivot, camera, viewport_height);
    if (!std::isfinite(scale) || scale <= 0.0) return;
    const auto active = m_active ? m_active() : std::nullopt;
    draw_list.emplace_back(GizmoCenterHandle{
        frame->pivot, scale * 0.08, GizmoCenterShape::sphere,
        active_color(active == TranslationConstraint::screen, light_gray)
    });
    for (const auto& [constraint, axis, color] : {
             std::tuple{TranslationConstraint::x, frame->x, red},
             std::tuple{TranslationConstraint::y, frame->y, green},
             std::tuple{TranslationConstraint::z, frame->z, blue}
         }) {
        draw_list.emplace_back(GizmoArrow{
            frame->pivot,
            frame->pivot + axis * (scale * 0.78),
            frame->pivot + axis * (scale * 0.72),
            frame->pivot + axis * scale,
            scale * 0.025,
            scale * 0.09,
            active_color(active == constraint, color)
        });
    }
    for (const auto& [constraint, first, second, color] : {
             std::tuple{TranslationConstraint::xy, frame->x, frame->y, plane_xy},
             std::tuple{TranslationConstraint::xz, frame->x, frame->z, plane_xz},
             std::tuple{TranslationConstraint::yz, frame->y, frame->z, plane_yz}
         }) {
        draw_list.emplace_back(GizmoPlaneHandle{
            frame->pivot + first * (scale * 0.18),
            frame->pivot + (first + second) * (scale * 0.38),
            frame->pivot + second * (scale * 0.18),
            active_color(active == constraint, color)
        });
    }
}

RotationGizmo::RotationGizmo(
    ActiveHandler active,
    FrameHandler frame,
    BeginHandler begin,
    PreviewHandler preview,
    FinishHandler finish,
    FinishHandler cancel
)
    : m_active(std::move(active)),
      m_frame_provider(std::move(frame)),
      m_begin(std::move(begin)),
      m_preview(std::move(preview)),
      m_finish(std::move(finish)),
      m_cancel(std::move(cancel)) {}

bool RotationGizmo::process_input(
    const InputFrameSnapshot& input,
    const CameraState& camera
) {
    const auto active = m_active ? m_active() : std::nullopt;
    if (input.escape_pressed && active) {
        m_dragging = false;
        if (m_cancel) m_cancel();
        return true;
    }
    if (input.left_mouse_pressed && !active && m_frame_provider) {
        const auto frame = m_frame_provider();
        if (!frame || input.screen_width <= 0 || input.screen_height <= 0) return false;
        const double scale = gizmo_world_scale(frame->pivot, camera, input.screen_height);
        const auto camera_axis = cad::normalized(camera.target - camera.position);
        if (!camera_axis || scale <= 0.0) return false;
        std::optional<std::pair<RotationConstraint, cad::Vector3>> hit;
        double closest = 9.0;
        for (const auto& [constraint, axis, radius] : {
                 std::tuple{RotationConstraint::x, frame->x, scale},
                 std::tuple{RotationConstraint::y, frame->y, scale},
                 std::tuple{RotationConstraint::z, frame->z, scale},
                 std::tuple{RotationConstraint::screen, *camera_axis, scale * 1.18}
             }) {
            const double distance = ring_screen_distance(
                input.mouse_position, frame->pivot, axis, radius, camera,
                input.screen_width, input.screen_height
            );
            if (distance < closest) {
                closest = distance;
                hit = std::pair{constraint, axis};
            }
        }
        if (!hit || !m_begin(hit->first, hit->second)) return false;
        m_center = project_to_viewport(
            frame->pivot, camera, input.screen_width, input.screen_height
        );
        m_start_angle = std::atan2(
            input.mouse_position.y - m_center.y,
            input.mouse_position.x - m_center.x
        );
        m_dragging = true;
        return true;
    }
    if (!m_dragging || !active) return false;
    if (input.left_mouse_released) {
        m_dragging = false;
        if (m_finish) m_finish();
        return true;
    }
    double angle = std::atan2(
        input.mouse_position.y - m_center.y,
        input.mouse_position.x - m_center.x
    ) - m_start_angle;
    if (input.modifiers.ctrl) {
        constexpr double increment = 15.0 * std::numbers::pi / 180.0;
        angle = std::round(angle / increment) * increment;
    }
    if (m_preview) (void)m_preview(angle);
    return true;
}

void RotationGizmo::append_draw_data(
    GizmoDrawList& draw_list,
    const CameraState& camera,
    int viewport_height
) const {
    const auto frame = m_frame_provider ? m_frame_provider() : std::nullopt;
    const auto screen_axis = cad::normalized(camera.target - camera.position);
    if (!frame || !screen_axis) return;
    const double scale = gizmo_world_scale(frame->pivot, camera, viewport_height);
    if (!std::isfinite(scale) || scale <= 0.0) return;
    const auto active = m_active ? m_active() : std::nullopt;
    draw_list.emplace_back(GizmoRing{
        frame->pivot, frame->x, scale, active_color(active == RotationConstraint::x, red)
    });
    draw_list.emplace_back(GizmoRing{
        frame->pivot, frame->y, scale, active_color(active == RotationConstraint::y, green)
    });
    draw_list.emplace_back(GizmoRing{
        frame->pivot, frame->z, scale, active_color(active == RotationConstraint::z, blue)
    });
    draw_list.emplace_back(GizmoRing{
        frame->pivot, *screen_axis, scale * 1.18,
        active_color(active == RotationConstraint::screen, light_gray)
    });
}

ScaleGizmo::ScaleGizmo(
    ActiveHandler active,
    FrameHandler frame,
    BeginHandler begin,
    PreviewHandler preview,
    FinishHandler finish,
    FinishHandler cancel
)
    : m_active(std::move(active)),
      m_frame_provider(std::move(frame)),
      m_begin(std::move(begin)),
      m_preview(std::move(preview)),
      m_finish(std::move(finish)),
      m_cancel(std::move(cancel)) {}

bool ScaleGizmo::process_input(
    const InputFrameSnapshot& input,
    const CameraState& camera
) {
    const auto active = m_active ? m_active() : std::nullopt;
    if (input.escape_pressed && active) {
        m_dragging = false;
        if (m_cancel) m_cancel();
        return true;
    }
    if (input.left_mouse_pressed && !active && m_frame_provider) {
        const auto frame = m_frame_provider();
        if (!frame || input.screen_width <= 0 || input.screen_height <= 0) return false;
        const double scale = gizmo_world_scale(frame->pivot, camera, input.screen_height);
        const Vec2 center = project_to_viewport(
            frame->pivot, camera, input.screen_width, input.screen_height
        );
        std::optional<ScaleConstraint> hit;
        if (std::hypot(input.mouse_position.x - center.x,
                       input.mouse_position.y - center.y) <= 9.0) {
            hit = ScaleConstraint::uniform;
        }
        if (!hit) {
            for (const auto& [constraint, axis] : {
                     std::pair{ScaleConstraint::x, frame->x},
                     std::pair{ScaleConstraint::y, frame->y},
                     std::pair{ScaleConstraint::z, frame->z}
                 }) {
                const Vec2 end = project_to_viewport(
                    frame->pivot + axis * scale, camera,
                    input.screen_width, input.screen_height
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
        if (!hit || (*hit != ScaleConstraint::uniform && m_axis_screen_length <= 1e-6) ||
            !m_begin(*hit)) {
            return false;
        }
        m_constraint = *hit;
        m_start_mouse = input.mouse_position;
        m_dragging = true;
        return true;
    }
    if (!m_dragging || !active) return false;
    if (input.left_mouse_released) {
        m_dragging = false;
        if (m_finish) m_finish();
        return true;
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
    if (m_preview) (void)m_preview(factor);
    return true;
}

void ScaleGizmo::append_draw_data(
    GizmoDrawList& draw_list,
    const CameraState& camera,
    int viewport_height
) const {
    const auto frame = m_frame_provider ? m_frame_provider() : std::nullopt;
    if (!frame) return;
    const double scale = gizmo_world_scale(frame->pivot, camera, viewport_height);
    if (!std::isfinite(scale) || scale <= 0.0) return;
    const auto active = m_active ? m_active() : std::nullopt;
    draw_list.emplace_back(GizmoCenterHandle{
        frame->pivot, scale * 0.14, GizmoCenterShape::box,
        active_color(active == ScaleConstraint::uniform, light_gray)
    });
    for (const auto& [constraint, axis, color] : {
             std::tuple{ScaleConstraint::x, frame->x, red},
             std::tuple{ScaleConstraint::y, frame->y, green},
             std::tuple{ScaleConstraint::z, frame->z, blue}
         }) {
        const cad::Point3 end = frame->pivot + axis * scale;
        const GizmoColor handle = active_color(active == constraint, color);
        draw_list.emplace_back(GizmoLine{frame->pivot, end, scale * 0.025, handle});
        draw_list.emplace_back(GizmoBox{end, scale * 0.13, handle});
    }
}
