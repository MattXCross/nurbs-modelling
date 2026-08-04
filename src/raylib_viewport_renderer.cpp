#include "raylib_viewport_renderer.h"

#include "core.h"
#include "nurbs_surface.h"
#include "raylib.h"
#include "rlgl.h"
#include "scene.h"
#include "surface_tessellation.h"
#include "viewport_math.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace {

Vector3 to_raylib(const Point3D& point) {
    return {
        static_cast<float>(point.x),
        static_cast<float>(point.y),
        static_cast<float>(point.z)
    };
}

Vector3 to_raylib(cad::Vector3 vector) {
    return {
        static_cast<float>(vector.x),
        static_cast<float>(vector.y),
        static_cast<float>(vector.z)
    };
}

Camera3D to_raylib(const CameraState& camera) {
    return {
        .position = to_raylib(camera.position),
        .target = to_raylib(camera.target),
        .up = to_raylib(camera.up),
        .fovy = camera.vertical_fov_degrees,
        .projection = camera.projection == ProjectionMode::perspective
            ? CAMERA_PERSPECTIVE : CAMERA_ORTHOGRAPHIC
    };
}

cad::Aabb3 visible_bounds(const Scene& scene) {
    cad::Aabb3 bounds;
    for (const SceneNode& node : scene.nodes()) {
        if (!node.visible || node.surface == nullptr) continue;
        const auto entity_bounds = node.surface->control_bounds();
        if (!entity_bounds) continue;
        if (const auto minimum = entity_bounds->minimum()) (void)bounds.expand(*minimum);
        if (const auto maximum = entity_bounds->maximum()) (void)bounds.expand(*maximum);
    }
    return bounds;
}

void draw_axis_letter(char letter, Vector2 center, Color color) {
    constexpr float size = 4.0f;
    const auto line = [color](Vector2 start, Vector2 end) {
        DrawLineEx(start, end, 2.0f, color);
    };
    if (letter == 'X') {
        line({center.x - size, center.y - size}, {center.x + size, center.y + size});
        line({center.x - size, center.y + size}, {center.x + size, center.y - size});
    } else if (letter == 'Y') {
        line({center.x - size, center.y - size}, center);
        line({center.x + size, center.y - size}, center);
        line(center, {center.x, center.y + size});
    } else {
        line({center.x - size, center.y - size}, {center.x + size, center.y - size});
        line({center.x + size, center.y - size}, {center.x - size, center.y + size});
        line({center.x - size, center.y + size}, {center.x + size, center.y + size});
    }
}

void draw_orientation_triad(const CameraState& camera, int framebuffer_height) {
    const auto forward = cad::normalized(camera.target - camera.position);
    const auto right = forward ? cad::normalized(cad::cross(*forward, camera.up)) : std::nullopt;
    const auto up = right && forward ? cad::normalized(cad::cross(*right, *forward)) : std::nullopt;
    if (!right || !up || !forward) return;

    const Vector2 origin{48.0f, static_cast<float>(framebuffer_height) - 48.0f};
    struct Axis { cad::Vector3 direction; Color color; char label; };
    for (const Axis axis : {
             Axis{{0.0, 0.0, 1.0}, BLUE, 'Z'},
             Axis{{0.0, 1.0, 0.0}, GREEN, 'Y'},
             Axis{{1.0, 0.0, 0.0}, RED, 'X'}
         }) {
        const double screen_x = cad::dot(axis.direction, *right);
        const double screen_y = cad::dot(axis.direction, *up);
        Vector2 end{
            origin.x + static_cast<float>(screen_x * 30.0),
            origin.y - static_cast<float>(screen_y * 30.0)
        };
        if (std::hypot(end.x - origin.x, end.y - origin.y) < 3.0f) {
            const float sign = cad::dot(axis.direction, *forward) >= 0.0 ? 1.0f : -1.0f;
            end = {origin.x + sign * 6.0f, origin.y + sign * 6.0f};
        }
        DrawLineEx(origin, end, 3.0f, axis.color);
        DrawCircleV(end, 4.0f, axis.color);
        const cad::Vector2 label_direction = cad::normalized(cad::Vector2{
            end.x - origin.x, end.y - origin.y
        }).value_or(cad::Vector2{1.0, 0.0});
        draw_axis_letter(
            axis.label,
            {end.x + static_cast<float>(label_direction.x * 9.0),
             end.y + static_cast<float>(label_direction.y * 9.0)},
            axis.color
        );
    }
    DrawCircleV(origin, 4.0f, LIGHTGRAY);
}

void draw_control_net(
    EntityId entity,
    const NurbsSurface& surface,
    std::span<const ControlPointSelection> selected_points
) {
    const auto net = surface.control_net_2d();
    for (std::size_t u = 0; u < net.extent(0); ++u) {
        for (std::size_t v = 0; v < net.extent(1); ++v) {
            const bool is_selected = std::ranges::any_of(
                selected_points,
                [entity, u, v](ControlPointSelection selection) {
                    return selection.entity == entity && selection.u == u && selection.v == v;
                }
            );
            const Vector3 position = to_raylib(net[u, v].position);
            DrawSphere(position, is_selected ? 0.24f : 0.15f, is_selected ? GOLD : RED);

            if (u + 1 < net.extent(0)) {
                DrawLine3D(position, to_raylib(net[u + 1, v].position), GRAY);
            }
            if (v + 1 < net.extent(1)) {
                DrawLine3D(position, to_raylib(net[u, v + 1].position), GRAY);
            }
        }
    }
}

std::pair<cad::Vector3, cad::Vector3> ring_basis(cad::Vector3 normal) {
    const cad::Vector3 reference = std::abs(normal.y) < 0.9
        ? cad::Vector3{0.0, 1.0, 0.0}
        : cad::Vector3{1.0, 0.0, 0.0};
    const cad::Vector3 first = cad::normalized(cad::cross(normal, reference))
        .value_or(cad::Vector3{1.0, 0.0, 0.0});
    return {first, cad::cross(normal, first)};
}

void draw_ring(cad::Point3 pivot, cad::Vector3 normal, double radius, Color color) {
    const auto [first, second] = ring_basis(normal);
    constexpr std::size_t segments = 64;
    cad::Point3 previous = pivot + first * radius;
    for (std::size_t index = 1; index <= segments; ++index) {
        const double angle = 2.0 * std::numbers::pi * static_cast<double>(index) /
            static_cast<double>(segments);
        const cad::Point3 current = pivot +
            (first * std::cos(angle) + second * std::sin(angle)) * radius;
        DrawLine3D(to_raylib(previous), to_raylib(current), color);
        previous = current;
    }
}

Color to_raylib(GizmoColor color) {
    return {color.red, color.green, color.blue, color.alpha};
}

void draw_gizmo(const GizmoPrimitive& primitive) {
    std::visit([](const auto& item) {
        using Item = std::decay_t<decltype(item)>;
        const Color color = to_raylib(item.color);
        if constexpr (std::is_same_v<Item, GizmoLine>) {
            if (item.radius > 0.0) {
                DrawCylinderEx(
                    to_raylib(item.start), to_raylib(item.end),
                    static_cast<float>(item.radius), static_cast<float>(item.radius), 8, color
                );
            } else {
                DrawLine3D(to_raylib(item.start), to_raylib(item.end), color);
            }
        } else if constexpr (std::is_same_v<Item, GizmoArrow>) {
            DrawCylinderEx(
                to_raylib(item.start), to_raylib(item.shaft_end),
                static_cast<float>(item.shaft_radius),
                static_cast<float>(item.shaft_radius), 8, color
            );
            DrawCylinderEx(
                to_raylib(item.head_start), to_raylib(item.tip),
                static_cast<float>(item.head_radius), 0.0f, 12, color
            );
        } else if constexpr (std::is_same_v<Item, GizmoBox>) {
            DrawCube(
                to_raylib(item.center), static_cast<float>(item.size),
                static_cast<float>(item.size), static_cast<float>(item.size), color
            );
        } else if constexpr (std::is_same_v<Item, GizmoPlaneHandle>) {
            DrawTriangle3D(
                to_raylib(item.first), to_raylib(item.second), to_raylib(item.third), color
            );
            DrawTriangle3D(
                to_raylib(item.third), to_raylib(item.second), to_raylib(item.first), color
            );
        } else if constexpr (std::is_same_v<Item, GizmoRing>) {
            draw_ring(item.center, item.normal, item.radius, color);
        } else if constexpr (std::is_same_v<Item, GizmoCenterHandle>) {
            if (item.shape == GizmoCenterShape::sphere) {
                DrawSphere(to_raylib(item.center), static_cast<float>(item.size), color);
            } else {
                DrawCube(
                    to_raylib(item.center), static_cast<float>(item.size),
                    static_cast<float>(item.size), static_cast<float>(item.size), color
                );
            }
        }
    }, primitive);
}

} // namespace

class RaylibViewportRenderer::Impl {
public:
    void render(
        const Scene& scene,
        const CameraState& camera,
        std::span<const ControlPointSelection> selected_points,
        std::optional<EntityId> selected_entity,
        std::optional<EntityId> hovered_entity,
        std::span<const GizmoPrimitive> gizmos,
        int framebuffer_width,
        int framebuffer_height
    ) {
        framebuffer_width = std::max(framebuffer_width, 1);
        framebuffer_height = std::max(framebuffer_height, 1);
        const double aspect = static_cast<double>(framebuffer_width) /
            static_cast<double>(framebuffer_height);
        constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
        const ClipPlanes clipping = derive_clip_planes(camera, visible_bounds(scene));

        rlClearColor(16, 20, 26, 255);
        rlClearScreenBuffers();
        rlMatrixMode(RL_PROJECTION);
        rlLoadIdentity();
        if (camera.projection == ProjectionMode::perspective) {
            const double top = clipping.near_plane * std::tan(
                static_cast<double>(camera.vertical_fov_degrees) * degrees_to_radians * 0.5
            );
            const double right = top * aspect;
            rlFrustum(
                -right, right, -top, top, clipping.near_plane, clipping.far_plane
            );
        } else {
            const double top = camera.orthographic_vertical_size * 0.5;
            const double right = top * aspect;
            rlOrtho(-right, right, -top, top, clipping.near_plane, clipping.far_plane);
        }
        rlMatrixMode(RL_MODELVIEW);
        rlSetMatrixModelview(GetCameraMatrix(to_raylib(camera)));
        rlEnableDepthTest();

        DrawGrid(20, 1.0f);
        for (const auto& node : scene.nodes()) {
            if (!node.visible || node.surface == nullptr) {
                continue;
            }

            draw_control_net(node.id, *node.surface, selected_points);
            const auto mesh = m_mesh_cache.get(
                *node.surface, node.geometry_revision, m_tessellation_settings
            );
            const Color surface_color = selected_entity == node.id
                ? GOLD
                : (hovered_entity == node.id ? SKYBLUE : BLUE);
            if (mesh) {
                draw_mesh_edges(**mesh, surface_color);
            }
        }
        for (const GizmoPrimitive& gizmo : gizmos) {
            draw_gizmo(gizmo);
        }
        rlDrawRenderBatchActive();
        rlDisableDepthTest();
        rlMatrixMode(RL_PROJECTION);
        rlLoadIdentity();
        rlOrtho(0.0, framebuffer_width, framebuffer_height, 0.0, -1.0, 1.0);
        rlMatrixMode(RL_MODELVIEW);
        rlLoadIdentity();
        draw_orientation_triad(camera, framebuffer_height);
        rlDrawRenderBatchActive();

    }

private:
    static void draw_mesh_edges(const cad::SurfaceMesh& mesh, Color color) {
        std::unordered_set<std::uint64_t> drawn;
        for (std::size_t index = 0; index + 2 < mesh.triangle_indices.size(); index += 3) {
            const std::array triangle{
                mesh.triangle_indices[index], mesh.triangle_indices[index + 1],
                mesh.triangle_indices[index + 2]
            };
            for (std::size_t edge = 0; edge < 3; ++edge) {
                const std::uint32_t a = triangle[edge];
                const std::uint32_t b = triangle[(edge + 1) % 3];
                if (a >= mesh.positions.size() || b >= mesh.positions.size()) continue;
                const std::uint32_t low = std::min(a, b);
                const std::uint32_t high = std::max(a, b);
                const std::uint64_t key = (static_cast<std::uint64_t>(low) << 32) | high;
                if (drawn.insert(key).second) {
                    DrawLine3D(to_raylib(mesh.positions[a]), to_raylib(mesh.positions[b]), color);
                }
            }
        }
    }

    cad::SurfaceTessellationSettings m_tessellation_settings{};
    cad::SurfaceTessellationCache m_mesh_cache;
};

RaylibViewportRenderer::RaylibViewportRenderer()
    : m_impl(std::make_unique<Impl>()) {}

RaylibViewportRenderer::~RaylibViewportRenderer() = default;

void RaylibViewportRenderer::render(
    const Scene& scene,
    const CameraState& camera,
    std::span<const ControlPointSelection> selected_points,
    std::optional<EntityId> selected_entity,
    std::optional<EntityId> hovered_entity,
    std::span<const GizmoPrimitive> gizmos,
    int framebuffer_width,
    int framebuffer_height
) {
    m_impl->render(
        scene,
        camera,
        selected_points,
        selected_entity,
        hovered_entity,
        gizmos,
        framebuffer_width,
        framebuffer_height
    );
}
