#include "raylib_viewport_renderer.h"

#include "core.h"
#include "nurbs_surface.h"
#include "raylib.h"
#include "rlgl.h"
#include "scene.h"
#include "surface_tessellation.h"
#include "viewport_math.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace {

constexpr const char* surface_vertex_shader = R"glsl(#version 330 core
layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec2 vertexUv;

uniform mat4 mvp;
uniform mat4 model;
uniform float depthBias;

out vec3 fragmentPosition;
out vec3 fragmentNormal;
out vec2 fragmentUv;

void main() {
    vec4 worldPosition = model * vec4(vertexPosition, 1.0);
    fragmentPosition = worldPosition.xyz;
    fragmentNormal = mat3(transpose(inverse(model))) * vertexNormal;
    fragmentUv = vertexUv;
    gl_Position = mvp * worldPosition;
    gl_Position.z -= depthBias * gl_Position.w;
}
)glsl";

constexpr const char* surface_fragment_shader = R"glsl(#version 330 core
in vec3 fragmentPosition;
in vec3 fragmentNormal;
in vec2 fragmentUv;

uniform vec3 objectColor;
uniform vec3 cameraPosition;
uniform vec3 lightDirection;
uniform int unlit;

out vec4 finalColor;

void main() {
    if (unlit != 0) {
        finalColor = vec4(objectColor, 1.0);
        return;
    }

    vec3 normal = normalize(fragmentNormal);
    normal = gl_FrontFacing ? normal : -normal;
    vec3 viewDirection = normalize(cameraPosition - fragmentPosition);
    float diffuse = max(dot(normal, normalize(lightDirection)), 0.0);
    float rim = pow(1.0 - max(dot(normal, viewDirection), 0.0), 2.0);
    vec3 sideColor = gl_FrontFacing ? objectColor : objectColor * vec3(0.62, 0.74, 0.92);
    float parameterCue = mix(0.94, 1.04, fragmentUv.x);
    finalColor = vec4(sideColor * (0.28 + 0.68 * diffuse) * parameterCue +
                      sideColor * 0.12 * rim, 1.0);
}
)glsl";

struct GpuVertex {
    float position[3];
    float normal[3];
    float uv[2];
};

struct GpuSurface {
    std::uint64_t geometry_revision{0};
    cad::SurfaceTessellationSettings settings;
    unsigned int vao{0};
    unsigned int vbo{0};
    int vertex_count{0};
};

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

const cad::AffineTransform3* preview_transform(
    EntityId entity,
    const std::optional<EntityTransformPreview>& preview
) {
    return preview && preview->entity == entity ? &preview->transform : nullptr;
}

cad::Point3 transformed_point(
    cad::Point3 point,
    const cad::AffineTransform3* transform
) {
    return transform == nullptr ? point : transform->transform_point(point);
}

cad::Aabb3 visible_bounds(
    const Scene& scene,
    const std::optional<EntityTransformPreview>& preview
) {
    cad::Aabb3 bounds;
    for (const SceneNode& node : scene.nodes()) {
        if (!node.visible || node.surface == nullptr) continue;
        const cad::AffineTransform3* transform = preview_transform(node.id, preview);
        const auto net = node.surface->control_net_2d();
        for (std::size_t u = 0; u < net.extent(0); ++u) {
            for (std::size_t v = 0; v < net.extent(1); ++v) {
                (void)bounds.expand(transformed_point(net[u, v].position, transform));
            }
        }
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
    const NurbsSurface& surface,
    const cad::AffineTransform3* transform
) {
    const auto net = surface.control_net_2d();
    for (std::size_t u = 0; u < net.extent(0); ++u) {
        for (std::size_t v = 0; v < net.extent(1); ++v) {
            const Vector3 position = to_raylib(transformed_point(net[u, v].position, transform));
            if (u + 1 < net.extent(0)) {
                DrawLine3D(position, to_raylib(transformed_point(
                    net[u + 1, v].position, transform
                )), LIGHTGRAY);
            }
            if (v + 1 < net.extent(1)) {
                DrawLine3D(position, to_raylib(transformed_point(
                    net[u, v + 1].position, transform
                )), LIGHTGRAY);
            }
        }
    }
}

void draw_control_points(
    EntityId entity,
    const NurbsSurface& surface,
    std::span<const ControlPointSelection> selected_points,
    const cad::AffineTransform3* transform
) {
    const auto net = surface.control_net_2d();
    for (std::size_t u = 0; u < net.extent(0); ++u) {
        for (std::size_t v = 0; v < net.extent(1); ++v) {
            const bool selected = std::ranges::any_of(
                selected_points,
                [entity, u, v](ControlPointSelection selection) {
                    return selection.entity == entity && selection.u == u && selection.v == v;
                }
            );
            DrawSphere(
                to_raylib(transformed_point(net[u, v].position, transform)),
                selected ? 0.24f : 0.15f,
                selected ? GOLD : RED
            );
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

bool finite_float(double value) {
    return std::isfinite(value) &&
        std::abs(value) <= static_cast<double>(std::numeric_limits<float>::max());
}

std::optional<GpuVertex> make_gpu_vertex(const cad::SurfaceMesh& mesh, std::uint32_t index) {
    if (index >= mesh.positions.size() || index >= mesh.normals.size() ||
        index >= mesh.uvs.size()) {
        return std::nullopt;
    }
    const cad::Point3& position = mesh.positions[index];
    const cad::Vector3& normal = mesh.normals[index];
    const cad::Point2& uv = mesh.uvs[index];
    if (!finite_float(position.x) || !finite_float(position.y) ||
        !finite_float(position.z) || !finite_float(normal.x) ||
        !finite_float(normal.y) || !finite_float(normal.z) ||
        !finite_float(uv.x) || !finite_float(uv.y)) {
        return std::nullopt;
    }
    return GpuVertex{
        {static_cast<float>(position.x), static_cast<float>(position.y),
         static_cast<float>(position.z)},
        {static_cast<float>(normal.x), static_cast<float>(normal.y),
         static_cast<float>(normal.z)},
        {static_cast<float>(uv.x), static_cast<float>(uv.y)}
    };
}

void unload_gpu_surface(GpuSurface& surface) {
    if (surface.vbo != 0) rlUnloadVertexBuffer(surface.vbo);
    if (surface.vao != 0) rlUnloadVertexArray(surface.vao);
    surface.vbo = 0;
    surface.vao = 0;
    surface.vertex_count = 0;
}

GpuSurface upload_gpu_surface(
    const cad::SurfaceMesh& mesh,
    std::uint64_t geometry_revision,
    const cad::SurfaceTessellationSettings& settings
) {
    GpuSurface result{.geometry_revision = geometry_revision, .settings = settings};
    if (mesh.triangle_indices.empty() || mesh.triangle_indices.size() % 3 != 0 ||
        mesh.triangle_indices.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
        mesh.triangle_indices.size() >
            static_cast<std::size_t>(std::numeric_limits<int>::max()) / sizeof(GpuVertex)) {
        return result;
    }

    std::vector<GpuVertex> vertices;
    vertices.reserve(mesh.triangle_indices.size());
    for (const std::uint32_t index : mesh.triangle_indices) {
        const auto vertex = make_gpu_vertex(mesh, index);
        if (!vertex) return result;
        vertices.push_back(*vertex);
    }

    result.vao = rlLoadVertexArray();
    if (result.vao == 0 || !rlEnableVertexArray(result.vao)) {
        unload_gpu_surface(result);
        return result;
    }
    const int byte_count = static_cast<int>(vertices.size() * sizeof(GpuVertex));
    result.vbo = rlLoadVertexBuffer(vertices.data(), byte_count, false);
    if (result.vbo == 0) {
        rlDisableVertexArray();
        unload_gpu_surface(result);
        return result;
    }
    rlSetVertexAttribute(0, 3, RL_FLOAT, false, sizeof(GpuVertex), offsetof(GpuVertex, position));
    rlEnableVertexAttribute(0);
    rlSetVertexAttribute(1, 3, RL_FLOAT, false, sizeof(GpuVertex), offsetof(GpuVertex, normal));
    rlEnableVertexAttribute(1);
    rlSetVertexAttribute(2, 2, RL_FLOAT, false, sizeof(GpuVertex), offsetof(GpuVertex, uv));
    rlEnableVertexAttribute(2);
    rlDisableVertexArray();
    rlDisableVertexBuffer();
    result.vertex_count = static_cast<int>(vertices.size());
    return result;
}

std::array<float, 3> color_components(DisplayColor color) {
    return {color.red, color.green, color.blue};
}

Matrix multiply_matrices(Matrix left, Matrix right) {
    Matrix result{};
    result.m0 = left.m0*right.m0 + left.m1*right.m4 + left.m2*right.m8 + left.m3*right.m12;
    result.m1 = left.m0*right.m1 + left.m1*right.m5 + left.m2*right.m9 + left.m3*right.m13;
    result.m2 = left.m0*right.m2 + left.m1*right.m6 + left.m2*right.m10 + left.m3*right.m14;
    result.m3 = left.m0*right.m3 + left.m1*right.m7 + left.m2*right.m11 + left.m3*right.m15;
    result.m4 = left.m4*right.m0 + left.m5*right.m4 + left.m6*right.m8 + left.m7*right.m12;
    result.m5 = left.m4*right.m1 + left.m5*right.m5 + left.m6*right.m9 + left.m7*right.m13;
    result.m6 = left.m4*right.m2 + left.m5*right.m6 + left.m6*right.m10 + left.m7*right.m14;
    result.m7 = left.m4*right.m3 + left.m5*right.m7 + left.m6*right.m11 + left.m7*right.m15;
    result.m8 = left.m8*right.m0 + left.m9*right.m4 + left.m10*right.m8 + left.m11*right.m12;
    result.m9 = left.m8*right.m1 + left.m9*right.m5 + left.m10*right.m9 + left.m11*right.m13;
    result.m10 = left.m8*right.m2 + left.m9*right.m6 + left.m10*right.m10 + left.m11*right.m14;
    result.m11 = left.m8*right.m3 + left.m9*right.m7 + left.m10*right.m11 + left.m11*right.m15;
    result.m12 = left.m12*right.m0 + left.m13*right.m4 + left.m14*right.m8 + left.m15*right.m12;
    result.m13 = left.m12*right.m1 + left.m13*right.m5 + left.m14*right.m9 + left.m15*right.m13;
    result.m14 = left.m12*right.m2 + left.m13*right.m6 + left.m14*right.m10 + left.m15*right.m14;
    result.m15 = left.m12*right.m3 + left.m13*right.m7 + left.m14*right.m11 + left.m15*right.m15;
    return result;
}

Matrix to_raylib(const cad::AffineTransform3& transform) {
    const cad::Point3 translation = transform.transform_point({});
    const cad::Vector3 x = transform.transform_vector({1.0, 0.0, 0.0});
    const cad::Vector3 y = transform.transform_vector({0.0, 1.0, 0.0});
    const cad::Vector3 z = transform.transform_vector({0.0, 0.0, 1.0});
    Matrix result{};
    result.m0 = static_cast<float>(x.x);
    result.m1 = static_cast<float>(x.y);
    result.m2 = static_cast<float>(x.z);
    result.m4 = static_cast<float>(y.x);
    result.m5 = static_cast<float>(y.y);
    result.m6 = static_cast<float>(y.z);
    result.m8 = static_cast<float>(z.x);
    result.m9 = static_cast<float>(z.y);
    result.m10 = static_cast<float>(z.z);
    result.m12 = static_cast<float>(translation.x);
    result.m13 = static_cast<float>(translation.y);
    result.m14 = static_cast<float>(translation.z);
    result.m15 = 1.0f;
    return result;
}

Matrix identity_matrix() {
    Matrix result{};
    result.m0 = 1.0f;
    result.m5 = 1.0f;
    result.m10 = 1.0f;
    result.m15 = 1.0f;
    return result;
}

DisplayColor surface_color(EntityId entity, bool selected, bool hovered) {
    if (selected) return {1.0f, 0.69f, 0.13f};
    if (hovered) return {0.25f, 0.78f, 1.0f};
    return object_display_color(entity);
}

DisplayColor edge_color(EntityId entity, bool selected, bool hovered) {
    if (selected) return {1.0f, 0.88f, 0.35f};
    if (hovered) return {0.62f, 0.92f, 1.0f};
    const DisplayColor base = object_display_color(entity);
    return {base.red * 0.22f, base.green * 0.22f, base.blue * 0.22f};
}

} // namespace

class RaylibViewportRenderer::Impl {
public:
    void cleanup_gl() {
        for (auto& [identity, surface] : m_gpu_surfaces) {
            (void)identity;
            unload_gpu_surface(surface);
        }
        m_gpu_surfaces.clear();
        if (m_shader != 0) rlUnloadShaderProgram(m_shader);
        m_shader = 0;
        m_mesh_cache.clear();
    }

    void render(
        const Scene& scene,
        const CameraState& camera,
        std::span<const ControlPointSelection> selected_points,
        std::optional<EntityId> selected_entity,
        std::optional<EntityId> hovered_entity,
        std::optional<EntityTransformPreview> entity_preview,
        std::span<const GizmoPrimitive> gizmos,
        const ViewportDisplaySettings& display_settings,
        bool interactive_geometry_edit,
        int framebuffer_width,
        int framebuffer_height
    ) {
        framebuffer_width = std::max(framebuffer_width, 1);
        framebuffer_height = std::max(framebuffer_height, 1);
        const double aspect = static_cast<double>(framebuffer_width) /
            static_cast<double>(framebuffer_height);
        constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
        const ClipPlanes clipping = derive_clip_planes(camera, visible_bounds(scene, entity_preview));

        rlClearColor(16, 20, 26, 255);
        rlClearScreenBuffers();
        rlMatrixMode(RL_PROJECTION);
        rlLoadIdentity();
        if (camera.projection == ProjectionMode::perspective) {
            const double top = clipping.near_plane * std::tan(
                static_cast<double>(camera.vertical_fov_degrees) * degrees_to_radians * 0.5
            );
            const double right = top * aspect;
            rlFrustum(-right, right, -top, top, clipping.near_plane, clipping.far_plane);
        } else {
            const double top = camera.orthographic_vertical_size * 0.5;
            const double right = top * aspect;
            rlOrtho(-right, right, -top, top, clipping.near_plane, clipping.far_plane);
        }
        rlMatrixMode(RL_MODELVIEW);
        rlSetMatrixModelview(GetCameraMatrix(to_raylib(camera)));
        rlEnableDepthTest();

        DrawGrid(20, 1.0f);
        rlDrawRenderBatchActive();

        ensure_shader();
        std::unordered_set<std::uint64_t> active_surfaces;
        for (const SceneNode& node : scene.nodes()) {
            if (node.surface != nullptr) active_surfaces.insert(node.surface->identity());
        }
        for (const SceneNode& node : scene.nodes()) {
            if (!node.visible || node.surface == nullptr) continue;
            const cad::SurfaceTessellationSettings& tessellation_settings =
                interactive_geometry_edit
                    ? m_interactive_tessellation_settings
                    : m_tessellation_settings;
            GpuSurface* gpu = gpu_surface(
                *node.surface,
                node.geometry_revision,
                tessellation_settings
            );
            if (gpu == nullptr || m_shader == 0) continue;

            const bool selected = selected_entity == node.id;
            const bool hovered = !selected && hovered_entity == node.id;
            const cad::AffineTransform3* transform = preview_transform(node.id, entity_preview);
            const Matrix model = transform == nullptr ? identity_matrix() : to_raylib(*transform);
            if (display_settings.surface_mode != SurfaceDisplayMode::wireframe) {
                draw_surface(
                    *gpu, surface_color(node.id, selected, hovered), camera, model, false, 0.0f
                );
            }
            if (display_settings.surface_mode != SurfaceDisplayMode::shaded) {
                const float depth_bias = display_settings.surface_mode ==
                    SurfaceDisplayMode::shaded_with_edges ? 0.0006f : 0.0f;
                draw_surface(
                    *gpu, edge_color(node.id, selected, hovered), camera, model, true, depth_bias
                );
            }
        }
        std::erase_if(m_gpu_surfaces, [&active_surfaces](auto& entry) {
            if (active_surfaces.contains(entry.first)) return false;
            unload_gpu_surface(entry.second);
            return true;
        });

        rlEnableBackfaceCulling();
        for (const SceneNode& node : scene.nodes()) {
            if (!node.visible || node.surface == nullptr) continue;
            const cad::AffineTransform3* transform = preview_transform(node.id, entity_preview);
            if (display_settings.show_control_net) draw_control_net(*node.surface, transform);
            if (display_settings.show_control_points) {
                draw_control_points(node.id, *node.surface, selected_points, transform);
            }
        }
        rlDrawRenderBatchActive();

        // Gizmos are interaction overlays and must remain reachable through geometry.
        rlDisableDepthTest();
        for (const GizmoPrimitive& gizmo : gizmos) draw_gizmo(gizmo);
        rlDrawRenderBatchActive();

        rlMatrixMode(RL_PROJECTION);
        rlLoadIdentity();
        rlOrtho(0.0, framebuffer_width, framebuffer_height, 0.0, -1.0, 1.0);
        rlMatrixMode(RL_MODELVIEW);
        rlLoadIdentity();
        draw_orientation_triad(camera, framebuffer_height);
        rlDrawRenderBatchActive();
    }

private:
    void ensure_shader() {
        if (m_shader != 0) return;
        m_shader = rlLoadShaderProgram(surface_vertex_shader, surface_fragment_shader);
        if (m_shader == 0) return;
        m_mvp_location = rlGetLocationUniform(m_shader, "mvp");
        m_model_location = rlGetLocationUniform(m_shader, "model");
        m_color_location = rlGetLocationUniform(m_shader, "objectColor");
        m_camera_location = rlGetLocationUniform(m_shader, "cameraPosition");
        m_light_location = rlGetLocationUniform(m_shader, "lightDirection");
        m_unlit_location = rlGetLocationUniform(m_shader, "unlit");
        m_depth_bias_location = rlGetLocationUniform(m_shader, "depthBias");
    }

    GpuSurface* gpu_surface(
        const NurbsSurface& surface,
        std::uint64_t geometry_revision,
        const cad::SurfaceTessellationSettings& settings
    ) {
        const std::uint64_t identity = surface.identity();
        auto found = m_gpu_surfaces.find(identity);
        if (found != m_gpu_surfaces.end() &&
            found->second.geometry_revision == geometry_revision &&
            found->second.settings == settings) {
            return found->second.vao == 0 ? nullptr : &found->second;
        }
        if (found != m_gpu_surfaces.end()) {
            unload_gpu_surface(found->second);
            m_gpu_surfaces.erase(found);
        }

        m_mesh_cache.clear(surface);
        const auto mesh = m_mesh_cache.get(surface, geometry_revision, settings);
        GpuSurface uploaded{.geometry_revision = geometry_revision,
                            .settings = settings};
        if (mesh) uploaded = upload_gpu_surface(**mesh, geometry_revision, settings);
        auto [stored, inserted] = m_gpu_surfaces.emplace(identity, std::move(uploaded));
        (void)inserted;
        return stored->second.vao == 0 ? nullptr : &stored->second;
    }

    void draw_surface(
        const GpuSurface& surface,
        DisplayColor color,
        const CameraState& camera,
        Matrix model,
        bool wireframe,
        float depth_bias
    ) const {
        if (surface.vao == 0 || surface.vertex_count <= 0) return;
        rlDisableBackfaceCulling();
        rlEnableShader(m_shader);
        const Matrix mvp = multiply_matrices(
            rlGetMatrixModelview(), rlGetMatrixProjection()
        );
        rlSetUniformMatrix(m_mvp_location, mvp);
        rlSetUniformMatrix(m_model_location, model);
        const auto color_values = color_components(color);
        rlSetUniform(m_color_location, color_values.data(), RL_SHADER_UNIFORM_VEC3, 1);
        const std::array<float, 3> camera_values{
            static_cast<float>(camera.position.x), static_cast<float>(camera.position.y),
            static_cast<float>(camera.position.z)
        };
        rlSetUniform(m_camera_location, camera_values.data(), RL_SHADER_UNIFORM_VEC3, 1);
        constexpr std::array<float, 3> light_values{0.42f, 0.82f, 0.38f};
        rlSetUniform(m_light_location, light_values.data(), RL_SHADER_UNIFORM_VEC3, 1);
        const int unlit = wireframe ? 1 : 0;
        rlSetUniform(m_unlit_location, &unlit, RL_SHADER_UNIFORM_INT, 1);
        rlSetUniform(m_depth_bias_location, &depth_bias, RL_SHADER_UNIFORM_FLOAT, 1);

        if (wireframe) {
            rlSetLineWidth(1.5f);
            rlEnableWireMode();
        }
        if (rlEnableVertexArray(surface.vao)) rlDrawVertexArray(0, surface.vertex_count);
        rlDisableVertexArray();
        if (wireframe) {
            rlDisableWireMode();
            rlSetLineWidth(1.0f);
        }
        rlDisableShader();
    }

    cad::SurfaceTessellationSettings m_tessellation_settings{
        .best_effort = true
    };
    cad::SurfaceTessellationSettings m_interactive_tessellation_settings{
        .chordal_tolerance = 0.12,
        .normal_angle_tolerance_radians = 0.35,
        .max_refinement_depth = 4,
        .max_vertices = 65'536,
        .best_effort = true
    };
    cad::SurfaceTessellationCache m_mesh_cache;
    std::unordered_map<std::uint64_t, GpuSurface> m_gpu_surfaces;
    unsigned int m_shader{0};
    int m_mvp_location{-1};
    int m_model_location{-1};
    int m_color_location{-1};
    int m_camera_location{-1};
    int m_light_location{-1};
    int m_unlit_location{-1};
    int m_depth_bias_location{-1};
};

RaylibViewportRenderer::RaylibViewportRenderer()
    : m_impl(std::make_unique<Impl>()) {}

RaylibViewportRenderer::~RaylibViewportRenderer() = default;

void RaylibViewportRenderer::cleanup_gl() {
    m_impl->cleanup_gl();
}

void RaylibViewportRenderer::render(
    const Scene& scene,
    const CameraState& camera,
    std::span<const ControlPointSelection> selected_points,
    std::optional<EntityId> selected_entity,
    std::optional<EntityId> hovered_entity,
    std::optional<EntityTransformPreview> entity_preview,
    std::span<const GizmoPrimitive> gizmos,
    const ViewportDisplaySettings& display_settings,
    bool interactive_geometry_edit,
    int framebuffer_width,
    int framebuffer_height
) {
    m_impl->render(
        scene,
        camera,
        selected_points,
        selected_entity,
        hovered_entity,
        entity_preview,
        gizmos,
        display_settings,
        interactive_geometry_edit,
        framebuffer_width,
        framebuffer_height
    );
}
