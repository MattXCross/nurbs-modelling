#include "raylib_viewport_renderer.h"

#include "core.h"
#include "nurbs_surface.h"
#include "raylib.h"
#include "rlgl.h"
#include "scene.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <utility>
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
        .projection = CAMERA_PERSPECTIVE
    };
}

void draw_control_net(const NurbsSurface& surface, const ControlPoint* selected_point) {
    const auto net = surface.control_net_2d();
    for (std::size_t u = 0; u < net.extent(0); ++u) {
        for (std::size_t v = 0; v < net.extent(1); ++v) {
            const bool is_selected = &net[u, v] == selected_point;
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

} // namespace

class RaylibViewportRenderer::Impl {
public:
    void render(
        const Scene& scene,
        const CameraState& camera,
        const ControlPoint* selected_point,
        std::optional<EntityId> selected_entity,
        std::optional<EntityId> hovered_entity,
        int framebuffer_width,
        int framebuffer_height
    ) {
        framebuffer_width = std::max(framebuffer_width, 1);
        framebuffer_height = std::max(framebuffer_height, 1);
        const double aspect = static_cast<double>(framebuffer_width) /
            static_cast<double>(framebuffer_height);
        constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
        const double near_plane = rlGetCullDistanceNear();
        const double far_plane = rlGetCullDistanceFar();
        const double top = near_plane * std::tan(
            static_cast<double>(camera.vertical_fov_degrees) * degrees_to_radians * 0.5
        );
        const double right = top * aspect;

        rlClearColor(16, 20, 26, 255);
        rlClearScreenBuffers();
        rlMatrixMode(RL_PROJECTION);
        rlLoadIdentity();
        rlFrustum(-right, right, -top, top, near_plane, far_plane);
        rlMatrixMode(RL_MODELVIEW);
        rlSetMatrixModelview(GetCameraMatrix(to_raylib(camera)));
        rlEnableDepthTest();

        DrawGrid(20, 1.0f);
        for (const auto& node : scene.nodes()) {
            if (!node.visible || node.surface == nullptr) {
                continue;
            }

            draw_control_net(*node.surface, selected_point);
            CachedSurface& cached = m_surface_cache[node.id.value];
            if (cached.surface != node.surface.get() || cached.revision != node.geometry_revision) {
                cached.line_vertices = tessellate_wireframe(*node.surface, 100, 100);
                cached.surface = node.surface.get();
                cached.revision = node.geometry_revision;
            }
            const Color surface_color = selected_entity == node.id
                ? GOLD
                : (hovered_entity == node.id ? SKYBLUE : BLUE);
            for (std::size_t index = 0; index + 1 < cached.line_vertices.size(); index += 2) {
                DrawLine3D(
                    cached.line_vertices[index],
                    cached.line_vertices[index + 1],
                    surface_color
                );
            }
        }
        rlDrawRenderBatchActive();
        rlDisableDepthTest();

        std::erase_if(m_surface_cache, [&scene](const auto& entry) {
            return scene.find_entity(EntityId{entry.first}) == nullptr;
        });
    }

private:
    struct CachedSurface {
        const NurbsSurface* surface{nullptr};
        std::uint64_t revision{0};
        std::vector<Vector3> line_vertices;
    };

    static std::vector<Vector3> tessellate_wireframe(
        const NurbsSurface& surface,
        std::size_t u_segments,
        std::size_t v_segments
    ) {
        const auto u_domain = surface.u_domain();
        const auto v_domain = surface.v_domain();
        if (!u_domain.has_value() || !v_domain.has_value() ||
            u_segments == 0 || v_segments == 0) {
            return {};
        }
        const auto [u_min, u_max] = *u_domain;
        const auto [v_min, v_max] = *v_domain;
        const std::size_t row_size = v_segments + 1;
        std::vector<Vector3> samples((u_segments + 1) * (v_segments + 1));
        std::vector<bool> valid(samples.size(), false);

        for (std::size_t u = 0; u <= u_segments; ++u) {
            const double u_parameter = u_min + (u_max - u_min) *
                static_cast<double>(u) / static_cast<double>(u_segments);
            for (std::size_t v = 0; v <= v_segments; ++v) {
                const double v_parameter = v_min + (v_max - v_min) *
                    static_cast<double>(v) / static_cast<double>(v_segments);
                const auto point = surface.evaluate(u_parameter, v_parameter);
                const std::size_t index = u * row_size + v;
                if (point.has_value()) {
                    samples[index] = to_raylib(*point);
                    valid[index] = true;
                }
            }
        }

        std::vector<Vector3> lines;
        lines.reserve(2 * (
            u_segments * (v_segments + 1) +
            v_segments * (u_segments + 1)
        ));
        for (std::size_t u = 0; u <= u_segments; ++u) {
            for (std::size_t v = 0; v <= v_segments; ++v) {
                const std::size_t index = u * row_size + v;
                if (u < u_segments) {
                    const std::size_t next_u = (u + 1) * row_size + v;
                    if (valid[index] && valid[next_u]) {
                        lines.push_back(samples[index]);
                        lines.push_back(samples[next_u]);
                    }
                }
                if (v < v_segments) {
                    const std::size_t next_v = index + 1;
                    if (valid[index] && valid[next_v]) {
                        lines.push_back(samples[index]);
                        lines.push_back(samples[next_v]);
                    }
                }
            }
        }
        return lines;
    }

    std::unordered_map<std::uint64_t, CachedSurface> m_surface_cache;
};

RaylibViewportRenderer::RaylibViewportRenderer()
    : m_impl(std::make_unique<Impl>()) {}

RaylibViewportRenderer::~RaylibViewportRenderer() = default;

void RaylibViewportRenderer::render(
    const Scene& scene,
    const CameraState& camera,
    const ControlPoint* selected_point,
    std::optional<EntityId> selected_entity,
    std::optional<EntityId> hovered_entity,
    int framebuffer_width,
    int framebuffer_height
) {
    m_impl->render(
        scene,
        camera,
        selected_point,
        selected_entity,
        hovered_entity,
        framebuffer_width,
        framebuffer_height
    );
}
