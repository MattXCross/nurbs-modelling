#include "raylib_viewport_renderer.h"

#include "core.h"
#include "nurbs_surface.h"
#include "raylib.h"
#include "scene.h"

#include <algorithm>
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

Vector3 to_raylib(Vec3 vector) {
    return {vector.x, vector.y, vector.z};
}

Rectangle to_raylib(Rect rectangle) {
    return {rectangle.x, rectangle.y, rectangle.width, rectangle.height};
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
    Impl(int width, int height) {
        resize(width, height);
    }

    ~Impl() {
        if (m_target.id != 0) {
            UnloadRenderTexture(m_target);
        }
    }

    void resize(int width, int height) {
        width = std::max(width, 1);
        height = std::max(height, 1);
        if (m_target.texture.width == width && m_target.texture.height == height) {
            return;
        }
        if (m_target.id != 0) {
            UnloadRenderTexture(m_target);
        }
        m_target = LoadRenderTexture(width, height);
    }

    void render(
        const Scene& scene,
        const CameraState& camera,
        const ControlPoint* selected_point
    ) {
        BeginTextureMode(m_target);
            ClearBackground(Color{16, 20, 26, 255});
            BeginMode3D(to_raylib(camera));
                DrawGrid(20, 1.0f);
                for (const auto& node : scene.nodes()) {
                    if (!node.visible || node.surface == nullptr) {
                        continue;
                    }

                    draw_control_net(*node.surface, selected_point);
                    CachedSurface& cached = m_surface_cache[node.id.value];
                    if (cached.revision != node.geometry_revision) {
                        cached.line_vertices = tessellate_wireframe(*node.surface, 100, 100);
                        cached.revision = node.geometry_revision;
                    }
                    for (std::size_t index = 0; index + 1 < cached.line_vertices.size(); index += 2) {
                        DrawLine3D(cached.line_vertices[index], cached.line_vertices[index + 1], BLUE);
                    }
                }
            EndMode3D();
        EndTextureMode();

        std::erase_if(m_surface_cache, [&scene](const auto& entry) {
            return scene.find_entity(EntityId{entry.first}) == nullptr;
        });
    }

    void composite(Rect destination) const {
        DrawTexturePro(
            m_target.texture,
            Rectangle{
                0.0f,
                0.0f,
                static_cast<float>(m_target.texture.width),
                -static_cast<float>(m_target.texture.height)
            },
            to_raylib(destination),
            Vector2{},
            0.0f,
            WHITE
        );
    }

private:
    struct CachedSurface {
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

    RenderTexture2D m_target{};
    std::unordered_map<std::uint64_t, CachedSurface> m_surface_cache;
};

RaylibViewportRenderer::RaylibViewportRenderer(int width, int height)
    : m_impl(std::make_unique<Impl>(width, height)) {}

RaylibViewportRenderer::~RaylibViewportRenderer() = default;

void RaylibViewportRenderer::resize(int width, int height) {
    m_impl->resize(width, height);
}

void RaylibViewportRenderer::render(
    const Scene& scene,
    const CameraState& camera,
    const ControlPoint* selected_point
) {
    m_impl->render(scene, camera, selected_point);
}

void RaylibViewportRenderer::composite(Rect destination) const {
    m_impl->composite(destination);
}
