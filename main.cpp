#include "raylib.h"
#include "raymath.h"
#include "core.h"
#include "nurbs_surface.h"
#include "scene.h"
#include "topology.h"

#include <algorithm>
#include <cmath>

inline Vector3 to_raylib(const Point3D& p) {
    return Vector3{ static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z) };
}

void draw_control_net(const NurbsSurface& surface) {
    auto net = surface.control_net_2d(); // C++23 std::mdspan
    for (size_t u = 0; u < net.extent(0); ++u) {
        for (size_t v = 0; v < net.extent(1); ++v) {
            Vector3 pos = to_raylib(net[u, v].position);
            DrawSphere(pos, 0.15f, RED);

            if (u + 1 < net.extent(0)) {
                DrawLine3D(pos, to_raylib(net[u + 1, v].position), GRAY);
            }
            if (v + 1 < net.extent(1)) {
                DrawLine3D(pos, to_raylib(net[u, v + 1].position), GRAY);
            }
        }
    }
}

void draw_surface_wireframe(const NurbsSurface& surface, int u_samples = 20, int v_samples = 20) {
    float du = 1.0f / u_samples;
    float dv = 1.0f / v_samples;

    for (int i = 0; i < u_samples; ++i) {
        for (int j = 0; j < v_samples; ++j) {
            auto p00 = surface.evaluate(i * du, j * dv);
            auto p10 = surface.evaluate((i + 1) * du, j * dv);
            auto p01 = surface.evaluate(i * du, (j + 1) * dv);
            auto p11 = surface.evaluate((i + 1) * du, (j + 1) * dv);

            if (p00 && p10 && p01 && p11) {
                Vector3 v00 = to_raylib(*p00);
                Vector3 v10 = to_raylib(*p10);
                Vector3 v01 = to_raylib(*p01);
                Vector3 v11 = to_raylib(*p11);

                DrawLine3D(v00, v10, BLUE);
                DrawLine3D(v10, v11, BLUE);
                DrawLine3D(v11, v01, BLUE);
                DrawLine3D(v01, v00, BLUE);
            }
        }
    }
}

int main() {
    InitWindow(1280, 720, "Modern C++23 CAD Engine - Raylib Viewport");
    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = Vector3{ 10.0f, 10.0f, 10.0f };
    camera.target   = Vector3{ 0.0f, 0.0f, 0.0f };
    camera.up       = Vector3{ 0.0f, 1.0f, 0.0f };
    camera.fovy     = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    constexpr float orbit_sensitivity = 0.005f;
    constexpr float pitch_limit = 1.5f;
    constexpr float min_camera_distance = 1.0f;
    constexpr float max_camera_distance = 100.0f;

    Vector3 camera_offset = {
        camera.position.x - camera.target.x,
        camera.position.y - camera.target.y,
        camera.position.z - camera.target.z
    };
    float camera_distance = std::sqrt(camera_offset.x * camera_offset.x
                                    + camera_offset.y * camera_offset.y
                                    + camera_offset.z * camera_offset.z);
    float camera_yaw = std::atan2(camera_offset.x, camera_offset.z);
    float camera_pitch = std::asin(camera_offset.y / camera_distance);

    // Demonstration of Topology Graph (shared_ptr / weak_ptr)
    auto edge = std::make_shared<CadEdge>(Point3D{0,0,0}, Point3D{1,0,0});
    auto face1 = std::make_shared<CadFace>("Face1");
    face1->boundary_edges.push_back(edge);
    edge->adjacent_faces.push_back(face1);

    // Define 3x3 control point grid
    std::vector<ControlPoint> points = {
        {{ -4, 0, -4 }, 1.0}, {{ -1, 3, -4 }, 1.0}, {{ 2, -2, -4 }, 1.0},
        {{ -4, 1,  0 }, 1.0}, {{ -1, 5,  0 }, 1.0}, {{ 2,  1,  0 }, 1.0},
        {{ -4, 0,  4 }, 1.0}, {{ -1, 2,  4 }, 1.0}, {{ 2,  0,  4 }, 1.0}
    };

    // Create surface and apply C++23 "deduce this" method chaining
    auto surface = std::make_unique<NurbsSurface>(3, 3, std::move(points));
    surface->translate(Point3D{1.0, 0.0, 0.0});

    Scene scene;
    scene.add_entity("WaveSurface", std::move(surface));

    while (!WindowShouldClose()) {
        if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
            Vector2 mouse_delta = GetMouseDelta();
            bool is_panning = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);

            if (is_panning) {
                Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
                Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, camera.up));
                Vector3 screen_up = Vector3Normalize(Vector3CrossProduct(right, forward));
                float pan_scale = 2.0f * camera_distance
                                * std::tan(camera.fovy * DEG2RAD * 0.5f)
                                / GetScreenHeight();

                camera.target = Vector3Add(camera.target, Vector3Add(
                    Vector3Scale(right, -mouse_delta.x * pan_scale),
                    Vector3Scale(screen_up, mouse_delta.y * pan_scale)
                ));
            } else {
                camera_yaw -= mouse_delta.x * orbit_sensitivity;
                camera_pitch += mouse_delta.y * orbit_sensitivity;
                camera_pitch = std::clamp(camera_pitch, -pitch_limit, pitch_limit);
            }
        }

        float wheel = GetMouseWheelMove();
        camera_distance *= 1.0f - wheel * 0.1f;
        camera_distance = std::clamp(camera_distance, min_camera_distance, max_camera_distance);

        float horizontal_distance = camera_distance * std::cos(camera_pitch);
        camera.position = Vector3{
            camera.target.x + horizontal_distance * std::sin(camera_yaw),
            camera.target.y + camera_distance * std::sin(camera_pitch),
            camera.target.z + horizontal_distance * std::cos(camera_yaw)
        };

        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);
                DrawGrid(20, 1.0f);

                for (const auto& node : scene.nodes()) {
                    if (node.surface) {
                        draw_control_net(*node.surface);
                        draw_surface_wireframe(*node.surface, 25, 25);
                    }
                }
            EndMode3D();

            DrawText("Raylib 3D Viewport - Modern C++23 CAD Engine", 10, 10, 20, DARKGRAY);
            DrawText("Middle Drag: Orbit | Shift + Middle Drag: Pan | Scroll: Zoom", 10, 35, 16, GRAY);
            const char* fps_text = TextFormat("FPS: %d", GetFPS());
            DrawText(fps_text, GetScreenWidth() - MeasureText(fps_text, 20) - 10, 10, 20, DARKGRAY);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
