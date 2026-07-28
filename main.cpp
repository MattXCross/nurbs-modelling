#include "input_frame.h"
#include "input_tools.h"
#include "raylib.h"
#include "raymath.h"

#include "core.h"
#include "nurbs_surface.h"
#include "scene.h"
#include "topology.h"
#include "orbit_camera.h"

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
    InitWindow(1280, 720, "Nurbsman");
    SetTargetFPS(60);

    OrbitCameraController camera_controller(
      Vector3{10.0f, 10.0f, 10.0f},
      Vector3{0.0, 0.0, 0.0}
    );

    // Demonstration of Topology Graph (shared_ptr / weak_ptr)
    auto edge = std::make_shared<CadEdge>(Point3D{0,0,0}, Point3D{1,0,0});
    auto face1 = std::make_shared<CadFace>("Face1");
    face1->boundary_edges.push_back(edge);
    edge->adjacent_faces.push_back(face1);

    // Define 3x3 control point grid
    std::vector<ControlPoint> points = {
        {{ -4, 0, -4 }, 1.0}, {{ -1, 3, -4 }, 1.0}, {{ 2, -2, -4 }, 1.0},
        {{ -4, 1,  0 }, 1.0}, {{ -1, 5,  0 }, 3.0}, {{ 2,  1,  0 }, 1.0},
        {{ -4, 0,  4 }, 1.0}, {{ -1, 2,  4 }, 1.0}, {{ 2,  0,  4 }, 1.0}
    };

    // Create surface and apply C++23 "deduce this" method chaining
    auto surface = std::make_unique<NurbsSurface>(3, 3, std::move(points));
    surface->translate(Point3D{1.0, 0.0, 0.0});

    Scene scene;
    scene.add_entity("WaveSurface", std::move(surface));

    InputToolDispatcher input_despatcher;
    input_despatcher.register_tools<CameraNavigationTool>();

    while (!WindowShouldClose()) {
        InputFrameSnapshot input = InputFrameSnapshot::capture_input_frame();
        input_despatcher.dispatch(input, camera_controller, scene);

        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera_controller.raw_camera());
                DrawGrid(20, 1.0f);

                for (const auto& node : scene.nodes()) {
                    if (node.surface) {
                        draw_control_net(*node.surface);
                        draw_surface_wireframe(*node.surface, 100, 100);
                    }
                }
            EndMode3D();

            DrawText("Middle Drag: Orbit | Shift + Middle Drag: Pan | Scroll: Zoom", 10, 10, 20, GRAY);
            const char* fps_text = TextFormat("FPS: %d", GetFPS());
            DrawText(fps_text, GetScreenWidth() - MeasureText(fps_text, 20) - 10, 10, 20, DARKGRAY);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
