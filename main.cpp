#include "input_frame.h"
#include "input_tools.h"
#include "raylib.h"

#include "core.h"
#include "editor_layout.h"
#include "nurbs_surface.h"
#include "scene.h"
#include "topology.h"
#include "orbit_camera.h"
#include "ui/control_point_inspector.h"
#include "ui/ui_layer.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

inline Vector3 to_raylib(const Point3D& p) {
    return Vector3{ static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z) };
}

Vector2 to_raylib(Vec2 vector) {
    return {vector.x, vector.y};
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

InputFrameSnapshot capture_input_frame() {
    const Vector2 mouse_position = GetMousePosition();
    const Vector2 mouse_delta = GetMouseDelta();
    return {
        .mouse_position = {mouse_position.x, mouse_position.y},
        .mouse_delta = {mouse_delta.x, mouse_delta.y},
        .mouse_wheel_delta = GetMouseWheelMove(),
        .screen_width = GetScreenWidth(),
        .screen_height = GetScreenHeight(),
        .middle_mouse = IsMouseButtonDown(MOUSE_MIDDLE_BUTTON),
        .left_mouse = IsMouseButtonDown(MOUSE_LEFT_BUTTON),
        .right_mouse = IsMouseButtonDown(MOUSE_RIGHT_BUTTON),
        .left_mouse_pressed = IsMouseButtonPressed(MOUSE_LEFT_BUTTON),
        .left_mouse_released = IsMouseButtonReleased(MOUSE_LEFT_BUTTON),
        .modifiers = {
            .shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT),
            .ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL),
            .alt = IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT)
        }
    };
}

void draw_control_net(const NurbsSurface& surface, const ControlPoint* selected_point = nullptr) {
    auto net = surface.control_net_2d(); // C++23 std::mdspan
    for (size_t u = 0; u < net.extent(0); ++u) {
        for (size_t v = 0; v < net.extent(1); ++v) {
            const bool is_selected = &net[u, v] == selected_point;
            Vector3 pos = to_raylib(net[u, v].position);
            DrawSphere(pos, is_selected ? 0.24f : 0.15f, is_selected ? GOLD : RED);

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

void draw_toolbar_button(Rect bounds, const char* label, bool active = false) {
    const Color fill = active ? Color{45, 108, 145, 255} : Color{37, 45, 57, 255};
    DrawRectangleRec(to_raylib(bounds), fill);
    DrawRectangleLinesEx(to_raylib(bounds), 1.0f, Color{69, 82, 99, 255});
    const int font_size = 15;
    const int text_width = MeasureText(label, font_size);
    DrawText(
        label,
        static_cast<int>(bounds.x + (bounds.width - static_cast<float>(text_width)) * 0.5f),
        static_cast<int>(bounds.y + (bounds.height - static_cast<float>(font_size)) * 0.5f),
        font_size,
        active ? RAYWHITE : Color{190, 199, 211, 255}
    );
}

void draw_editor_chrome(const EditorLayout& layout, bool has_selection) {
    DrawRectangleRec(to_raylib(layout.toolbar), Color{23, 28, 36, 255});
    DrawLine(
        0,
        static_cast<int>(layout.toolbar.height - 1.0f),
        static_cast<int>(layout.toolbar.width),
        static_cast<int>(layout.toolbar.height - 1.0f),
        Color{69, 82, 99, 255}
    );

    DrawText("NURBSMAN", 16, 15, 18, Color{126, 191, 236, 255});
    draw_toolbar_button({142.0f, 8.0f, 72.0f, 32.0f}, "Select", true);
    draw_toolbar_button({222.0f, 8.0f, 72.0f, 32.0f}, "Create");
    draw_toolbar_button({302.0f, 8.0f, 72.0f, 32.0f}, "Modify");
    draw_toolbar_button({382.0f, 8.0f, 72.0f, 32.0f}, "View");

    DrawRectangleRec(to_raylib(layout.inspector), Color{20, 25, 32, 255});
    DrawLine(
        static_cast<int>(layout.inspector.width - 1.0f),
        static_cast<int>(layout.inspector.y),
        static_cast<int>(layout.inspector.width - 1.0f),
        static_cast<int>(layout.inspector.y + layout.inspector.height),
        Color{69, 82, 99, 255}
    );
    DrawText("PROPERTIES", 16, 66, 13, Color{126, 139, 156, 255});

    if (!has_selection) {
        DrawText("Nothing selected", 16, 102, 16, Color{132, 143, 157, 255});
        DrawText("Select a control point in the viewport", 16, 128, 13, Color{91, 103, 118, 255});
    }
}

int main() {
    InitWindow(1280, 720, "Nurbsman");
    SetWindowState(FLAG_WINDOW_RESIZABLE);
    SetWindowMinSize(800, 500);
    SetTargetFPS(60);

    OrbitCameraController camera_controller(
      Vec3{10.0f, 10.0f, 10.0f},
      Vec3{0.0f, 0.0f, 0.0f}
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

    UILayer ui_layer;
    auto* inspector = ui_layer.add_element<ControlPointInspectorPanel>(Vec2{10.0f, 92.0f});

    Scene scene;
    scene.add_entity("WaveSurface", std::move(surface));

    InputToolDispatcher input_dispatcher;
    input_dispatcher.register_tools<CameraNavigationTool>();
    input_dispatcher.register_tools<ControlPointSelectionTool>(
        [inspector](NurbsSurface&, size_t u, size_t v, ControlPoint& point) {
            inspector->inspect_point(u, v, &point);
        },
        [inspector] { inspector->clear_selection(); }
    );

    EditorLayout layout = EditorLayout::calculate(GetScreenWidth(), GetScreenHeight());
    RenderTexture2D viewport_target = LoadRenderTexture(
        static_cast<int>(layout.viewport.width),
        static_cast<int>(layout.viewport.height)
    );
    bool viewport_has_pointer_capture = false;

    while (!WindowShouldClose()) {
        const EditorLayout next_layout = EditorLayout::calculate(GetScreenWidth(), GetScreenHeight());
        if (next_layout.viewport.width != layout.viewport.width ||
            next_layout.viewport.height != layout.viewport.height) {
            UnloadRenderTexture(viewport_target);
            viewport_target = LoadRenderTexture(
                static_cast<int>(next_layout.viewport.width),
                static_cast<int>(next_layout.viewport.height)
            );
        }
        layout = next_layout;

        InputFrameSnapshot input = capture_input_frame();
        const bool ui_consumed_input = ui_layer.handle_input(input);
        const bool pointer_in_viewport = layout.viewport.contains(input.mouse_position);
        if (!input.middle_mouse) {
            viewport_has_pointer_capture = false;
        } else if (pointer_in_viewport) {
            viewport_has_pointer_capture = true;
        }

        if (!ui_consumed_input && (pointer_in_viewport || viewport_has_pointer_capture)) {
            InputFrameSnapshot viewport_input = input;
            viewport_input.mouse_position = {
                input.mouse_position.x - layout.viewport.x,
                input.mouse_position.y - layout.viewport.y
            };
            viewport_input.screen_width = static_cast<int>(layout.viewport.width);
            viewport_input.screen_height = static_cast<int>(layout.viewport.height);
            input_dispatcher.dispatch(viewport_input, camera_controller, scene);
        }

        BeginTextureMode(viewport_target);
            ClearBackground(Color{16, 20, 26, 255});
            BeginMode3D(to_raylib(camera_controller.camera()));
                DrawGrid(20, 1.0f);

                for (const auto& node : scene.nodes()) {
                    if (node.visible && node.surface) {
                        draw_control_net(*node.surface, inspector->selected_point());
                        draw_surface_wireframe(*node.surface, 100, 100);
                    }
                }
            EndMode3D();
        EndTextureMode();

        BeginDrawing();
            ClearBackground(Color{16, 20, 26, 255});
            DrawTexturePro(
                viewport_target.texture,
                Rectangle{
                    0.0f,
                    0.0f,
                    static_cast<float>(viewport_target.texture.width),
                    -static_cast<float>(viewport_target.texture.height)
                },
                to_raylib(layout.viewport),
                Vector2{},
                0.0f,
                WHITE
            );
            draw_editor_chrome(layout, inspector->selected_point() != nullptr);

            DrawText(
                "LMB Select   MMB Orbit   Shift + MMB Pan   Wheel Zoom",
                static_cast<int>(layout.viewport.x + 14.0f),
                static_cast<int>(layout.viewport.y + layout.viewport.height - 28.0f),
                14,
                Color{154, 165, 179, 255}
            );
            const char* fps_text = TextFormat("FPS: %d", GetFPS());
            DrawText(
                fps_text,
                GetScreenWidth() - MeasureText(fps_text, 15) - 16,
                17,
                15,
                Color{154, 165, 179, 255}
            );
            ui_layer.render();
        EndDrawing();
    }

    UnloadRenderTexture(viewport_target);
    CloseWindow();
    return 0;
}
