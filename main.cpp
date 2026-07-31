#include "command_history.h"
#include "editor_chrome.h"
#include "input_frame.h"
#include "input_tools.h"
#include "raylib.h"

#include "core.h"
#include "editor_layout.h"
#include "nurbs_surface.h"
#include "raylib_input.h"
#include "raylib_ui_renderer.h"
#include "raylib_viewport_renderer.h"
#include "scene.h"
#include "topology.h"
#include "orbit_camera.h"
#include "ui/control_point_inspector.h"
#include "ui/ui_layer.h"

#include <memory>
#include <vector>

int run_editor() {
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

    Scene scene;
    [[maybe_unused]] const EntityId surface_id = scene.add_entity("WaveSurface", std::move(surface));

    SelectionModel selection;
    CommandHistory history;
    UILayer ui_layer;
    auto* inspector = ui_layer.add_element<ControlPointInspectorPanel>(
        Vec2{10.0f, 92.0f},
        scene,
        selection,
        history
    );

    InputToolDispatcher input_dispatcher;
    input_dispatcher.register_tools<CameraNavigationTool>();
    input_dispatcher.register_tools<ControlPointSelectionTool>(
        [inspector](ControlPointSelection selected) { inspector->inspect_point(selected); },
        [inspector] { inspector->clear_selection(); }
    );

    EditorLayout layout = EditorLayout::calculate(GetScreenWidth(), GetScreenHeight());
    RaylibViewportRenderer viewport_renderer(
        static_cast<int>(layout.viewport.width),
        static_cast<int>(layout.viewport.height)
    );
    RaylibUiRenderer ui_renderer;
    bool viewport_has_pointer_capture = false;

    while (!WindowShouldClose()) {
        const EditorLayout next_layout = EditorLayout::calculate(GetScreenWidth(), GetScreenHeight());
        if (next_layout.viewport.width != layout.viewport.width ||
            next_layout.viewport.height != layout.viewport.height) {
            viewport_renderer.resize(
                static_cast<int>(next_layout.viewport.width),
                static_cast<int>(next_layout.viewport.height)
            );
        }
        layout = next_layout;

        InputFrameSnapshot input = capture_raylib_input_frame();
        if ((input.undo_pressed && history.undo()) ||
            (input.redo_pressed && history.redo())) {
            inspector->refresh();
        }
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

        viewport_renderer.render(scene, camera_controller.camera(), inspector->selected_point());

        BeginDrawing();
            ClearBackground(Color{16, 20, 26, 255});
            viewport_renderer.composite(layout.viewport);
            render_editor_chrome(ui_renderer, layout, !selection.empty(), GetFPS());
            ui_layer.render(ui_renderer);
        EndDrawing();
    }

    return 0;
}

int main() {
    InitWindow(1280, 720, "Nurbsman");
    SetWindowState(FLAG_WINDOW_RESIZABLE);
    SetWindowMinSize(800, 500);
    SetTargetFPS(60);

    const int result = run_editor();
    CloseWindow();
    return result;
}
