#include "editor_chrome.h"
#include "editor_session.h"
#include "input_frame.h"
#include "raylib.h"

#include "editor_layout.h"
#include "raylib_input.h"
#include "raylib_ui_renderer.h"
#include "raylib_viewport_renderer.h"
#include "ui/control_point_inspector.h"
#include "ui/ui_layer.h"

int run_editor() {
    EditorSession session;
    UILayer ui_layer;
    auto* inspector = ui_layer.add_element<ControlPointInspectorPanel>(
        Vec2{10.0f, 92.0f},
        session
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
        if ((input.undo_pressed && session.undo()) ||
            (input.redo_pressed && session.redo())) {
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
            if (session.process_viewport_input(viewport_input)) {
                inspector->refresh();
            }
        }

        viewport_renderer.render(session.scene(), session.camera(), session.selected_control_point());

        BeginDrawing();
            ClearBackground(Color{16, 20, 26, 255});
            viewport_renderer.composite(layout.viewport);
            render_editor_chrome(ui_renderer, layout, !session.selection().empty(), GetFPS());
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
