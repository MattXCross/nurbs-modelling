#include "raylib_input.h"

#include "raylib.h"

InputFrameSnapshot capture_raylib_input_frame() {
    const Vector2 mouse_position = GetMousePosition();
    const Vector2 mouse_delta = GetMouseDelta();
    const bool shift_down = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
    const bool ctrl_down = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
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
        .undo_pressed = ctrl_down && !shift_down && IsKeyPressed(KEY_Z),
        .redo_pressed = ctrl_down && (IsKeyPressed(KEY_Y) || (shift_down && IsKeyPressed(KEY_Z))),
        .modifiers = {
            .shift = shift_down,
            .ctrl = ctrl_down,
            .alt = IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT)
        }
    };
}
