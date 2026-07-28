#pragma once

#include "raylib.h"

struct ModifierKeys {
    bool shift{false}, crtl{false}, alt{false};
};

struct InputFrameSnapshot {
    Vector2 mouse_position{0.0f, 0.0f}, mouse_delta{0.0f, 0.0f};
    float mouse_wheel_delta{0.0f};

    bool middle_mouse{false}, left_mouse{false}, right_mouse{false};

    ModifierKeys modifiers;

    static InputFrameSnapshot capture_input_frame() {
        return InputFrameSnapshot{
            .mouse_position = GetMousePosition(),
            .mouse_delta = GetMouseDelta(),
            .mouse_wheel_delta = GetMouseWheelMove(),
            .middle_mouse = IsMouseButtonDown(MOUSE_MIDDLE_BUTTON),
            .left_mouse = IsMouseButtonDown(MOUSE_LEFT_BUTTON),
            .right_mouse = IsMouseButtonDown(MOUSE_RIGHT_BUTTON),
            .modifiers = {
                .shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT),
                .crtl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL),
                .alt = IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT)
            }
        };
    }
};