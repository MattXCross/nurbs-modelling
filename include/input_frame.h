#pragma once

#include "math_types.h"

struct ModifierKeys {
    bool shift{false}, ctrl{false}, alt{false};
};

struct InputFrameSnapshot {
    Vec2 mouse_position{}, mouse_delta{};
    float mouse_wheel_delta{0.0f};
    int screen_width{0}, screen_height{0};

    bool middle_mouse{false}, left_mouse{false}, right_mouse{false};
    bool left_mouse_pressed{false}, left_mouse_released{false};

    ModifierKeys modifiers;
};
