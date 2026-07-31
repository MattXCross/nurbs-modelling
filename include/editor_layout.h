#pragma once

#include "raylib.h"

struct EditorLayout {
    Rectangle toolbar;
    Rectangle inspector;
    Rectangle viewport;

    [[nodiscard]] static EditorLayout calculate(int window_width, int window_height);
};
