#pragma once

#include "math_types.h"

struct EditorLayout {
    Rect toolbar;
    Rect inspector;
    Rect viewport;

    [[nodiscard]] static EditorLayout calculate(int window_width, int window_height);
};
