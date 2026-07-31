#pragma once

#include "math_types.h"

#include <string_view>

class IUiRenderer {
public:
    virtual ~IUiRenderer() = default;

    virtual void fill_rect(Rect bounds, Rgba color) = 0;
    virtual void stroke_rect(Rect bounds, float thickness, Rgba color) = 0;
    virtual void draw_text(std::string_view text, Vec2 position, int font_size, Rgba color) = 0;
    [[nodiscard]] virtual Vec2 measure_text(std::string_view text, int font_size) const = 0;
};
