#pragma once

#include "ui/ui_renderer.h"

class RaylibUiRenderer final : public IUiRenderer {
public:
    void fill_rect(Rect bounds, Rgba color) override;
    void stroke_rect(Rect bounds, float thickness, Rgba color) override;
    void draw_text(std::string_view text, Vec2 position, int font_size, Rgba color) override;
    [[nodiscard]] Vec2 measure_text(std::string_view text, int font_size) const override;
};
