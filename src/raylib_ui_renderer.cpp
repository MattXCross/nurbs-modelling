#include "raylib_ui_renderer.h"

#include "raylib.h"

#include <string>

namespace {

Rectangle to_raylib(Rect rectangle) {
    return {rectangle.x, rectangle.y, rectangle.width, rectangle.height};
}

Color to_raylib(Rgba color) {
    return {color.red, color.green, color.blue, color.alpha};
}

} // namespace

void RaylibUiRenderer::fill_rect(Rect bounds, Rgba color) {
    DrawRectangleRec(to_raylib(bounds), to_raylib(color));
}

void RaylibUiRenderer::stroke_rect(Rect bounds, float thickness, Rgba color) {
    DrawRectangleLinesEx(to_raylib(bounds), thickness, to_raylib(color));
}

void RaylibUiRenderer::draw_text(
    std::string_view text,
    Vec2 position,
    int font_size,
    Rgba color
) {
    const std::string owned_text{text};
    DrawText(
        owned_text.c_str(),
        static_cast<int>(position.x),
        static_cast<int>(position.y),
        font_size,
        to_raylib(color)
    );
}

Vec2 RaylibUiRenderer::measure_text(std::string_view text, int font_size) const {
    const std::string owned_text{text};
    return {
        static_cast<float>(MeasureText(owned_text.c_str(), font_size)),
        static_cast<float>(font_size)
    };
}
