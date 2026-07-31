#include "ui/ui_widgets.h"

#include "raylib.h"

#include <algorithm>
#include <format>
#include <utility>

namespace {

Rectangle to_raylib(Rect rectangle) {
    return {rectangle.x, rectangle.y, rectangle.width, rectangle.height};
}

Color to_raylib(Rgba color) {
    return {color.red, color.green, color.blue, color.alpha};
}

} // namespace

UILabel::UILabel(Vec2 position, std::string text, int font_size, Rgba color)
    : m_text(std::move(text)), m_color(color), m_font_size(font_size) {
    m_bounds = {
        position.x,
        position.y,
        static_cast<float>(MeasureText(m_text.c_str(), m_font_size)),
        static_cast<float>(m_font_size)
    };
}

void UILabel::set_text(std::string text) {
    m_text = std::move(text);
    m_bounds.width = static_cast<float>(MeasureText(m_text.c_str(), m_font_size));
}

bool UILabel::handle_input(const InputFrameSnapshot&) {
    return false;
}

void UILabel::render() const {
    DrawText(
        m_text.c_str(),
        static_cast<int>(m_bounds.x),
        static_cast<int>(m_bounds.y),
        m_font_size,
        to_raylib(m_color)
    );
}

void UILabel::set_position(Vec2 position) {
    m_bounds.x = position.x;
    m_bounds.y = position.y;
}

Rect UILabel::bounds() const {
    return m_bounds;
}

UISlider::UISlider(
    Rect bounds,
    std::string label,
    float minimum,
    float maximum,
    float initial_value,
    std::move_only_function<void(float)> on_value_changed
)
    : m_bounds(bounds),
      m_label(std::move(label)),
      m_minimum(std::min(minimum, maximum)),
      m_maximum(std::max(minimum, maximum)),
      m_on_value_changed(std::move(on_value_changed)) {
    set_value(initial_value);
}

void UISlider::set_value(float value) {
    m_value = std::clamp(value, m_minimum, m_maximum);
}

float UISlider::value() const {
    return m_value;
}

bool UISlider::is_dragging() const {
    return m_is_dragging;
}

bool UISlider::handle_input(const InputFrameSnapshot& input) {
    const bool mouse_over = m_bounds.contains(input.mouse_position);
    if (input.left_mouse_pressed && mouse_over) {
        m_is_dragging = true;
    }

    if (m_is_dragging) {
        update_from_mouse(input.mouse_position.x);
        if (input.left_mouse_released || !input.left_mouse) {
            m_is_dragging = false;
        }
        return true;
    }

    return mouse_over;
}

bool UISlider::has_pointer_capture() const {
    return m_is_dragging;
}

void UISlider::render() const {
    DrawRectangleRec(to_raylib(m_bounds), Color{45, 53, 66, 255});

    const float filled_width = normalized_value() * m_bounds.width;
    DrawRectangleRec(
        Rectangle{m_bounds.x, m_bounds.y, filled_width, m_bounds.height},
        Color{43, 144, 217, 255}
    );
    DrawRectangleLinesEx(to_raylib(m_bounds), 1.0f, Color{103, 116, 134, 255});

    const int handle_x = static_cast<int>(m_bounds.x + filled_width - 3.0f);
    DrawRectangle(
        handle_x,
        static_cast<int>(m_bounds.y - 2.0f),
        6,
        static_cast<int>(m_bounds.height + 4.0f),
        Color{196, 224, 246, 255}
    );

    const std::string text = std::format("{}  {:.2f}", m_label, m_value);
    DrawText(
        text.c_str(),
        static_cast<int>(m_bounds.x),
        static_cast<int>(m_bounds.y - 18.0f),
        14,
        Color{205, 214, 225, 255}
    );
}

void UISlider::set_position(Vec2 position) {
    m_bounds.x = position.x;
    m_bounds.y = position.y;
}

Rect UISlider::bounds() const {
    return m_bounds;
}

float UISlider::normalized_value() const {
    const float range = m_maximum - m_minimum;
    return range == 0.0f ? 0.0f : (m_value - m_minimum) / range;
}

void UISlider::update_from_mouse(float mouse_x) {
    const float normalized = m_bounds.width <= 0.0f
        ? 0.0f
        : std::clamp((mouse_x - m_bounds.x) / m_bounds.width, 0.0f, 1.0f);
    const float new_value = m_minimum + normalized * (m_maximum - m_minimum);
    if (new_value == m_value) {
        return;
    }

    m_value = new_value;
    if (m_on_value_changed) {
        m_on_value_changed(m_value);
    }
}
