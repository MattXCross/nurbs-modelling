#include "ui/ui_panel.h"

#include "raylib.h"

#include <utility>

namespace {

Rectangle to_raylib(Rect rectangle) {
    return {rectangle.x, rectangle.y, rectangle.width, rectangle.height};
}

} // namespace

UIPanel::UIPanel(Rect bounds, std::string title)
    : m_bounds(bounds), m_title(std::move(title)) {}

void UIPanel::clear_children() {
    m_children.clear();
}

bool UIPanel::handle_input(const InputFrameSnapshot& input) {
    for (auto child = m_children.rbegin(); child != m_children.rend(); ++child) {
        if ((*child)->has_pointer_capture()) {
            return (*child)->handle_input(input);
        }
    }

    for (auto child = m_children.rbegin(); child != m_children.rend(); ++child) {
        if ((*child)->handle_input(input)) {
            return true;
        }
    }

    return m_bounds.contains(input.mouse_position);
}

bool UIPanel::has_pointer_capture() const {
    for (const auto& child : m_children) {
        if (child->has_pointer_capture()) {
            return true;
        }
    }

    return false;
}

void UIPanel::render() const {
    DrawRectangleRec(to_raylib(m_bounds), Color{28, 34, 43, 238});
    DrawRectangleLinesEx(to_raylib(m_bounds), 1.0f, Color{86, 99, 116, 255});

    const Rect header{m_bounds.x, m_bounds.y, m_bounds.width, 30.0f};
    DrawRectangleRec(to_raylib(header), Color{37, 45, 57, 255});
    DrawRectangleLinesEx(to_raylib(header), 1.0f, Color{86, 99, 116, 255});
    DrawText(
        m_title.c_str(),
        static_cast<int>(m_bounds.x + 12.0f),
        static_cast<int>(m_bounds.y + 7.0f),
        16,
        Color{126, 191, 236, 255}
    );

    for (const auto& child : m_children) {
        child->render();
    }
}

void UIPanel::set_position(Vec2 position) {
    const Vec2 offset{position.x - m_bounds.x, position.y - m_bounds.y};
    m_bounds.x = position.x;
    m_bounds.y = position.y;

    for (const auto& child : m_children) {
        const Rect child_bounds = child->bounds();
        child->set_position({child_bounds.x + offset.x, child_bounds.y + offset.y});
    }
}

Rect UIPanel::bounds() const {
    return m_bounds;
}
