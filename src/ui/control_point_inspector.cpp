#include "ui/control_point_inspector.h"

#include "ui/ui_widgets.h"

#include <algorithm>
#include <format>

ControlPointInspectorPanel::ControlPointInspectorPanel(Vec2 position)
    : m_panel(Rect{position.x, position.y, 300.0f, 250.0f}, "Control Point") {}

void ControlPointInspectorPanel::inspect_point(size_t u, size_t v, ControlPoint* point) {
    m_selected_u = u;
    m_selected_v = v;
    m_selected_point = point;
    rebuild_ui();
}

void ControlPointInspectorPanel::clear_selection() {
    m_selected_point = nullptr;
    m_panel.clear_children();
}

ControlPoint* ControlPointInspectorPanel::selected_point() const {
    return m_selected_point;
}

bool ControlPointInspectorPanel::handle_input(const InputFrameSnapshot& input) {
    return m_selected_point != nullptr && m_panel.handle_input(input);
}

bool ControlPointInspectorPanel::has_pointer_capture() const {
    return m_selected_point != nullptr && m_panel.has_pointer_capture();
}

void ControlPointInspectorPanel::render() const {
    if (m_selected_point != nullptr) {
        m_panel.render();
    }
}

void ControlPointInspectorPanel::set_position(Vec2 position) {
    m_panel.set_position(position);
}

Rect ControlPointInspectorPanel::bounds() const {
    return m_panel.bounds();
}

void ControlPointInspectorPanel::rebuild_ui() {
    m_panel.clear_children();
    if (m_selected_point == nullptr) {
        return;
    }

    const Rect panel_bounds = m_panel.bounds();
    const float x = panel_bounds.x + 16.0f;
    const float first_slider_y = panel_bounds.y + 76.0f;
    const float slider_width = panel_bounds.width - 32.0f;
    const float position_minimum = static_cast<float>(std::min({
        -10.0,
        m_selected_point->position.x,
        m_selected_point->position.y,
        m_selected_point->position.z
    }));
    const float position_maximum = static_cast<float>(std::max({
        10.0,
        m_selected_point->position.x,
        m_selected_point->position.y,
        m_selected_point->position.z
    }));
    const float weight_minimum = static_cast<float>(std::min(0.05, m_selected_point->weight));
    const float weight_maximum = static_cast<float>(std::max(5.0, m_selected_point->weight));

    m_panel.add_child<UILabel>(
        Vec2{x, panel_bounds.y + 40.0f},
        std::format("Control vertex  U{} : V{}", m_selected_u, m_selected_v),
        14,
        Rgba{167, 178, 193, 255}
    );

    m_panel.add_child<UISlider>(
        Rect{x, first_slider_y, slider_width, 16.0f},
        "Position X", position_minimum, position_maximum,
        static_cast<float>(m_selected_point->position.x),
        [this](float value) { if (m_selected_point) m_selected_point->position.x = value; }
    );
    m_panel.add_child<UISlider>(
        Rect{x, first_slider_y + 42.0f, slider_width, 16.0f},
        "Position Y", position_minimum, position_maximum,
        static_cast<float>(m_selected_point->position.y),
        [this](float value) { if (m_selected_point) m_selected_point->position.y = value; }
    );
    m_panel.add_child<UISlider>(
        Rect{x, first_slider_y + 84.0f, slider_width, 16.0f},
        "Position Z", position_minimum, position_maximum,
        static_cast<float>(m_selected_point->position.z),
        [this](float value) { if (m_selected_point) m_selected_point->position.z = value; }
    );
    m_panel.add_child<UISlider>(
        Rect{x, first_slider_y + 126.0f, slider_width, 16.0f},
        "Weight W", weight_minimum, weight_maximum, static_cast<float>(m_selected_point->weight),
        [this](float value) { if (m_selected_point) m_selected_point->weight = value; }
    );
}
