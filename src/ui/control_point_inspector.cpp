#include "ui/control_point_inspector.h"

#include "ui/ui_widgets.h"

#include <algorithm>
#include <format>

ControlPointInspectorPanel::ControlPointInspectorPanel(
    Vec2 position,
    Scene& scene,
    SelectionModel& selection
)
    : m_panel(Rect{position.x, position.y, 300.0f, 250.0f}, "Control Point"),
      m_scene(scene),
      m_selection(selection) {}

void ControlPointInspectorPanel::inspect_point(ControlPointSelection selection) {
    m_selection.select(selection);
    rebuild_ui();
}

void ControlPointInspectorPanel::clear_selection() {
    m_selection.clear();
    m_panel.clear_children();
}

ControlPoint* ControlPointInspectorPanel::selected_point() const {
    const ControlPointSelection* selection = m_selection.control_point();
    return selection == nullptr ? nullptr : m_scene.resolve(*selection);
}

bool ControlPointInspectorPanel::handle_input(const InputFrameSnapshot& input) {
    return selected_point() != nullptr && m_panel.handle_input(input);
}

bool ControlPointInspectorPanel::has_pointer_capture() const {
    return selected_point() != nullptr && m_panel.has_pointer_capture();
}

void ControlPointInspectorPanel::render() const {
    if (selected_point() != nullptr) {
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
    ControlPoint* current_point = selected_point();
    const ControlPointSelection* selection = m_selection.control_point();
    if (current_point == nullptr || selection == nullptr) {
        return;
    }

    const Rect panel_bounds = m_panel.bounds();
    const float x = panel_bounds.x + 16.0f;
    const float first_slider_y = panel_bounds.y + 76.0f;
    const float slider_width = panel_bounds.width - 32.0f;
    const float position_minimum = static_cast<float>(std::min({
        -10.0,
        current_point->position.x,
        current_point->position.y,
        current_point->position.z
    }));
    const float position_maximum = static_cast<float>(std::max({
        10.0,
        current_point->position.x,
        current_point->position.y,
        current_point->position.z
    }));
    const float weight_minimum = static_cast<float>(std::min(0.05, current_point->weight));
    const float weight_maximum = static_cast<float>(std::max(5.0, current_point->weight));

    m_panel.add_child<UILabel>(
        Vec2{x, panel_bounds.y + 40.0f},
        std::format("Control vertex  U{} : V{}", selection->u, selection->v),
        14,
        Rgba{167, 178, 193, 255}
    );

    m_panel.add_child<UISlider>(
        Rect{x, first_slider_y, slider_width, 16.0f},
        "Position X", position_minimum, position_maximum,
        static_cast<float>(current_point->position.x),
        [this](float value) {
            if (ControlPoint* point = this->selected_point()) point->position.x = value;
        }
    );
    m_panel.add_child<UISlider>(
        Rect{x, first_slider_y + 42.0f, slider_width, 16.0f},
        "Position Y", position_minimum, position_maximum,
        static_cast<float>(current_point->position.y),
        [this](float value) {
            if (ControlPoint* point = this->selected_point()) point->position.y = value;
        }
    );
    m_panel.add_child<UISlider>(
        Rect{x, first_slider_y + 84.0f, slider_width, 16.0f},
        "Position Z", position_minimum, position_maximum,
        static_cast<float>(current_point->position.z),
        [this](float value) {
            if (ControlPoint* point = this->selected_point()) point->position.z = value;
        }
    );
    m_panel.add_child<UISlider>(
        Rect{x, first_slider_y + 126.0f, slider_width, 16.0f},
        "Weight W", weight_minimum, weight_maximum, static_cast<float>(current_point->weight),
        [this](float value) {
            if (ControlPoint* point = this->selected_point()) point->weight = value;
        }
    );
}
