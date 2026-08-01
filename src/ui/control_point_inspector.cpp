#include "ui/control_point_inspector.h"

#include "ui/ui_widgets.h"

#include <algorithm>
#include <format>
ControlPointInspectorPanel::ControlPointInspectorPanel(
    Vec2 position,
    EditorSession& session
)
    : m_panel(Rect{position.x, position.y, 300.0f, 250.0f}, "Control Point"),
      m_session(session) {}

void ControlPointInspectorPanel::refresh() {
    rebuild_ui();
}

const ControlPoint* ControlPointInspectorPanel::selected_point() const {
    return m_session.selected_control_point();
}

bool ControlPointInspectorPanel::handle_input(const InputFrameSnapshot& input) {
    return selected_point() != nullptr && m_panel.handle_input(input);
}

bool ControlPointInspectorPanel::has_pointer_capture() const {
    return selected_point() != nullptr && m_panel.has_pointer_capture();
}

void ControlPointInspectorPanel::render(IUiRenderer& renderer) const {
    if (selected_point() != nullptr) {
        m_panel.render(renderer);
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
    const ControlPoint* current_point = selected_point();
    const ControlPointSelection* selection = m_session.selection().control_point();
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
        [this](float value) { m_session.preview_control_point_edit(EditorSession::ControlPointField::position_x, value); },
        [this] { m_session.begin_control_point_edit(EditorSession::ControlPointField::position_x); },
        [this](float) { m_session.finish_control_point_edit(EditorSession::ControlPointField::position_x); }
    );
    m_panel.add_child<UISlider>(
        Rect{x, first_slider_y + 42.0f, slider_width, 16.0f},
        "Position Y", position_minimum, position_maximum,
        static_cast<float>(current_point->position.y),
        [this](float value) { m_session.preview_control_point_edit(EditorSession::ControlPointField::position_y, value); },
        [this] { m_session.begin_control_point_edit(EditorSession::ControlPointField::position_y); },
        [this](float) { m_session.finish_control_point_edit(EditorSession::ControlPointField::position_y); }
    );
    m_panel.add_child<UISlider>(
        Rect{x, first_slider_y + 84.0f, slider_width, 16.0f},
        "Position Z", position_minimum, position_maximum,
        static_cast<float>(current_point->position.z),
        [this](float value) { m_session.preview_control_point_edit(EditorSession::ControlPointField::position_z, value); },
        [this] { m_session.begin_control_point_edit(EditorSession::ControlPointField::position_z); },
        [this](float) { m_session.finish_control_point_edit(EditorSession::ControlPointField::position_z); }
    );
    m_panel.add_child<UISlider>(
        Rect{x, first_slider_y + 126.0f, slider_width, 16.0f},
        "Weight W", weight_minimum, weight_maximum, static_cast<float>(current_point->weight),
        [this](float value) { m_session.preview_control_point_edit(EditorSession::ControlPointField::weight, value); },
        [this] { m_session.begin_control_point_edit(EditorSession::ControlPointField::weight); },
        [this](float) { m_session.finish_control_point_edit(EditorSession::ControlPointField::weight); }
    );
}
