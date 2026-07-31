#include "ui/control_point_inspector.h"

#include "ui/ui_widgets.h"

#include <algorithm>
#include <format>
#include <functional>
#include <memory>
#include <utility>

namespace {

class AppliedValueCommand final : public ICommand {
public:
    AppliedValueCommand(
        std::move_only_function<void(double)> set_value,
        double initial_value,
        double final_value
    )
        : m_set_value(std::move(set_value)),
          m_initial_value(initial_value),
          m_final_value(final_value) {}

    void undo() override { m_set_value(m_initial_value); }
    void redo() override { m_set_value(m_final_value); }

private:
    std::move_only_function<void(double)> m_set_value;
    double m_initial_value{0.0};
    double m_final_value{0.0};
};

} // namespace

ControlPointInspectorPanel::ControlPointInspectorPanel(
    Vec2 position,
    Scene& scene,
    SelectionModel& selection,
    CommandHistory& history
)
    : m_panel(Rect{position.x, position.y, 300.0f, 250.0f}, "Control Point"),
      m_scene(scene),
      m_selection(selection),
      m_history(history) {}

void ControlPointInspectorPanel::inspect_point(ControlPointSelection selection) {
    m_pending_edit.reset();
    m_selection.select(selection);
    rebuild_ui();
}

void ControlPointInspectorPanel::clear_selection() {
    m_pending_edit.reset();
    m_selection.clear();
    m_panel.clear_children();
}

void ControlPointInspectorPanel::refresh() {
    rebuild_ui();
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

void ControlPointInspectorPanel::begin_edit(ControlPointField field) {
    m_pending_edit.reset();
    const ControlPointSelection* selection = m_selection.control_point();
    ControlPoint* point = selected_point();
    if (selection == nullptr || point == nullptr) {
        return;
    }

    m_pending_edit = PendingEdit{*selection, field, field_value(*point, field)};
}

void ControlPointInspectorPanel::preview_edit(ControlPointField field, float value) {
    const ControlPointSelection* selection = m_selection.control_point();
    ControlPoint* point = selected_point();
    if (selection != nullptr && point != nullptr) {
        set_field_value(*point, field, value);
        (void)m_scene.mark_geometry_modified(selection->entity);
    }
}

void ControlPointInspectorPanel::finish_edit(ControlPointField field) {
    if (!m_pending_edit.has_value() || m_pending_edit->field != field) {
        m_pending_edit.reset();
        return;
    }

    const PendingEdit edit = *m_pending_edit;
    m_pending_edit.reset();
    ControlPoint* point = m_scene.resolve(edit.selection);
    if (point == nullptr) {
        return;
    }

    const double final_value = field_value(*point, field);
    if (final_value == edit.initial_value) {
        return;
    }

    Scene* scene = &m_scene;
    m_history.record_applied(std::make_unique<AppliedValueCommand>(
        [scene, selection = edit.selection, field](double value) {
            if (ControlPoint* selected = scene->resolve(selection)) {
                set_field_value(*selected, field, value);
                (void)scene->mark_geometry_modified(selection.entity);
            }
        },
        edit.initial_value,
        final_value
    ));
}

double ControlPointInspectorPanel::field_value(
    const ControlPoint& point,
    ControlPointField field
) {
    switch (field) {
        case ControlPointField::position_x: return point.position.x;
        case ControlPointField::position_y: return point.position.y;
        case ControlPointField::position_z: return point.position.z;
        case ControlPointField::weight: return point.weight;
    }
    return 0.0;
}

void ControlPointInspectorPanel::set_field_value(
    ControlPoint& point,
    ControlPointField field,
    double value
) {
    switch (field) {
        case ControlPointField::position_x: point.position.x = value; break;
        case ControlPointField::position_y: point.position.y = value; break;
        case ControlPointField::position_z: point.position.z = value; break;
        case ControlPointField::weight: point.weight = value; break;
    }
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
        [this](float value) { preview_edit(ControlPointField::position_x, value); },
        [this] { begin_edit(ControlPointField::position_x); },
        [this](float) { finish_edit(ControlPointField::position_x); }
    );
    m_panel.add_child<UISlider>(
        Rect{x, first_slider_y + 42.0f, slider_width, 16.0f},
        "Position Y", position_minimum, position_maximum,
        static_cast<float>(current_point->position.y),
        [this](float value) { preview_edit(ControlPointField::position_y, value); },
        [this] { begin_edit(ControlPointField::position_y); },
        [this](float) { finish_edit(ControlPointField::position_y); }
    );
    m_panel.add_child<UISlider>(
        Rect{x, first_slider_y + 84.0f, slider_width, 16.0f},
        "Position Z", position_minimum, position_maximum,
        static_cast<float>(current_point->position.z),
        [this](float value) { preview_edit(ControlPointField::position_z, value); },
        [this] { begin_edit(ControlPointField::position_z); },
        [this](float) { finish_edit(ControlPointField::position_z); }
    );
    m_panel.add_child<UISlider>(
        Rect{x, first_slider_y + 126.0f, slider_width, 16.0f},
        "Weight W", weight_minimum, weight_maximum, static_cast<float>(current_point->weight),
        [this](float value) { preview_edit(ControlPointField::weight, value); },
        [this] { begin_edit(ControlPointField::weight); },
        [this](float) { finish_edit(ControlPointField::weight); }
    );
}
