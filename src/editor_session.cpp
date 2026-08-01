#include "editor_session.h"

#include "core.h"
#include "nurbs_surface.h"

#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace {

double field_value(
    const ControlPoint& point,
    EditorSession::ControlPointField field
) {
    switch (field) {
        case EditorSession::ControlPointField::position_x: return point.position.x;
        case EditorSession::ControlPointField::position_y: return point.position.y;
        case EditorSession::ControlPointField::position_z: return point.position.z;
        case EditorSession::ControlPointField::weight: return point.weight;
    }
    return 0.0;
}

void set_field_value(
    ControlPoint& point,
    EditorSession::ControlPointField field,
    double value
) {
    switch (field) {
        case EditorSession::ControlPointField::position_x: point.position.x = value; break;
        case EditorSession::ControlPointField::position_y: point.position.y = value; break;
        case EditorSession::ControlPointField::position_z: point.position.z = value; break;
        case EditorSession::ControlPointField::weight: point.weight = value; break;
    }
}

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

EditorSession::EditorSession()
    : m_camera_controller(
          Vec3{10.0f, 10.0f, 10.0f},
          Vec3{0.0f, 0.0f, 0.0f}
      ) {
    std::vector<ControlPoint> points = {
        {{-4, 0, -4}, 1.0}, {{-1, 3, -4}, 1.0}, {{2, -2, -4}, 1.0},
        {{-4, 1,  0}, 1.0}, {{-1, 5,  0}, 3.0}, {{2,  1,  0}, 1.0},
        {{-4, 0,  4}, 1.0}, {{-1, 2,  4}, 1.0}, {{2,  0,  4}, 1.0}
    };

    auto surface = std::make_unique<NurbsSurface>(3, 3, std::move(points));
    surface->translate(Point3D{1.0, 0.0, 0.0});
    (void)m_scene.add_entity("WaveSurface", std::move(surface));

    m_input_dispatcher.register_tools<CameraNavigationTool>();
    m_input_dispatcher.register_tools<ControlPointSelectionTool>(
        [this](ControlPointSelection selection) {
            (void)select_control_point(selection);
        },
        [this] {
            (void)clear_selection();
        }
    );
}

bool EditorSession::process_viewport_input(const InputFrameSnapshot& input) {
    m_selection_changed = false;
    m_input_dispatcher.dispatch(input, m_camera_controller, m_scene);
    return m_selection_changed;
}

bool EditorSession::select_control_point(ControlPointSelection selection) {
    if (m_scene.resolve(selection) == nullptr) {
        return false;
    }

    const ControlPointSelection* current = m_selection.control_point();
    if (current != nullptr && current->entity == selection.entity &&
        current->u == selection.u && current->v == selection.v) {
        return false;
    }

    (void)cancel_pending_edit();
    m_selection.select(selection);
    m_selection_changed = true;
    return true;
}

bool EditorSession::clear_selection() {
    if (m_selection.empty()) {
        return false;
    }

    (void)cancel_pending_edit();
    m_selection.clear();
    m_selection_changed = true;
    return true;
}

bool EditorSession::undo() {
    if (cancel_pending_edit()) {
        return true;
    }
    return m_history.undo();
}

bool EditorSession::redo() {
    if (cancel_pending_edit()) {
        return true;
    }
    return m_history.redo();
}

bool EditorSession::can_undo() const {
    return pending_edit_has_preview() || m_history.can_undo();
}

bool EditorSession::can_redo() const {
    return pending_edit_has_preview() || m_history.can_redo();
}

void EditorSession::begin_control_point_edit(ControlPointField field) {
    (void)cancel_pending_edit();
    const ControlPointSelection* selection = m_selection.control_point();
    ControlPoint* point = selected_control_point_mutable();
    if (selection != nullptr && point != nullptr) {
        m_pending_edit = PendingEdit{*selection, field, field_value(*point, field)};
    }
}

void EditorSession::preview_control_point_edit(ControlPointField field, double value) {
    if (!m_pending_edit.has_value() || m_pending_edit->field != field) {
        return;
    }

    const ControlPointSelection* selection = m_selection.control_point();
    ControlPoint* point = selected_control_point_mutable();
    if (selection == nullptr || point == nullptr ||
        selection->entity != m_pending_edit->selection.entity ||
        selection->u != m_pending_edit->selection.u ||
        selection->v != m_pending_edit->selection.v) {
        return;
    }

    set_field_value(*point, field, value);
    (void)m_scene.mark_geometry_modified(selection->entity);
}

void EditorSession::finish_control_point_edit(ControlPointField field) {
    if (!m_pending_edit.has_value() || m_pending_edit->field != field) {
        (void)cancel_pending_edit();
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

bool EditorSession::cancel_pending_edit() {
    if (!m_pending_edit.has_value()) {
        return false;
    }

    const PendingEdit edit = *m_pending_edit;
    m_pending_edit.reset();
    ControlPoint* point = m_scene.resolve(edit.selection);
    if (point != nullptr && field_value(*point, edit.field) != edit.initial_value) {
        set_field_value(*point, edit.field, edit.initial_value);
        (void)m_scene.mark_geometry_modified(edit.selection.entity);
        return true;
    }
    return false;
}

bool EditorSession::pending_edit_has_preview() const {
    if (!m_pending_edit.has_value()) {
        return false;
    }

    const ControlPoint* point = m_scene.resolve(m_pending_edit->selection);
    return point != nullptr &&
        field_value(*point, m_pending_edit->field) != m_pending_edit->initial_value;
}

ControlPoint* EditorSession::selected_control_point_mutable() {
    const ControlPointSelection* selection = m_selection.control_point();
    return selection == nullptr ? nullptr : m_scene.resolve(*selection);
}

const ControlPoint* EditorSession::selected_control_point() const {
    const ControlPointSelection* selection = m_selection.control_point();
    return selection == nullptr ? nullptr : m_scene.resolve(*selection);
}
