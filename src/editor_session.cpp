#include "editor_session.h"

#include "core.h"
#include "nurbs_surface.h"

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
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
        std::move_only_function<bool(double)> set_value,
        double initial_value,
        double final_value
    )
        : m_set_value(std::move(set_value)),
          m_initial_value(initial_value),
          m_final_value(final_value) {}

    bool undo() override { return m_set_value(m_initial_value); }
    bool redo() override { return m_set_value(m_final_value); }

private:
    std::move_only_function<bool(double)> m_set_value;
    double m_initial_value{0.0};
    double m_final_value{0.0};
};

class CreatedEntityCommand final : public ICommand {
public:
    CreatedEntityCommand(
        Scene& scene,
        EntityId entity,
        std::move_only_function<void(EntityId)> on_removed
    )
        : m_scene(scene),
          m_entity(entity),
          m_on_removed(std::move(on_removed)) {}

    bool undo() override {
        auto removed = m_scene.remove_entity(m_entity);
        if (!removed.has_value()) {
            return false;
        }
        m_removed = std::move(*removed);
        m_on_removed(m_entity);
        return true;
    }

    bool redo() override {
        if (!m_removed.has_value() || !m_scene.restore_entity(*m_removed).has_value()) {
            return false;
        }
        m_removed.reset();
        return true;
    }

private:
    Scene& m_scene;
    EntityId m_entity;
    std::move_only_function<void(EntityId)> m_on_removed;
    std::optional<RemovedSceneNode> m_removed;
};

class DeletedEntityCommand final : public ICommand {
public:
    DeletedEntityCommand(
        Scene& scene,
        RemovedSceneNode removed,
        std::move_only_function<void(EntityId)> on_removed
    )
        : m_scene(scene),
          m_removed(std::move(removed)),
          m_entity(m_removed->entity()),
          m_on_removed(std::move(on_removed)) {}

    bool undo() override {
        if (!m_removed.has_value() || !m_scene.restore_entity(*m_removed).has_value()) {
            return false;
        }
        m_removed.reset();
        return true;
    }

    bool redo() override {
        auto removed = m_scene.remove_entity(m_entity);
        if (!removed.has_value()) {
            return false;
        }
        m_removed = std::move(*removed);
        m_on_removed(m_entity);
        return true;
    }

private:
    Scene& m_scene;
    std::optional<RemovedSceneNode> m_removed;
    EntityId m_entity;
    std::move_only_function<void(EntityId)> m_on_removed;
};

class RenamedEntityCommand final : public ICommand {
public:
    RenamedEntityCommand(
        Scene& scene,
        EntityId entity,
        std::string initial_name,
        std::string final_name
    )
        : m_scene(scene),
          m_entity(entity),
          m_initial_name(std::move(initial_name)),
          m_final_name(std::move(final_name)) {}

    bool undo() override { return m_scene.rename_entity(m_entity, m_initial_name).has_value(); }
    bool redo() override { return m_scene.rename_entity(m_entity, m_final_name).has_value(); }

private:
    Scene& m_scene;
    EntityId m_entity;
    std::string m_initial_name;
    std::string m_final_name;
};

class VisibilityCommand final : public ICommand {
public:
    VisibilityCommand(Scene& scene, EntityId entity, bool initial, bool final)
        : m_scene(scene), m_entity(entity), m_initial(initial), m_final(final) {}

    bool undo() override {
        return m_scene.set_entity_visibility(m_entity, m_initial).has_value();
    }
    bool redo() override {
        return m_scene.set_entity_visibility(m_entity, m_final).has_value();
    }

private:
    Scene& m_scene;
    EntityId m_entity;
    bool m_initial;
    bool m_final;
};

bool selection_references_entity(const Selection& selection, EntityId entity) {
    if (const auto* selected_entity = std::get_if<EntitySelection>(&selection)) {
        return selected_entity->entity == entity;
    }
    if (const auto* selected_point = std::get_if<ControlPointSelection>(&selection)) {
        return selected_point->entity == entity;
    }
    return false;
}

} // namespace

EditorSession::EditorSession()
    : m_camera_controller(
          Vec3{10.0f, 10.0f, 10.0f},
          Vec3{0.0f, 0.0f, 0.0f}
      ) {
    std::vector<ControlPoint> points = {
        {{-3, 0, -4}, 1.0}, {{0, 3, -4}, 1.0}, {{3, -2, -4}, 1.0},
        {{-3, 1,  0}, 1.0}, {{0, 5,  0}, 3.0}, {{3,  1,  0}, 1.0},
        {{-3, 0,  4}, 1.0}, {{0, 2,  4}, 1.0}, {{3,  0,  4}, 1.0}
    };

    auto surface = NurbsSurface::create(3, 3, std::move(points));
    if (!surface.has_value()) {
        throw std::logic_error("Failed to construct the default NURBS surface");
    }
    if (!m_scene.add_entity("WaveSurface", std::move(*surface)).has_value()) {
        throw std::logic_error("Failed to add the default NURBS surface");
    }

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
    if (m_pending_edit.has_value()) {
        return false;
    }
    return m_history.redo();
}

bool EditorSession::can_undo() const {
    return pending_edit_has_preview() || m_history.can_undo();
}

bool EditorSession::can_redo() const {
    return !m_pending_edit.has_value() && m_history.can_redo();
}

std::expected<EntityId, SceneMutationError> EditorSession::create_surface_entity(
    std::string name,
    std::unique_ptr<NurbsSurface> surface
) {
    auto entity = m_scene.add_entity(std::move(name), std::move(surface));
    if (!entity.has_value()) {
        return std::unexpected(entity.error());
    }

    (void)cancel_pending_edit();
    m_history.record_applied(std::make_unique<CreatedEntityCommand>(
        m_scene,
        *entity,
        [this](EntityId removed) { clear_selection_for_entity(removed); }
    ));
    return *entity;
}

std::expected<bool, SceneMutationError> EditorSession::delete_entity(EntityId id) {
    if (m_scene.find_entity(id) == nullptr) {
        return std::unexpected(SceneMutationError::entity_not_found);
    }

    (void)cancel_pending_edit();
    auto removed = m_scene.remove_entity(id);
    if (!removed.has_value()) {
        return std::unexpected(removed.error());
    }

    clear_selection_for_entity(id);
    m_history.record_applied(std::make_unique<DeletedEntityCommand>(
        m_scene,
        std::move(*removed),
        [this](EntityId removed_id) { clear_selection_for_entity(removed_id); }
    ));
    return true;
}

std::expected<bool, SceneMutationError> EditorSession::rename_entity(
    EntityId id,
    std::string name
) {
    const SceneNode* node = m_scene.find_entity(id);
    if (node == nullptr) {
        return std::unexpected(SceneMutationError::entity_not_found);
    }
    if (node->name == name) {
        return false;
    }

    (void)cancel_pending_edit();
    std::string initial_name = node->name;
    auto renamed = m_scene.rename_entity(id, name);
    if (!renamed.has_value()) {
        return std::unexpected(renamed.error());
    }
    m_history.record_applied(std::make_unique<RenamedEntityCommand>(
        m_scene,
        id,
        std::move(initial_name),
        std::move(name)
    ));
    return *renamed;
}

std::expected<bool, SceneMutationError> EditorSession::set_entity_visibility(
    EntityId id,
    bool visible
) {
    const SceneNode* node = m_scene.find_entity(id);
    if (node == nullptr) {
        return std::unexpected(SceneMutationError::entity_not_found);
    }
    if (node->visible == visible) {
        return false;
    }

    (void)cancel_pending_edit();
    const bool initial_visibility = node->visible;
    auto changed = m_scene.set_entity_visibility(id, visible);
    if (!changed.has_value()) {
        return std::unexpected(changed.error());
    }
    m_history.record_applied(std::make_unique<VisibilityCommand>(
        m_scene,
        id,
        initial_visibility,
        visible
    ));
    return *changed;
}

bool EditorSession::begin_control_point_edit(ControlPointField field) {
    (void)cancel_pending_edit();
    const ControlPointSelection* selection = m_selection.control_point();
    const ControlPoint* point = selected_control_point();
    if (selection != nullptr && point != nullptr) {
        m_pending_edit = PendingEdit{*selection, field, field_value(*point, field)};
        return true;
    }
    return false;
}

bool EditorSession::preview_control_point_edit(ControlPointField field, double value) {
    if (!m_pending_edit.has_value() || m_pending_edit->field != field) {
        return false;
    }

    const ControlPointSelection* selection = m_selection.control_point();
    const ControlPoint* point = selected_control_point();
    if (selection == nullptr || point == nullptr ||
        selection->entity != m_pending_edit->selection.entity ||
        selection->u != m_pending_edit->selection.u ||
        selection->v != m_pending_edit->selection.v) {
        return false;
    }

    ControlPoint updated = *point;
    set_field_value(updated, field, value);
    if (!m_scene.set_control_point(*selection, updated).has_value()) {
        (void)cancel_pending_edit();
        return false;
    }
    return true;
}

void EditorSession::finish_control_point_edit(ControlPointField field) {
    if (!m_pending_edit.has_value() || m_pending_edit->field != field) {
        (void)cancel_pending_edit();
        return;
    }

    const PendingEdit edit = *m_pending_edit;
    m_pending_edit.reset();
    const ControlPoint* point = m_scene.resolve(edit.selection);
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
            if (const ControlPoint* selected = scene->resolve(selection)) {
                ControlPoint updated = *selected;
                set_field_value(updated, field, value);
                return scene->set_control_point(selection, updated).has_value();
            }
            return false;
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
    const ControlPoint* point = m_scene.resolve(edit.selection);
    if (point != nullptr && field_value(*point, edit.field) != edit.initial_value) {
        ControlPoint updated = *point;
        set_field_value(updated, edit.field, edit.initial_value);
        const auto restored = m_scene.set_control_point(edit.selection, updated);
        return restored.has_value() && *restored;
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

const ControlPoint* EditorSession::selected_control_point() const {
    const ControlPointSelection* selection = m_selection.control_point();
    return selection == nullptr ? nullptr : m_scene.resolve(*selection);
}

void EditorSession::clear_selection_for_entity(EntityId id) {
    if (selection_references_entity(m_selection.current(), id)) {
        m_selection.clear();
        m_selection_changed = true;
    }
}
