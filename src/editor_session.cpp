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

std::string_view field_description(EditorSession::ControlPointField field) {
    switch (field) {
        case EditorSession::ControlPointField::position_x: return "Edit X Position";
        case EditorSession::ControlPointField::position_y: return "Edit Y Position";
        case EditorSession::ControlPointField::position_z: return "Edit Z Position";
        case EditorSession::ControlPointField::weight: return "Edit Weight";
    }
    return "Edit Control Point";
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
        std::string description,
        std::move_only_function<bool(double)> set_value,
        double initial_value,
        double final_value
    )
        : m_description(std::move(description)),
          m_set_value(std::move(set_value)),
          m_initial_value(initial_value),
          m_final_value(final_value) {}

    [[nodiscard]] std::string_view description() const override { return m_description; }
    bool undo() override { return m_set_value(m_initial_value); }
    bool redo() override { return m_set_value(m_final_value); }

private:
    std::string m_description;
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

    [[nodiscard]] std::string_view description() const override { return "Create Surface"; }

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

    [[nodiscard]] std::string_view description() const override { return "Delete Surface"; }

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

    [[nodiscard]] std::string_view description() const override { return "Rename Surface"; }

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

    [[nodiscard]] std::string_view description() const override {
        return m_final ? "Show Surface" : "Hide Surface";
    }

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

EditorSession::NotificationBatch::NotificationBatch(EditorSession& session)
    : m_session(session) {
    ++m_session.m_notification_depth;
}

EditorSession::NotificationBatch::~NotificationBatch() {
    --m_session.m_notification_depth;
    m_session.flush_notifications();
}

EditorSession::EditorSession()
    : m_camera_controller(
          cad::Point3{10.0, 10.0, 10.0},
          cad::Point3{0.0, 0.0, 0.0}
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
            if (m_selection_mode == SelectionMode::object) {
                (void)select_entity(EntitySelection{selection.entity});
            } else {
                (void)select_control_point(selection);
            }
        },
        [this] {
            (void)clear_selection();
        }
    );
}

void EditorSession::process_viewport_input(const InputFrameSnapshot& input) {
    NotificationBatch notifications(*this);
    m_input_dispatcher.dispatch(input, m_camera_controller, m_scene);
}

void EditorSession::set_change_handler(ChangeHandler handler) {
    m_change_handler = std::move(handler);
}

bool EditorSession::select_entity(EntitySelection selection) {
    NotificationBatch notifications(*this);
    if (m_scene.find_entity(selection.entity) == nullptr) {
        return false;
    }

    const EntitySelection* current = m_selection.entity();
    if (current != nullptr && current->entity == selection.entity) {
        return false;
    }

    (void)cancel_pending_edit();
    m_selection.select(selection);
    notify(EditorChange{.selection = true});
    return true;
}

bool EditorSession::select_control_point(ControlPointSelection selection) {
    NotificationBatch notifications(*this);
    if (m_selection_mode != SelectionMode::control_point ||
        m_scene.resolve(selection) == nullptr) {
        return false;
    }

    const ControlPointSelection* current = m_selection.control_point();
    if (current != nullptr && current->entity == selection.entity &&
        current->u == selection.u && current->v == selection.v) {
        return false;
    }

    (void)cancel_pending_edit();
    m_selection.select(selection);
    notify(EditorChange{.selection = true});
    return true;
}

bool EditorSession::clear_selection() {
    NotificationBatch notifications(*this);
    if (m_selection.empty()) {
        return false;
    }

    (void)cancel_pending_edit();
    m_selection.clear();
    notify(EditorChange{.selection = true});
    return true;
}

bool EditorSession::set_selection_mode(SelectionMode mode) {
    NotificationBatch notifications(*this);
    if (m_selection_mode == mode) {
        return false;
    }

    (void)cancel_pending_edit();
    m_selection_mode = mode;
    bool selection_changed = false;
    if (mode == SelectionMode::object) {
        if (const ControlPointSelection* point = m_selection.control_point()) {
            m_selection.select(EntitySelection{point->entity});
            selection_changed = true;
        }
    }
    notify(EditorChange{
        .selection = selection_changed,
        .interaction_mode = true
    });
    return true;
}

bool EditorSession::undo() {
    NotificationBatch notifications(*this);
    if (cancel_pending_edit()) {
        return true;
    }
    if (!m_history.undo()) {
        return false;
    }
    notify(EditorChange{
        .selection = true,
        .entities = true,
        .geometry = true,
        .properties = true,
        .history = true
    });
    return true;
}

bool EditorSession::redo() {
    NotificationBatch notifications(*this);
    if (m_pending_edit.has_value()) {
        return false;
    }
    if (!m_history.redo()) {
        return false;
    }
    notify(EditorChange{
        .selection = true,
        .entities = true,
        .geometry = true,
        .properties = true,
        .history = true
    });
    return true;
}

bool EditorSession::can_undo() const {
    return pending_edit_has_preview() || m_history.can_undo();
}

bool EditorSession::can_redo() const {
    return !m_pending_edit.has_value() && m_history.can_redo();
}

std::string EditorSession::undo_description() const {
    if (pending_edit_has_preview()) {
        return std::string(field_description(m_pending_edit->field));
    }
    return std::string(m_history.undo_description());
}

std::string EditorSession::redo_description() const {
    return std::string(m_history.redo_description());
}

bool EditorSession::is_dirty() const {
    return pending_edit_has_preview() || m_history.is_dirty();
}

void EditorSession::mark_saved() {
    if (!m_pending_edit.has_value()) {
        m_history.mark_saved();
        notify(EditorChange{.history = true});
    }
}

void EditorSession::commit_pending_edit() {
    if (m_pending_edit.has_value()) {
        finish_control_point_edit(m_pending_edit->field);
    }
}

void EditorSession::replace_document(Scene scene) {
    NotificationBatch notifications(*this);
    m_pending_edit.reset();
    m_selection.clear();
    m_history.clear();
    m_scene = std::move(scene);
    notify(EditorChange{
        .selection = true,
        .entities = true,
        .geometry = true,
        .properties = true,
        .history = true
    });
}

std::expected<EntityId, SceneMutationError> EditorSession::create_surface_entity(
    std::string name,
    std::unique_ptr<NurbsSurface> surface
) {
    NotificationBatch notifications(*this);
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
    notify(EditorChange{.entities = true, .geometry = true, .history = true});
    return *entity;
}

std::expected<bool, SceneMutationError> EditorSession::delete_entity(EntityId id) {
    NotificationBatch notifications(*this);
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
    notify(EditorChange{.entities = true, .geometry = true, .history = true});
    return true;
}

std::expected<bool, SceneMutationError> EditorSession::rename_entity(
    EntityId id,
    std::string name
) {
    NotificationBatch notifications(*this);
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
    notify(EditorChange{.entities = true, .history = true});
    return *renamed;
}

std::expected<bool, SceneMutationError> EditorSession::set_entity_visibility(
    EntityId id,
    bool visible
) {
    NotificationBatch notifications(*this);
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
    notify(EditorChange{.entities = true, .geometry = true, .history = true});
    return *changed;
}

bool EditorSession::begin_control_point_edit(ControlPointField field) {
    NotificationBatch notifications(*this);
    (void)cancel_pending_edit();
    const ControlPointSelection* selection = m_selection.control_point();
    const ControlPoint* point = selected_control_point();
    if (selection != nullptr && point != nullptr) {
        m_pending_edit = PendingEdit{*selection, field, field_value(*point, field)};
        notify(EditorChange{.history = true});
        return true;
    }
    return false;
}

bool EditorSession::preview_control_point_edit(ControlPointField field, double value) {
    NotificationBatch notifications(*this);
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
    notify(EditorChange{.geometry = true, .history = true});
    return true;
}

void EditorSession::finish_control_point_edit(ControlPointField field) {
    NotificationBatch notifications(*this);
    if (!m_pending_edit.has_value() || m_pending_edit->field != field) {
        (void)cancel_pending_edit();
        return;
    }

    const PendingEdit edit = *m_pending_edit;
    m_pending_edit.reset();
    const ControlPoint* point = m_scene.resolve(edit.selection);
    if (point == nullptr) {
        notify(EditorChange{.history = true});
        return;
    }

    const double final_value = field_value(*point, field);
    if (final_value == edit.initial_value) {
        notify(EditorChange{.history = true});
        return;
    }

    Scene* scene = &m_scene;
    m_history.record_applied(std::make_unique<AppliedValueCommand>(
        std::string(field_description(field)),
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
    notify(EditorChange{.history = true});
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
        const bool changed = restored.has_value() && *restored;
        notify(EditorChange{
            .geometry = changed,
            .properties = changed,
            .history = true
        });
        return changed;
    }
    notify(EditorChange{.history = true});
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
        notify(EditorChange{.selection = true});
    }
}

void EditorSession::notify(EditorChange change) {
    m_pending_change.merge(change);
    flush_notifications();
}

void EditorSession::flush_notifications() {
    if (m_notification_depth != 0 || m_notifying || !m_change_handler) {
        return;
    }

    m_notifying = true;
    while (!m_pending_change.empty() && m_change_handler) {
        const EditorChange change = m_pending_change;
        m_pending_change = {};
        m_change_handler(change);
    }
    m_notifying = false;
}
