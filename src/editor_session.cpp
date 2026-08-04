#include "editor_session.h"

#include "core.h"
#include "nurbs_surface.h"

#include <algorithm>
#include <functional>
#include <memory>
#include <ranges>
#include <span>
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

bool apply_field_values(
    Scene& scene,
    const std::vector<ControlPointSelection>& selections,
    EditorSession::ControlPointField field,
    const std::vector<double>& values
) {
    if (selections.size() != values.size()) {
        return false;
    }
    std::vector<ControlPoint> originals;
    originals.reserve(selections.size());
    for (const ControlPointSelection selection : selections) {
        const ControlPoint* point = scene.resolve(selection);
        if (point == nullptr) {
            return false;
        }
        originals.push_back(*point);
    }
    for (std::size_t index = 0; index < selections.size(); ++index) {
        ControlPoint updated = originals[index];
        set_field_value(updated, field, values[index]);
        if (!scene.set_control_point(selections[index], updated).has_value()) {
            for (std::size_t rollback = 0; rollback < index; ++rollback) {
                (void)scene.set_control_point(selections[rollback], originals[rollback]);
            }
            return false;
        }
    }
    return true;
}

class AppliedValuesCommand final : public ICommand {
public:
    AppliedValuesCommand(
        std::string description,
        std::move_only_function<bool(const std::vector<double>&)> set_values,
        std::vector<double> initial_values,
        std::vector<double> final_values
    )
        : m_description(std::move(description)),
          m_set_values(std::move(set_values)),
          m_initial_values(std::move(initial_values)),
          m_final_values(std::move(final_values)) {}

    [[nodiscard]] std::string_view description() const override { return m_description; }
    bool undo() override { return m_set_values(m_initial_values); }
    bool redo() override { return m_set_values(m_final_values); }

private:
    std::string m_description;
    std::move_only_function<bool(const std::vector<double>&)> m_set_values;
    std::vector<double> m_initial_values;
    std::vector<double> m_final_values;
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

EditorSession::SelectionOperation selection_operation(ModifierKeys modifiers) {
    if (modifiers.ctrl) {
        return EditorSession::SelectionOperation::toggle;
    }
    return modifiers.shift ? EditorSession::SelectionOperation::add :
        EditorSession::SelectionOperation::replace;
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
    m_input_dispatcher.register_tools<SurfaceSelectionTool>(
        [this] { return m_selection_mode == SelectionMode::object; },
        [this](EntitySelection selection) { (void)select_entity(selection); },
        [this](std::optional<EntityId> entity) { set_hovered_entity(entity); },
        [this] { (void)clear_selection(); }
    );
    m_input_dispatcher.register_tools<ControlPointSelectionTool>(
        [this] { return m_selection_mode == SelectionMode::control_point; },
        [this](ControlPointSelection selection, ModifierKeys modifiers) {
            (void)select_control_point(selection, selection_operation(modifiers));
        },
        [this](std::vector<ControlPointSelection> selections, ModifierKeys modifiers) {
            (void)select_control_points(std::move(selections), selection_operation(modifiers));
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

bool EditorSession::select_control_point(
    ControlPointSelection selection,
    SelectionOperation operation
) {
    return select_control_points({selection}, operation);
}

bool EditorSession::select_control_points(
    std::vector<ControlPointSelection> selections,
    SelectionOperation operation
) {
    NotificationBatch notifications(*this);
    if (m_selection_mode != SelectionMode::control_point ||
        std::ranges::any_of(selections, [this](ControlPointSelection selection) {
            return m_scene.resolve(selection) == nullptr;
        })) {
        return false;
    }

    (void)cancel_pending_edit();
    const Selection previous = m_selection.current();
    if (operation == SelectionOperation::replace) {
        m_selection.select(std::move(selections));
    } else {
        for (const ControlPointSelection selection : selections) {
            if (operation == SelectionOperation::add) {
                (void)m_selection.add(selection);
            } else {
                (void)m_selection.toggle(selection);
            }
        }
    }
    if (m_selection.current() == previous) {
        return false;
    }
    notify(EditorChange{.selection = true});
    return true;
}

bool EditorSession::select_all_control_points() {
    const std::optional<EntityId> entity = selected_entity_id();
    const SceneNode* node = entity ? m_scene.find_entity(*entity) : nullptr;
    if (m_selection_mode != SelectionMode::control_point || node == nullptr ||
        node->surface == nullptr) {
        return false;
    }
    std::vector<ControlPointSelection> points;
    points.reserve(node->surface->u_count() * node->surface->v_count());
    for (std::size_t u = 0; u < node->surface->u_count(); ++u) {
        for (std::size_t v = 0; v < node->surface->v_count(); ++v) {
            points.push_back({*entity, u, v});
        }
    }
    return select_control_points(std::move(points));
}

bool EditorSession::select_control_point_row() {
    const ControlPointSelection* primary = m_selection.control_point();
    const SceneNode* node = primary == nullptr ? nullptr : m_scene.find_entity(primary->entity);
    if (node == nullptr || node->surface == nullptr) {
        return false;
    }
    std::vector<ControlPointSelection> points;
    points.reserve(node->surface->v_count());
    for (std::size_t v = 0; v < node->surface->v_count(); ++v) {
        points.push_back({primary->entity, primary->u, v});
    }
    return select_control_points(std::move(points));
}

bool EditorSession::select_control_point_column() {
    const ControlPointSelection* primary = m_selection.control_point();
    const SceneNode* node = primary == nullptr ? nullptr : m_scene.find_entity(primary->entity);
    if (node == nullptr || node->surface == nullptr) {
        return false;
    }
    std::vector<ControlPointSelection> points;
    points.reserve(node->surface->u_count());
    for (std::size_t u = 0; u < node->surface->u_count(); ++u) {
        points.push_back({primary->entity, u, primary->v});
    }
    return select_control_points(std::move(points));
}

bool EditorSession::grow_control_point_selection() {
    const std::span selected = m_selection.control_points();
    if (m_selection_mode != SelectionMode::control_point || selected.empty()) {
        return false;
    }
    std::vector<ControlPointSelection> grown{selected.begin(), selected.end()};
    for (const ControlPointSelection point : selected) {
        const SceneNode* node = m_scene.find_entity(point.entity);
        if (node == nullptr || node->surface == nullptr) {
            continue;
        }
        if (point.u > 0) {
            grown.push_back({point.entity, point.u - 1, point.v});
        }
        if (point.u + 1 < node->surface->u_count()) {
            grown.push_back({point.entity, point.u + 1, point.v});
        }
        if (point.v > 0) {
            grown.push_back({point.entity, point.u, point.v - 1});
        }
        if (point.v + 1 < node->surface->v_count()) {
            grown.push_back({point.entity, point.u, point.v + 1});
        }
    }
    return select_control_points(std::move(grown));
}

bool EditorSession::shrink_control_point_selection() {
    const std::span selected = m_selection.control_points();
    if (m_selection_mode != SelectionMode::control_point || selected.empty()) {
        return false;
    }
    const auto contains = [selected](ControlPointSelection candidate) {
        return std::ranges::find(selected, candidate) != selected.end();
    };
    std::vector<ControlPointSelection> shrunk;
    for (const ControlPointSelection point : selected) {
        const SceneNode* node = m_scene.find_entity(point.entity);
        if (node == nullptr || node->surface == nullptr || point.u == 0 || point.v == 0 ||
            point.u + 1 >= node->surface->u_count() ||
            point.v + 1 >= node->surface->v_count()) {
            continue;
        }
        if (contains({point.entity, point.u - 1, point.v}) &&
            contains({point.entity, point.u + 1, point.v}) &&
            contains({point.entity, point.u, point.v - 1}) &&
            contains({point.entity, point.u, point.v + 1})) {
            shrunk.push_back(point);
        }
    }
    return select_control_points(std::move(shrunk));
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
    if (mode == SelectionMode::control_point) {
        set_hovered_entity(std::nullopt);
    }
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
    m_hovered_entity.reset();
    m_history.clear();
    m_scene = std::move(scene);
    notify(EditorChange{
        .selection = true,
        .entities = true,
        .geometry = true,
        .properties = true,
        .history = true,
        .hover = true
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
    if (m_hovered_entity == id) {
        set_hovered_entity(std::nullopt);
    }
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
    if (!visible && m_hovered_entity == id) {
        set_hovered_entity(std::nullopt);
    }
    notify(EditorChange{.entities = true, .geometry = true, .history = true});
    return *changed;
}

bool EditorSession::begin_control_point_edit(ControlPointField field) {
    NotificationBatch notifications(*this);
    (void)cancel_pending_edit();
    const std::span selections = m_selection.control_points();
    if (selections.empty()) {
        return false;
    }
    PendingEdit edit{{selections.begin(), selections.end()}, field, {}};
    edit.initial_values.reserve(selections.size());
    for (const ControlPointSelection selection : selections) {
        const ControlPoint* point = m_scene.resolve(selection);
        if (point == nullptr) {
            return false;
        }
        edit.initial_values.push_back(field_value(*point, field));
    }
    m_pending_edit = std::move(edit);
    notify(EditorChange{.history = true});
    return true;
}

bool EditorSession::preview_control_point_edit(ControlPointField field, double value) {
    NotificationBatch notifications(*this);
    if (!m_pending_edit.has_value() || m_pending_edit->field != field) {
        return false;
    }

    const std::span selections = m_selection.control_points();
    if (!std::ranges::equal(selections, m_pending_edit->selections)) {
        return false;
    }
    const std::vector<double> values(selections.size(), value);
    if (!apply_field_values(m_scene, m_pending_edit->selections, field, values)) {
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
    std::vector<double> final_values;
    final_values.reserve(edit.selections.size());
    for (const ControlPointSelection selection : edit.selections) {
        const ControlPoint* point = m_scene.resolve(selection);
        if (point == nullptr) {
            notify(EditorChange{.history = true});
            return;
        }
        final_values.push_back(field_value(*point, field));
    }
    if (final_values == edit.initial_values) {
        notify(EditorChange{.history = true});
        return;
    }

    Scene* scene = &m_scene;
    m_history.record_applied(std::make_unique<AppliedValuesCommand>(
        std::string(field_description(field)),
        [scene, selections = edit.selections, field](const std::vector<double>& values) {
            return apply_field_values(*scene, selections, field, values);
        },
        edit.initial_values,
        std::move(final_values)
    ));
    notify(EditorChange{.history = true});
}

bool EditorSession::cancel_pending_edit() {
    if (!m_pending_edit.has_value()) {
        return false;
    }

    const PendingEdit edit = *m_pending_edit;
    m_pending_edit.reset();
    std::vector<double> current_values;
    current_values.reserve(edit.selections.size());
    for (const ControlPointSelection selection : edit.selections) {
        const ControlPoint* point = m_scene.resolve(selection);
        if (point == nullptr) {
            notify(EditorChange{.history = true});
            return false;
        }
        current_values.push_back(field_value(*point, edit.field));
    }
    if (current_values != edit.initial_values) {
        const bool changed = apply_field_values(
            m_scene,
            edit.selections,
            edit.field,
            edit.initial_values
        );
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
    for (std::size_t index = 0; index < m_pending_edit->selections.size(); ++index) {
        const ControlPoint* point = m_scene.resolve(m_pending_edit->selections[index]);
        if (point == nullptr || field_value(*point, m_pending_edit->field) !=
            m_pending_edit->initial_values[index]) {
            return true;
        }
    }
    return false;
}

const ControlPoint* EditorSession::selected_control_point() const {
    const ControlPointSelection* selection = m_selection.control_point();
    return selection == nullptr ? nullptr : m_scene.resolve(*selection);
}

std::optional<EntityId> EditorSession::selected_entity_id() const {
    if (const EntitySelection* entity = m_selection.entity()) {
        return entity->entity;
    }
    if (const ControlPointSelection* point = m_selection.control_point()) {
        return point->entity;
    }
    return std::nullopt;
}

void EditorSession::set_hovered_entity(std::optional<EntityId> entity) {
    if (entity.has_value()) {
        const SceneNode* node = m_scene.find_entity(*entity);
        if (node == nullptr || !node->visible) {
            entity.reset();
        }
    }
    if (m_hovered_entity == entity) {
        return;
    }
    m_hovered_entity = entity;
    notify(EditorChange{.hover = true});
}

void EditorSession::clear_selection_for_entity(EntityId id) {
    if (m_selection.remove_entity(id)) {
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
