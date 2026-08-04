#pragma once

#include "command_history.h"
#include "editor_change.h"
#include "input_frame.h"
#include "input_tools.h"
#include "orbit_camera.h"
#include "scene.h"
#include "selection.h"
#include "translation.h"
#include "transform_controller.h"

#include <functional>
#include <optional>
#include <string>

class EditorSession {
public:
    using ChangeHandler = std::move_only_function<void(EditorChange)>;

    enum class ControlPointField { position_x, position_y, position_z, weight };
    enum class SelectionMode { object, control_point };
    enum class SelectionOperation { replace, add, toggle };
    using TranslationConstraint = ::TranslationConstraint;
    using TransformMode = ::TransformMode;
    using PivotMode = ::PivotMode;
    using RotationConstraint = ::RotationConstraint;
    using ScaleConstraint = ::ScaleConstraint;
    using TransformOrientation = ::TransformOrientation;

    EditorSession();

    EditorSession(const EditorSession&) = delete;
    EditorSession& operator=(const EditorSession&) = delete;
    EditorSession(EditorSession&&) = delete;
    EditorSession& operator=(EditorSession&&) = delete;

    void process_viewport_input(const InputFrameSnapshot& input);
    void set_change_handler(ChangeHandler handler);

    [[nodiscard]] bool select_entity(EntitySelection selection);
    [[nodiscard]] bool select_control_point(
        ControlPointSelection selection,
        SelectionOperation operation = SelectionOperation::replace
    );
    [[nodiscard]] bool select_control_points(
        std::vector<ControlPointSelection> selections,
        SelectionOperation operation = SelectionOperation::replace
    );
    [[nodiscard]] bool select_all_control_points();
    [[nodiscard]] bool select_control_point_row();
    [[nodiscard]] bool select_control_point_column();
    [[nodiscard]] bool grow_control_point_selection();
    [[nodiscard]] bool shrink_control_point_selection();
    [[nodiscard]] bool begin_translation(TranslationConstraint constraint);
    [[nodiscard]] bool preview_translation(cad::Vector3 delta);
    [[nodiscard]] bool finish_translation();
    [[nodiscard]] bool cancel_translation();
    [[nodiscard]] bool translate_selection(cad::Vector3 delta);
    [[nodiscard]] bool begin_rotation(RotationConstraint constraint, cad::Vector3 axis);
    [[nodiscard]] bool preview_rotation(double angle_radians);
    [[nodiscard]] bool finish_rotation();
    [[nodiscard]] bool cancel_rotation();
    [[nodiscard]] bool rotate_selection(cad::Vector3 axis, double angle_radians);
    [[nodiscard]] bool begin_scale(ScaleConstraint constraint);
    [[nodiscard]] bool preview_scale(double factor);
    [[nodiscard]] bool finish_scale();
    [[nodiscard]] bool cancel_scale();
    [[nodiscard]] bool scale_selection(ScaleConstraint constraint, double factor);
    [[nodiscard]] bool set_transform_mode(TransformMode mode);
    [[nodiscard]] bool set_pivot_mode(PivotMode mode);
    [[nodiscard]] bool set_transform_orientation(TransformOrientation orientation);
    [[nodiscard]] bool clear_selection();
    [[nodiscard]] bool set_selection_mode(SelectionMode mode);
    [[nodiscard]] bool fit_all(int viewport_width, int viewport_height);
    [[nodiscard]] bool frame_selection(int viewport_width, int viewport_height);
    void set_standard_view(StandardView view);
    [[nodiscard]] bool set_camera_projection(ProjectionMode projection);
    [[nodiscard]] cad::Aabb3 visible_bounds() const;
    [[nodiscard]] cad::Aabb3 selected_bounds() const;

    [[nodiscard]] bool undo();
    [[nodiscard]] bool redo();
    [[nodiscard]] bool can_undo() const;
    [[nodiscard]] bool can_redo() const;
    [[nodiscard]] std::string undo_description() const;
    [[nodiscard]] std::string redo_description() const;
    [[nodiscard]] bool is_dirty() const;
    [[nodiscard]] bool translation_active() const { return m_transform.translation_active(); }
    [[nodiscard]] bool rotation_active() const { return m_transform.rotation_active(); }
    [[nodiscard]] bool scale_active() const { return m_transform.scale_active(); }
    [[nodiscard]] bool transform_active() const { return m_transform.active(); }
    [[nodiscard]] TransformMode transform_mode() const { return m_transform.mode(); }
    [[nodiscard]] PivotMode pivot_mode() const { return m_transform.pivot_mode(); }
    [[nodiscard]] TransformOrientation transform_orientation() const {
        return m_transform.orientation();
    }
    [[nodiscard]] std::optional<TransformFrame> transform_frame() const {
        return m_transform.frame();
    }
    [[nodiscard]] std::optional<TranslationConstraint> active_translation_constraint() const {
        return m_transform.translation_constraint();
    }
    [[nodiscard]] std::optional<RotationConstraint> active_rotation_constraint() const {
        return m_transform.rotation_constraint();
    }
    [[nodiscard]] std::optional<ScaleConstraint> active_scale_constraint() const {
        return m_transform.scale_constraint();
    }
    [[nodiscard]] std::optional<cad::Point3> selection_pivot() const;
    void mark_saved();
    void commit_pending_edit();
    void replace_document(Scene scene);

    [[nodiscard]] std::expected<EntityId, SceneMutationError> create_surface_entity(
        std::string name,
        std::unique_ptr<NurbsSurface> surface
    );
    [[nodiscard]] std::expected<bool, SceneMutationError> delete_entity(EntityId id);
    [[nodiscard]] std::expected<bool, SceneMutationError> rename_entity(
        EntityId id,
        std::string name
    );
    [[nodiscard]] std::expected<bool, SceneMutationError> set_entity_visibility(
        EntityId id,
        bool visible
    );

    [[nodiscard]] bool begin_control_point_edit(ControlPointField field);
    [[nodiscard]] bool preview_control_point_edit(ControlPointField field, double value);
    void finish_control_point_edit(ControlPointField field);

    [[nodiscard]] const Scene& scene() const { return m_scene; }
    [[nodiscard]] const SelectionModel& selection() const { return m_selection; }
    [[nodiscard]] SelectionMode selection_mode() const { return m_selection_mode; }
    [[nodiscard]] std::optional<EntityId> selected_entity_id() const;
    [[nodiscard]] std::optional<EntityId> hovered_entity_id() const { return m_hovered_entity; }
    [[nodiscard]] const CameraState& camera() const { return m_camera_controller.camera(); }
    [[nodiscard]] const ControlPoint* selected_control_point() const;

private:
    class NotificationBatch {
    public:
        explicit NotificationBatch(EditorSession& session);
        ~NotificationBatch();

        NotificationBatch(const NotificationBatch&) = delete;
        NotificationBatch& operator=(const NotificationBatch&) = delete;

    private:
        EditorSession& m_session;
    };

    struct PendingEdit {
        std::vector<ControlPointSelection> selections;
        ControlPointField field;
        std::vector<double> initial_values;
    };

    [[nodiscard]] bool cancel_pending_edit();
    [[nodiscard]] bool pending_edit_has_preview() const;
    [[nodiscard]] bool cancel_active_transform();
    void set_hovered_entity(std::optional<EntityId> entity);
    void clear_selection_for_entity(EntityId id);
    void notify(EditorChange change);
    void flush_notifications();

    Scene m_scene;
    SelectionModel m_selection;
    SelectionMode m_selection_mode{SelectionMode::object};
    std::optional<EntityId> m_hovered_entity;
    CommandHistory m_history;
    TransformController m_transform;
    OrbitCameraController m_camera_controller;
    InputToolDispatcher m_input_dispatcher;
    ChangeHandler m_change_handler;
    EditorChange m_pending_change;
    std::optional<PendingEdit> m_pending_edit;
    std::size_t m_notification_depth{0};
    bool m_notifying{false};
};
