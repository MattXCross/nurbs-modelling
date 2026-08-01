#pragma once

#include "command_history.h"
#include "input_frame.h"
#include "input_tools.h"
#include "orbit_camera.h"
#include "scene.h"
#include "selection.h"

#include <optional>
#include <string>

class EditorSession {
public:
    enum class ControlPointField { position_x, position_y, position_z, weight };

    EditorSession();

    EditorSession(const EditorSession&) = delete;
    EditorSession& operator=(const EditorSession&) = delete;
    EditorSession(EditorSession&&) = delete;
    EditorSession& operator=(EditorSession&&) = delete;

    [[nodiscard]] bool process_viewport_input(const InputFrameSnapshot& input);

    [[nodiscard]] bool select_control_point(ControlPointSelection selection);
    [[nodiscard]] bool clear_selection();

    [[nodiscard]] bool undo();
    [[nodiscard]] bool redo();
    [[nodiscard]] bool can_undo() const;
    [[nodiscard]] bool can_redo() const;

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
    [[nodiscard]] const CameraState& camera() const { return m_camera_controller.camera(); }
    [[nodiscard]] const ControlPoint* selected_control_point() const;

private:
    struct PendingEdit {
        ControlPointSelection selection;
        ControlPointField field;
        double initial_value{0.0};
    };

    [[nodiscard]] bool cancel_pending_edit();
    [[nodiscard]] bool pending_edit_has_preview() const;
    void clear_selection_for_entity(EntityId id);

    Scene m_scene;
    SelectionModel m_selection;
    CommandHistory m_history;
    OrbitCameraController m_camera_controller;
    InputToolDispatcher m_input_dispatcher;
    std::optional<PendingEdit> m_pending_edit;
    bool m_selection_changed{false};
};
