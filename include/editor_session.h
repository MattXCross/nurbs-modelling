#pragma once

#include "command_history.h"
#include "input_frame.h"
#include "input_tools.h"
#include "orbit_camera.h"
#include "scene.h"
#include "selection.h"

#include <optional>

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

    void begin_control_point_edit(ControlPointField field);
    void preview_control_point_edit(ControlPointField field, double value);
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
    [[nodiscard]] ControlPoint* selected_control_point_mutable();

    Scene m_scene;
    SelectionModel m_selection;
    CommandHistory m_history;
    OrbitCameraController m_camera_controller;
    InputToolDispatcher m_input_dispatcher;
    std::optional<PendingEdit> m_pending_edit;
    bool m_selection_changed{false};
};
