#include "gizmo_controller.h"

bool GizmoController::process_input(
    const InputFrameSnapshot& input,
    const CameraState& camera
) {
    for (const auto& gizmo : m_gizmos) {
        if (gizmo->process_input(input, camera)) {
            return true;
        }
    }
    return false;
}

GizmoDrawList GizmoController::draw_data(
    const CameraState& camera,
    int viewport_height
) const {
    GizmoDrawList result;
    for (const auto& gizmo : m_gizmos) {
        gizmo->append_draw_data(result, camera, viewport_height);
    }
    return result;
}
