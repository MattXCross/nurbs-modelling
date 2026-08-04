#pragma once

#include "gizmo.h"

#include <memory>
#include <utility>
#include <vector>

class GizmoController {
public:
    template<typename GizmoType, typename... Args>
    GizmoType& add(Args&&... args) {
        auto gizmo = std::make_unique<GizmoType>(std::forward<Args>(args)...);
        GizmoType& result = *gizmo;
        m_gizmos.push_back(std::move(gizmo));
        return result;
    }

    [[nodiscard]] bool process_input(
        const InputFrameSnapshot& input,
        const CameraState& camera
    );
    [[nodiscard]] GizmoDrawList draw_data(
        const CameraState& camera,
        int viewport_height
    ) const;

private:
    std::vector<std::unique_ptr<IGizmo>> m_gizmos;
};
