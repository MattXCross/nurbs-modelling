#pragma once

#include "gizmo.h"
#include "orbit_camera.h"
#include "entity_id.h"
#include "viewport_display_settings.h"

#include <memory>
#include <optional>
#include <span>

struct ControlPointSelection;
class Scene;

class RaylibViewportRenderer {
public:
    RaylibViewportRenderer();
    ~RaylibViewportRenderer();

    RaylibViewportRenderer(const RaylibViewportRenderer&) = delete;
    RaylibViewportRenderer& operator=(const RaylibViewportRenderer&) = delete;

    void cleanup_gl();

    void render(
        const Scene& scene,
        const CameraState& camera,
        std::span<const ControlPointSelection> selected_points,
        std::optional<EntityId> selected_entity,
        std::optional<EntityId> hovered_entity,
        std::span<const GizmoPrimitive> gizmos,
        const ViewportDisplaySettings& display_settings,
        bool interactive_geometry_edit,
        int framebuffer_width,
        int framebuffer_height
    );

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
