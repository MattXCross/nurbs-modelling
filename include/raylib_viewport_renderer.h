#pragma once

#include "orbit_camera.h"
#include "entity_id.h"

#include <memory>
#include <optional>

struct ControlPoint;
class Scene;

class RaylibViewportRenderer {
public:
    RaylibViewportRenderer();
    ~RaylibViewportRenderer();

    RaylibViewportRenderer(const RaylibViewportRenderer&) = delete;
    RaylibViewportRenderer& operator=(const RaylibViewportRenderer&) = delete;

    void render(
        const Scene& scene,
        const CameraState& camera,
        const ControlPoint* selected_point,
        std::optional<EntityId> selected_entity,
        std::optional<EntityId> hovered_entity,
        int framebuffer_width,
        int framebuffer_height
    );

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
