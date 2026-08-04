#pragma once

#include "orbit_camera.h"
#include "entity_id.h"
#include "translation.h"

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

    void render(
        const Scene& scene,
        const CameraState& camera,
        std::span<const ControlPointSelection> selected_points,
        std::optional<EntityId> selected_entity,
        std::optional<EntityId> hovered_entity,
        std::optional<TransformFrame> transform_frame,
        TransformMode transform_mode,
        std::optional<TranslationConstraint> active_translation_constraint,
        std::optional<RotationConstraint> active_rotation_constraint,
        std::optional<ScaleConstraint> active_scale_constraint,
        int framebuffer_width,
        int framebuffer_height
    );

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
