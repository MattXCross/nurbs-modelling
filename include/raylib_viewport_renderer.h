#pragma once

#include "orbit_camera.h"

#include <memory>

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
        int framebuffer_width,
        int framebuffer_height
    );

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
