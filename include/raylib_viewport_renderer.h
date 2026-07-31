#pragma once

#include "math_types.h"
#include "orbit_camera.h"

#include <memory>

struct ControlPoint;
class Scene;

class RaylibViewportRenderer {
public:
    RaylibViewportRenderer(int width, int height);
    ~RaylibViewportRenderer();

    RaylibViewportRenderer(const RaylibViewportRenderer&) = delete;
    RaylibViewportRenderer& operator=(const RaylibViewportRenderer&) = delete;

    void resize(int width, int height);
    void render(
        const Scene& scene,
        const CameraState& camera,
        const ControlPoint* selected_point
    );
    void composite(Rect destination) const;

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
