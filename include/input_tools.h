#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "input_frame.h"
#include "orbit_camera.h"
#include "raylib.h"
#include "scene.h"

class IInputTool {
public:
    virtual ~IInputTool() = default;
    virtual void process_input(
        const InputFrameSnapshot& input,
        OrbitCameraController& camera_controller,
        Scene& scene
    ) = 0;
};

class CameraNavigationTool : public IInputTool {
public:
    void process_input(
        const InputFrameSnapshot& input,
        OrbitCameraController& camera_controller,
        Scene& scene
    ) override {
        if (input.middle_mouse) {
            if (input.modifiers.shift) {
                camera_controller.pan(input.mouse_delta, static_cast<float>(GetScreenHeight()));
            } else {
                camera_controller.orbit(input.mouse_delta);
            }
        }

        if (input.mouse_wheel_delta != 0.0f) {
            camera_controller.zoom(input.mouse_wheel_delta);
        }
    }
};

class InputToolDispatcher {
private:
    std::vector<std::unique_ptr<IInputTool>> m_active_tools;

public:
    template<typename ToolType, typename... Args>
    void register_tools(Args&&... args) {
        m_active_tools.push_back(std::make_unique<ToolType>(std::forward<Args>(args)...));
    }

    void dispatch(
        const InputFrameSnapshot& input,
        OrbitCameraController& camera,
        Scene& scene
    ) {
        for (auto& tool : m_active_tools) {
            tool->process_input(input, camera, scene);
        }
    }
};