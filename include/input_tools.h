#pragma once

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "input_frame.h"
#include "orbit_camera.h"
#include "scene.h"

class IInputTool {
public:
    virtual ~IInputTool() = default;
    virtual void process_input(
        const InputFrameSnapshot& input,
        OrbitCameraController& camera_controller,
        Scene&
    ) = 0;
};

class CameraNavigationTool : public IInputTool {
public:
    void process_input(
        const InputFrameSnapshot& input,
        OrbitCameraController& camera_controller,
        Scene&
    ) override {
        if (input.middle_mouse) {
            if (input.modifiers.shift) {
                camera_controller.pan(input.mouse_delta, static_cast<float>(input.screen_height));
            } else {
                camera_controller.orbit(input.mouse_delta);
            }
        }

        if (input.mouse_wheel_delta != 0.0f) {
            camera_controller.zoom(input.mouse_wheel_delta);
        }
    }
};

class ControlPointSelectionTool final : public IInputTool {
public:
    using SelectionHandler = std::move_only_function<void(
        NurbsSurface& surface,
        size_t u,
        size_t v,
        ControlPoint& point
    )>;
    using ClearHandler = std::move_only_function<void()>;

    explicit ControlPointSelectionTool(
        SelectionHandler on_selection,
        ClearHandler on_clear = {},
        float hit_radius = 12.0f
    );

    void process_input(
        const InputFrameSnapshot& input,
        OrbitCameraController& camera_controller,
        Scene& scene
    ) override;

private:
    SelectionHandler m_on_selection;
    ClearHandler m_on_clear;
    float m_hit_radius{12.0f};
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
