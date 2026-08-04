#pragma once

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "input_frame.h"
#include "orbit_camera.h"
#include "scene.h"
#include "selection.h"
#include "surface_picking.h"
#include "translation.h"

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

class TranslationTool final : public IInputTool {
public:
    using ActiveHandler = std::move_only_function<bool()>;
    using PivotHandler = std::move_only_function<std::optional<TransformFrame>()>;
    using BeginHandler = std::move_only_function<bool(TranslationConstraint)>;
    using PreviewHandler = std::move_only_function<bool(cad::Vector3)>;
    using FinishHandler = std::move_only_function<void()>;

    TranslationTool(
        ActiveHandler active,
        PivotHandler pivot,
        BeginHandler begin,
        PreviewHandler preview,
        FinishHandler finish,
        FinishHandler cancel,
        double snap_increment = 0.5
    );

    void process_input(
        const InputFrameSnapshot& input,
        OrbitCameraController& camera_controller,
        Scene& scene
    ) override;

private:
    ActiveHandler m_active;
    PivotHandler m_pivot;
    BeginHandler m_begin;
    PreviewHandler m_preview;
    FinishHandler m_finish;
    FinishHandler m_cancel;
    TranslationConstraint m_constraint{TranslationConstraint::screen};
    cad::Point3 m_start_pivot;
    TransformFrame m_frame;
    cad::Point3 m_start_plane_point;
    cad::Vector3 m_plane_normal;
    Vec2 m_start_mouse;
    Vec2 m_axis_screen_direction;
    double m_gizmo_scale{1.0};
    double m_axis_screen_length{1.0};
    double m_snap_increment{0.5};
    bool m_dragging{false};
};

class RotationTool final : public IInputTool {
public:
    using ActiveHandler = std::move_only_function<bool()>;
    using PivotHandler = std::move_only_function<std::optional<TransformFrame>()>;
    using BeginHandler =
        std::move_only_function<bool(RotationConstraint, cad::Vector3)>;
    using PreviewHandler = std::move_only_function<bool(double)>;
    using FinishHandler = std::move_only_function<void()>;

    RotationTool(
        ActiveHandler active,
        PivotHandler pivot,
        BeginHandler begin,
        PreviewHandler preview,
        FinishHandler finish,
        FinishHandler cancel
    );
    void process_input(
        const InputFrameSnapshot& input,
        OrbitCameraController& camera_controller,
        Scene& scene
    ) override;

private:
    ActiveHandler m_active;
    PivotHandler m_pivot;
    BeginHandler m_begin;
    PreviewHandler m_preview;
    FinishHandler m_finish;
    FinishHandler m_cancel;
    Vec2 m_center;
    double m_start_angle{0.0};
    bool m_dragging{false};
};

class ScaleTool final : public IInputTool {
public:
    using ActiveHandler = std::move_only_function<bool()>;
    using PivotHandler = std::move_only_function<std::optional<TransformFrame>()>;
    using BeginHandler = std::move_only_function<bool(ScaleConstraint)>;
    using PreviewHandler = std::move_only_function<bool(double)>;
    using FinishHandler = std::move_only_function<void()>;

    ScaleTool(
        ActiveHandler active,
        PivotHandler pivot,
        BeginHandler begin,
        PreviewHandler preview,
        FinishHandler finish,
        FinishHandler cancel
    );
    void process_input(
        const InputFrameSnapshot& input,
        OrbitCameraController& camera_controller,
        Scene& scene
    ) override;

private:
    ActiveHandler m_active;
    PivotHandler m_pivot;
    BeginHandler m_begin;
    PreviewHandler m_preview;
    FinishHandler m_finish;
    FinishHandler m_cancel;
    ScaleConstraint m_constraint{ScaleConstraint::uniform};
    TransformFrame m_frame;
    Vec2 m_start_mouse;
    Vec2 m_axis_screen_direction;
    double m_axis_screen_length{80.0};
    bool m_dragging{false};
};

class ControlPointSelectionTool final : public IInputTool {
public:
    using EnabledHandler = std::move_only_function<bool()>;
    using SelectionHandler =
        std::move_only_function<void(ControlPointSelection, ModifierKeys)>;
    using RectangleHandler =
        std::move_only_function<void(std::vector<ControlPointSelection>, ModifierKeys)>;
    using ClearHandler = std::move_only_function<void()>;

    explicit ControlPointSelectionTool(
        EnabledHandler enabled,
        SelectionHandler on_selection,
        RectangleHandler on_rectangle,
        ClearHandler on_clear = {},
        float hit_radius = 12.0f
    );

    void process_input(
        const InputFrameSnapshot& input,
        OrbitCameraController& camera_controller,
        Scene& scene
    ) override;

private:
    EnabledHandler m_enabled;
    SelectionHandler m_on_selection;
    RectangleHandler m_on_rectangle;
    ClearHandler m_on_clear;
    float m_hit_radius{12.0f};
    Vec2 m_press_position{};
    bool m_selecting{false};
};

class SurfaceSelectionTool final : public IInputTool {
public:
    using EnabledHandler = std::move_only_function<bool()>;
    using SelectionHandler = std::move_only_function<void(EntitySelection)>;
    using HoverHandler = std::move_only_function<void(std::optional<EntityId>)>;
    using ClearHandler = std::move_only_function<void()>;

    SurfaceSelectionTool(
        EnabledHandler enabled,
        SelectionHandler on_selection,
        HoverHandler on_hover,
        ClearHandler on_clear = {}
    );

    void process_input(
        const InputFrameSnapshot& input,
        OrbitCameraController& camera_controller,
        Scene& scene
    ) override;

private:
    EnabledHandler m_enabled;
    SelectionHandler m_on_selection;
    HoverHandler m_on_hover;
    ClearHandler m_on_clear;
    std::vector<EntityId> m_last_hits;
    Vec2 m_last_click_position{};
    std::size_t m_cycle_index{0};
    bool m_has_last_click{false};
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
