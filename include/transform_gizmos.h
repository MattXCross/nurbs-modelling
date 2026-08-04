#pragma once

#include "gizmo.h"
#include "translation.h"

#include <functional>
#include <optional>

class TranslationGizmo final : public IGizmo {
public:
    using ActiveHandler = std::move_only_function<std::optional<TranslationConstraint>() const>;
    using FrameHandler = std::move_only_function<std::optional<TransformFrame>() const>;
    using BeginHandler = std::move_only_function<bool(TranslationConstraint)>;
    using PreviewHandler = std::move_only_function<bool(cad::Vector3)>;
    using FinishHandler = std::move_only_function<void()>;

    TranslationGizmo(
        ActiveHandler active,
        FrameHandler frame,
        BeginHandler begin,
        PreviewHandler preview,
        FinishHandler finish,
        FinishHandler cancel,
        double snap_increment = 0.5
    );

    [[nodiscard]] bool process_input(
        const InputFrameSnapshot& input,
        const CameraState& camera
    ) override;
    void append_draw_data(
        GizmoDrawList& draw_list,
        const CameraState& camera,
        int viewport_height
    ) const override;

private:
    ActiveHandler m_active;
    FrameHandler m_frame_provider;
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

class RotationGizmo final : public IGizmo {
public:
    using ActiveHandler = std::move_only_function<std::optional<RotationConstraint>() const>;
    using FrameHandler = std::move_only_function<std::optional<TransformFrame>() const>;
    using BeginHandler = std::move_only_function<bool(RotationConstraint, cad::Vector3)>;
    using PreviewHandler = std::move_only_function<bool(double)>;
    using FinishHandler = std::move_only_function<void()>;

    RotationGizmo(
        ActiveHandler active,
        FrameHandler frame,
        BeginHandler begin,
        PreviewHandler preview,
        FinishHandler finish,
        FinishHandler cancel
    );
    [[nodiscard]] bool process_input(
        const InputFrameSnapshot& input,
        const CameraState& camera
    ) override;
    void append_draw_data(
        GizmoDrawList& draw_list,
        const CameraState& camera,
        int viewport_height
    ) const override;

private:
    ActiveHandler m_active;
    FrameHandler m_frame_provider;
    BeginHandler m_begin;
    PreviewHandler m_preview;
    FinishHandler m_finish;
    FinishHandler m_cancel;
    Vec2 m_center;
    double m_start_angle{0.0};
    bool m_dragging{false};
};

class ScaleGizmo final : public IGizmo {
public:
    using ActiveHandler = std::move_only_function<std::optional<ScaleConstraint>() const>;
    using FrameHandler = std::move_only_function<std::optional<TransformFrame>() const>;
    using BeginHandler = std::move_only_function<bool(ScaleConstraint)>;
    using PreviewHandler = std::move_only_function<bool(double)>;
    using FinishHandler = std::move_only_function<void()>;

    ScaleGizmo(
        ActiveHandler active,
        FrameHandler frame,
        BeginHandler begin,
        PreviewHandler preview,
        FinishHandler finish,
        FinishHandler cancel
    );
    [[nodiscard]] bool process_input(
        const InputFrameSnapshot& input,
        const CameraState& camera
    ) override;
    void append_draw_data(
        GizmoDrawList& draw_list,
        const CameraState& camera,
        int viewport_height
    ) const override;

private:
    ActiveHandler m_active;
    FrameHandler m_frame_provider;
    BeginHandler m_begin;
    PreviewHandler m_preview;
    FinishHandler m_finish;
    FinishHandler m_cancel;
    ScaleConstraint m_constraint{ScaleConstraint::uniform};
    Vec2 m_start_mouse;
    Vec2 m_axis_screen_direction;
    double m_axis_screen_length{80.0};
    bool m_dragging{false};
};
