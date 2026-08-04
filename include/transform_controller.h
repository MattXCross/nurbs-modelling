#pragma once

#include "command_history.h"
#include "scene.h"
#include "selection.h"
#include "translation.h"

#include <optional>
#include <string_view>
#include <vector>

class TransformController {
public:
    TransformController(Scene& scene, SelectionModel& selection, CommandHistory& history)
        : m_scene(scene), m_selection(selection), m_history(history) {}

    [[nodiscard]] TransformMode mode() const { return m_mode; }
    [[nodiscard]] PivotMode pivot_mode() const { return m_pivot_mode; }
    [[nodiscard]] TransformOrientation orientation() const { return m_orientation; }
    [[nodiscard]] bool set_mode(TransformMode mode);
    [[nodiscard]] bool set_pivot_mode(PivotMode mode);
    [[nodiscard]] bool set_orientation(TransformOrientation orientation);

    [[nodiscard]] std::optional<cad::Point3> pivot() const;
    [[nodiscard]] std::optional<TransformFrame> frame() const;

    [[nodiscard]] bool begin_translation(TranslationConstraint constraint);
    [[nodiscard]] bool preview_translation(cad::Vector3 delta);
    [[nodiscard]] bool finish_translation();
    [[nodiscard]] bool cancel_translation();
    [[nodiscard]] bool translate(cad::Vector3 delta);

    [[nodiscard]] bool begin_rotation(RotationConstraint constraint, cad::Vector3 axis);
    [[nodiscard]] bool preview_rotation(double angle_radians);
    [[nodiscard]] bool finish_rotation();
    [[nodiscard]] bool cancel_rotation();
    [[nodiscard]] bool rotate(cad::Vector3 axis, double angle_radians);

    [[nodiscard]] bool begin_scale(ScaleConstraint constraint);
    [[nodiscard]] bool preview_scale(double factor);
    [[nodiscard]] bool finish_scale();
    [[nodiscard]] bool cancel_scale();
    [[nodiscard]] bool scale(ScaleConstraint constraint, double factor);

    [[nodiscard]] bool translation_active() const { return m_translation.has_value(); }
    [[nodiscard]] bool rotation_active() const { return m_rotation.has_value(); }
    [[nodiscard]] bool scale_active() const { return m_scale.has_value(); }
    [[nodiscard]] bool active() const {
        return translation_active() || rotation_active() || scale_active();
    }
    [[nodiscard]] bool has_preview() const;
    [[nodiscard]] std::string_view active_description() const;
    [[nodiscard]] bool cancel();
    [[nodiscard]] bool commit();
    void reset();

    [[nodiscard]] std::optional<TranslationConstraint> translation_constraint() const;
    [[nodiscard]] std::optional<RotationConstraint> rotation_constraint() const;
    [[nodiscard]] std::optional<ScaleConstraint> scale_constraint() const;

private:
    struct TranslationState {
        std::vector<ControlPointSelection> selections;
        std::vector<ControlPoint> initial_points;
        TranslationConstraint constraint;
        cad::Vector3 delta;
    };
    struct RotationState {
        std::vector<ControlPointSelection> selections;
        std::vector<ControlPoint> initial_points;
        RotationConstraint constraint;
        cad::Vector3 axis;
        cad::Point3 pivot;
        double angle_radians{0.0};
    };
    struct ScaleState {
        std::vector<ControlPointSelection> selections;
        std::vector<ControlPoint> initial_points;
        ScaleConstraint constraint;
        cad::Vector3 axis;
        cad::Point3 pivot;
        double factor{1.0};
    };

    [[nodiscard]] std::vector<ControlPointSelection> targets() const;
    [[nodiscard]] std::vector<ControlPoint> capture(
        const std::vector<ControlPointSelection>& selections
    ) const;

    Scene& m_scene;
    SelectionModel& m_selection;
    CommandHistory& m_history;
    TransformMode m_mode{TransformMode::translate};
    PivotMode m_pivot_mode{PivotMode::selection_center};
    TransformOrientation m_orientation{TransformOrientation::world};
    std::optional<TranslationState> m_translation;
    std::optional<RotationState> m_rotation;
    std::optional<ScaleState> m_scale;
};
