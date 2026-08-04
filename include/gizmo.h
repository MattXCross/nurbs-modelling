#pragma once

#include "input_frame.h"
#include "orbit_camera.h"

#include <cstdint>
#include <variant>
#include <vector>

struct GizmoColor {
    std::uint8_t red{255};
    std::uint8_t green{255};
    std::uint8_t blue{255};
    std::uint8_t alpha{255};
};

struct GizmoLine {
    cad::Point3 start;
    cad::Point3 end;
    double radius{0.0};
    GizmoColor color;
};

struct GizmoArrow {
    cad::Point3 start;
    cad::Point3 shaft_end;
    cad::Point3 head_start;
    cad::Point3 tip;
    double shaft_radius{0.0};
    double head_radius{0.0};
    GizmoColor color;
};

struct GizmoBox {
    cad::Point3 center;
    double size{0.0};
    GizmoColor color;
};

struct GizmoPlaneHandle {
    cad::Point3 first;
    cad::Point3 second;
    cad::Point3 third;
    GizmoColor color;
};

struct GizmoRing {
    cad::Point3 center;
    cad::Vector3 normal;
    double radius{0.0};
    GizmoColor color;
};

enum class GizmoCenterShape { sphere, box };

struct GizmoCenterHandle {
    cad::Point3 center;
    double size{0.0};
    GizmoCenterShape shape{GizmoCenterShape::sphere};
    GizmoColor color;
};

using GizmoPrimitive = std::variant<
    GizmoLine,
    GizmoArrow,
    GizmoBox,
    GizmoPlaneHandle,
    GizmoRing,
    GizmoCenterHandle
>;
using GizmoDrawList = std::vector<GizmoPrimitive>;

class IGizmo {
public:
    virtual ~IGizmo() = default;
    [[nodiscard]] virtual bool process_input(
        const InputFrameSnapshot& input,
        const CameraState& camera
    ) = 0;
    virtual void append_draw_data(
        GizmoDrawList& draw_list,
        const CameraState& camera,
        int viewport_height
    ) const = 0;
};
