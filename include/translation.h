#pragma once

#include "kernel_math.h"

enum class TranslationConstraint { x, y, z, xy, xz, yz, screen };
enum class TransformMode { translate, rotate, scale };
enum class PivotMode { selection_center, primary_control_point, world_origin };
enum class RotationConstraint { x, y, z, screen };
enum class ScaleConstraint { x, y, z, uniform };
enum class TransformOrientation { world, local };

struct TransformFrame {
    cad::Point3 pivot;
    cad::Vector3 x{1.0, 0.0, 0.0};
    cad::Vector3 y{0.0, 1.0, 0.0};
    cad::Vector3 z{0.0, 0.0, 1.0};
};
