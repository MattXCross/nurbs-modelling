#pragma once

#include "kernel_math.h"

using Point3D = cad::Point3;

struct ControlPoint {
    Point3D position;
    double weight{1.0};
};

enum class CadError { OutOfBounds = 0, DegenerateSurface, GPUTransferFailed };
