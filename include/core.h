#pragma once

#include <concepts>

template<typename T>
concept GeometricScalar = std::floating_point<T>;

template<typename T>
concept Vector3D = requires (T vector, double scalar_mult) {
    { vector.x } -> std::convertible_to<double>;
    { vector.y } -> std::convertible_to<double>;
    { vector.z } -> std::convertible_to<double>;
    { vector + vector } -> std::same_as<T>;
    { vector * scalar_mult } -> std::same_as<T>;
};

struct Point3D {
    double x{0.0}, y{0.0}, z{0.0};
    
    Point3D operator+(const Point3D& other) const {
        return { x + other.x, y + other.y, z + other.z };
    }

    Point3D operator*(double scalar_mult) const {
        return { x * scalar_mult, y * scalar_mult, z * scalar_mult };
    }
};

struct ControlPoint {
    Point3D position;
    double weight{1.0};
};

enum class CadError { OutOfBounds = 0, DegenerateSurface, GPUTransferFailed };