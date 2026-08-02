#pragma once

#include "kernel_math.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>

namespace cad {

class ScalarTolerance {
public:
    [[nodiscard]] static std::optional<ScalarTolerance> create(
        double absolute,
        double relative
    ) noexcept {
        if (!std::isfinite(absolute) || absolute <= 0.0 ||
            !std::isfinite(relative) || relative < 0.0 || relative > 1.0) {
            return std::nullopt;
        }
        return ScalarTolerance(absolute, relative);
    }

    [[nodiscard]] double absolute() const noexcept { return m_absolute; }
    [[nodiscard]] double relative() const noexcept { return m_relative; }

private:
    constexpr ScalarTolerance(double absolute, double relative) noexcept
        : m_absolute(absolute), m_relative(relative) {}

    double m_absolute;
    double m_relative;
};

class GeometryTolerance {
public:
    [[nodiscard]] static std::optional<GeometryTolerance> create(
        double model_absolute,
        double parameter_absolute,
        double angular_radians,
        double relative
    ) noexcept {
        const auto model = ScalarTolerance::create(model_absolute, relative);
        const auto parameter = ScalarTolerance::create(parameter_absolute, relative);
        if (!model || !parameter || !std::isfinite(angular_radians) ||
            angular_radians <= 0.0 || angular_radians >= std::numbers::pi / 2.0) {
            return std::nullopt;
        }
        return GeometryTolerance(*model, *parameter, angular_radians);
    }

    [[nodiscard]] static GeometryTolerance defaults() noexcept {
        return GeometryTolerance(
            *ScalarTolerance::create(1e-9, 1e-12),
            *ScalarTolerance::create(1e-12, 1e-12),
            1e-9
        );
    }

    [[nodiscard]] const ScalarTolerance& model() const noexcept { return m_model; }
    [[nodiscard]] const ScalarTolerance& parameter() const noexcept { return m_parameter; }
    [[nodiscard]] double angular_radians() const noexcept { return m_angular_radians; }

private:
    constexpr GeometryTolerance(
        ScalarTolerance model,
        ScalarTolerance parameter,
        double angular_radians
    ) noexcept
        : m_model(model), m_parameter(parameter), m_angular_radians(angular_radians) {}

    ScalarTolerance m_model;
    ScalarTolerance m_parameter;
    double m_angular_radians;
};

[[nodiscard]] inline bool approximately_equal(
    double left,
    double right,
    const ScalarTolerance& tolerance
) noexcept {
    if (!std::isfinite(left) || !std::isfinite(right)) {
        return false;
    }
    if (left == right) {
        return true;
    }

    const long double difference = std::abs(
        static_cast<long double>(left) - static_cast<long double>(right)
    );
    const long double scale = std::max(
        std::abs(static_cast<long double>(left)),
        std::abs(static_cast<long double>(right))
    );
    const long double allowed =
        static_cast<long double>(tolerance.absolute()) +
        static_cast<long double>(tolerance.relative()) * scale;
    return std::isfinite(difference) && std::isfinite(allowed) && difference <= allowed;
}

[[nodiscard]] inline bool near_zero(
    double value,
    double reference_scale,
    const ScalarTolerance& tolerance
) noexcept {
    if (!std::isfinite(value) || !std::isfinite(reference_scale) || reference_scale < 0.0) {
        return false;
    }
    const long double allowed =
        static_cast<long double>(tolerance.absolute()) +
        static_cast<long double>(tolerance.relative()) *
            static_cast<long double>(reference_scale);
    return std::isfinite(allowed) &&
        std::abs(static_cast<long double>(value)) <= allowed;
}

[[nodiscard]] inline bool parameters_near(
    double left,
    double right,
    const GeometryTolerance& tolerance
) noexcept {
    return approximately_equal(left, right, tolerance.parameter());
}

[[nodiscard]] inline bool points_near(
    Point2 left,
    Point2 right,
    const GeometryTolerance& tolerance
) noexcept {
    if (!is_finite(left) || !is_finite(right)) {
        return false;
    }
    const double separation = distance(left, right);
    return std::isfinite(separation) && separation <= tolerance.model().absolute();
}

[[nodiscard]] inline bool points_near(
    Point3 left,
    Point3 right,
    const GeometryTolerance& tolerance
) noexcept {
    if (!is_finite(left) || !is_finite(right)) {
        return false;
    }
    const double separation = distance(left, right);
    return std::isfinite(separation) && separation <= tolerance.model().absolute();
}

[[nodiscard]] inline bool same_direction(
    Vector2 left,
    Vector2 right,
    const GeometryTolerance& tolerance
) noexcept {
    const auto left_unit = normalized(left);
    const auto right_unit = normalized(right);
    if (!left_unit || !right_unit) {
        return false;
    }
    const double angle = std::atan2(
        std::abs(cross(*left_unit, *right_unit)),
        std::clamp(dot(*left_unit, *right_unit), -1.0, 1.0)
    );
    return angle <= tolerance.angular_radians();
}

[[nodiscard]] inline bool same_direction(
    Vector3 left,
    Vector3 right,
    const GeometryTolerance& tolerance
) noexcept {
    const auto left_unit = normalized(left);
    const auto right_unit = normalized(right);
    if (!left_unit || !right_unit) {
        return false;
    }
    const double angle = std::atan2(
        length(cross(*left_unit, *right_unit)),
        std::clamp(dot(*left_unit, *right_unit), -1.0, 1.0)
    );
    return angle <= tolerance.angular_radians();
}

[[nodiscard]] inline bool parallel(
    Vector3 left,
    Vector3 right,
    const GeometryTolerance& tolerance
) noexcept {
    const auto left_unit = normalized(left);
    const auto right_unit = normalized(right);
    if (!left_unit || !right_unit) {
        return false;
    }
    const double angle = std::atan2(
        length(cross(*left_unit, *right_unit)),
        std::abs(std::clamp(dot(*left_unit, *right_unit), -1.0, 1.0))
    );
    return angle <= tolerance.angular_radians();
}

} // namespace cad
