#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <span>

namespace cad {

struct Vector2 {
    double x{0.0};
    double y{0.0};

    bool operator==(const Vector2&) const = default;
};

struct Point2 {
    double x{0.0};
    double y{0.0};

    bool operator==(const Point2&) const = default;
};

struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    bool operator==(const Vector3&) const = default;
};

struct Point3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    bool operator==(const Point3&) const = default;
};

[[nodiscard]] constexpr Vector2 operator+(Vector2 left, Vector2 right) noexcept {
    return {left.x + right.x, left.y + right.y};
}

[[nodiscard]] constexpr Vector2 operator-(Vector2 left, Vector2 right) noexcept {
    return {left.x - right.x, left.y - right.y};
}

[[nodiscard]] constexpr Vector2 operator-(Vector2 vector) noexcept {
    return {-vector.x, -vector.y};
}

[[nodiscard]] constexpr Vector2 operator*(Vector2 vector, double scalar) noexcept {
    return {vector.x * scalar, vector.y * scalar};
}

[[nodiscard]] constexpr Vector2 operator*(double scalar, Vector2 vector) noexcept {
    return vector * scalar;
}

[[nodiscard]] constexpr Vector2 operator/(Vector2 vector, double scalar) noexcept {
    return {vector.x / scalar, vector.y / scalar};
}

[[nodiscard]] constexpr Point2 operator+(Point2 point, Vector2 offset) noexcept {
    return {point.x + offset.x, point.y + offset.y};
}

[[nodiscard]] constexpr Point2 operator-(Point2 point, Vector2 offset) noexcept {
    return {point.x - offset.x, point.y - offset.y};
}

[[nodiscard]] constexpr Vector2 operator-(Point2 left, Point2 right) noexcept {
    return {left.x - right.x, left.y - right.y};
}

[[nodiscard]] constexpr Vector3 operator+(Vector3 left, Vector3 right) noexcept {
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

[[nodiscard]] constexpr Vector3 operator-(Vector3 left, Vector3 right) noexcept {
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

[[nodiscard]] constexpr Vector3 operator-(Vector3 vector) noexcept {
    return {-vector.x, -vector.y, -vector.z};
}

[[nodiscard]] constexpr Vector3 operator*(Vector3 vector, double scalar) noexcept {
    return {vector.x * scalar, vector.y * scalar, vector.z * scalar};
}

[[nodiscard]] constexpr Vector3 operator*(double scalar, Vector3 vector) noexcept {
    return vector * scalar;
}

[[nodiscard]] constexpr Vector3 operator/(Vector3 vector, double scalar) noexcept {
    return {vector.x / scalar, vector.y / scalar, vector.z / scalar};
}

[[nodiscard]] constexpr Point3 operator+(Point3 point, Vector3 offset) noexcept {
    return {point.x + offset.x, point.y + offset.y, point.z + offset.z};
}

[[nodiscard]] constexpr Point3 operator-(Point3 point, Vector3 offset) noexcept {
    return {point.x - offset.x, point.y - offset.y, point.z - offset.z};
}

[[nodiscard]] constexpr Vector3 operator-(Point3 left, Point3 right) noexcept {
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

[[nodiscard]] constexpr double dot(Vector2 left, Vector2 right) noexcept {
    return left.x * right.x + left.y * right.y;
}

[[nodiscard]] constexpr double dot(Vector3 left, Vector3 right) noexcept {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

[[nodiscard]] constexpr double cross(Vector2 left, Vector2 right) noexcept {
    return left.x * right.y - left.y * right.x;
}

[[nodiscard]] constexpr Vector3 cross(Vector3 left, Vector3 right) noexcept {
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x
    };
}

[[nodiscard]] inline double length(Vector2 vector) noexcept {
    return std::hypot(vector.x, vector.y);
}

[[nodiscard]] inline double length(Vector3 vector) noexcept {
    return std::hypot(vector.x, vector.y, vector.z);
}

[[nodiscard]] inline double distance(Point2 left, Point2 right) noexcept {
    return length(left - right);
}

[[nodiscard]] inline double distance(Point3 left, Point3 right) noexcept {
    return length(left - right);
}

[[nodiscard]] constexpr bool is_finite(Vector2 vector) noexcept {
    return std::isfinite(vector.x) && std::isfinite(vector.y);
}

[[nodiscard]] constexpr bool is_finite(Point2 point) noexcept {
    return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] constexpr bool is_finite(Vector3 vector) noexcept {
    return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] constexpr bool is_finite(Point3 point) noexcept {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] inline std::optional<Vector2> normalized(Vector2 vector) noexcept {
    const double scale = std::max(std::abs(vector.x), std::abs(vector.y));
    if (!std::isfinite(scale) || scale == 0.0) {
        return std::nullopt;
    }
    const Vector2 scaled = vector / scale;
    return scaled / length(scaled);
}

[[nodiscard]] inline std::optional<Vector3> normalized(Vector3 vector) noexcept {
    const double scale = std::max({std::abs(vector.x), std::abs(vector.y), std::abs(vector.z)});
    if (!std::isfinite(scale) || scale == 0.0) {
        return std::nullopt;
    }
    const Vector3 scaled = vector / scale;
    return scaled / length(scaled);
}

class Interval {
public:
    [[nodiscard]] static std::optional<Interval> from_bounds(double lower, double upper) noexcept {
        if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper) {
            return std::nullopt;
        }
        return Interval(lower, upper);
    }

    [[nodiscard]] double lower() const noexcept { return m_lower; }
    [[nodiscard]] double upper() const noexcept { return m_upper; }
    [[nodiscard]] double length() const noexcept { return m_upper - m_lower; }
    [[nodiscard]] bool contains(double value) const noexcept {
        return std::isfinite(value) && value >= m_lower && value <= m_upper;
    }

private:
    constexpr Interval(double lower, double upper) noexcept : m_lower(lower), m_upper(upper) {}

    double m_lower;
    double m_upper;
};

class Ray3 {
public:
    [[nodiscard]] static std::optional<Ray3> from_origin_direction(
        Point3 origin,
        Vector3 direction
    ) noexcept {
        const auto unit_direction = normalized(direction);
        if (!is_finite(origin) || !unit_direction) {
            return std::nullopt;
        }
        return Ray3(origin, *unit_direction);
    }

    [[nodiscard]] Point3 origin() const noexcept { return m_origin; }
    [[nodiscard]] Vector3 direction() const noexcept { return m_direction; }
    [[nodiscard]] Point3 at(double parameter) const noexcept {
        return m_origin + m_direction * parameter;
    }

private:
    constexpr Ray3(Point3 origin, Vector3 direction) noexcept
        : m_origin(origin), m_direction(direction) {}

    Point3 m_origin;
    Vector3 m_direction;
};

class Plane {
public:
    [[nodiscard]] static std::optional<Plane> from_point_normal(
        Point3 point,
        Vector3 normal
    ) noexcept {
        const auto unit_normal = normalized(normal);
        if (!is_finite(point) || !unit_normal) {
            return std::nullopt;
        }
        return Plane(point, *unit_normal);
    }

    [[nodiscard]] static std::optional<Plane> from_points(
        Point3 first,
        Point3 second,
        Point3 third
    ) noexcept {
        return from_point_normal(first, cross(second - first, third - first));
    }

    [[nodiscard]] Vector3 normal() const noexcept { return m_normal; }
    [[nodiscard]] Point3 origin() const noexcept { return m_origin; }
    [[nodiscard]] double signed_distance(Point3 point) const noexcept {
        const long double distance =
            static_cast<long double>(m_normal.x) *
                (static_cast<long double>(point.x) - m_origin.x) +
            static_cast<long double>(m_normal.y) *
                (static_cast<long double>(point.y) - m_origin.y) +
            static_cast<long double>(m_normal.z) *
                (static_cast<long double>(point.z) - m_origin.z);
        return static_cast<double>(distance);
    }
    [[nodiscard]] Point3 project(Point3 point) const noexcept {
        return point - m_normal * signed_distance(point);
    }

private:
    constexpr Plane(Point3 origin, Vector3 normal) noexcept
        : m_origin(origin), m_normal(normal) {}

    Point3 m_origin;
    Vector3 m_normal;
};

class Aabb3 {
public:
    [[nodiscard]] bool empty() const noexcept { return m_empty; }

    [[nodiscard]] bool expand(Point3 point) noexcept {
        if (!is_finite(point)) {
            return false;
        }
        if (m_empty) {
            m_minimum = point;
            m_maximum = point;
            m_empty = false;
            return true;
        }
        m_minimum.x = std::min(m_minimum.x, point.x);
        m_minimum.y = std::min(m_minimum.y, point.y);
        m_minimum.z = std::min(m_minimum.z, point.z);
        m_maximum.x = std::max(m_maximum.x, point.x);
        m_maximum.y = std::max(m_maximum.y, point.y);
        m_maximum.z = std::max(m_maximum.z, point.z);
        return true;
    }

    [[nodiscard]] static std::optional<Aabb3> from_points(std::span<const Point3> points) noexcept {
        if (points.empty()) {
            return std::nullopt;
        }
        Aabb3 bounds;
        for (const Point3 point : points) {
            if (!bounds.expand(point)) {
                return std::nullopt;
            }
        }
        return bounds;
    }

    [[nodiscard]] std::optional<Point3> minimum() const noexcept {
        return m_empty ? std::nullopt : std::optional{m_minimum};
    }
    [[nodiscard]] std::optional<Point3> maximum() const noexcept {
        return m_empty ? std::nullopt : std::optional{m_maximum};
    }
    [[nodiscard]] std::optional<Point3> center() const noexcept {
        if (m_empty) {
            return std::nullopt;
        }
        return Point3{
            std::midpoint(m_minimum.x, m_maximum.x),
            std::midpoint(m_minimum.y, m_maximum.y),
            std::midpoint(m_minimum.z, m_maximum.z)
        };
    }
    [[nodiscard]] std::optional<Vector3> extent() const noexcept {
        if (m_empty) {
            return std::nullopt;
        }
        const Vector3 size = m_maximum - m_minimum;
        return is_finite(size) ? std::optional{size} : std::nullopt;
    }
    [[nodiscard]] bool contains(Point3 point) const noexcept {
        return !m_empty && is_finite(point) &&
            point.x >= m_minimum.x && point.x <= m_maximum.x &&
            point.y >= m_minimum.y && point.y <= m_maximum.y &&
            point.z >= m_minimum.z && point.z <= m_maximum.z;
    }

private:
    Point3 m_minimum{};
    Point3 m_maximum{};
    bool m_empty{true};
};

class AffineTransform3 {
public:
    constexpr AffineTransform3() noexcept = default;

    [[nodiscard]] static std::optional<AffineTransform3> translation(Vector3 offset) noexcept {
        if (!is_finite(offset)) {
            return std::nullopt;
        }
        auto transform = AffineTransform3{};
        transform.m_values[3] = offset.x;
        transform.m_values[7] = offset.y;
        transform.m_values[11] = offset.z;
        return transform;
    }

    [[nodiscard]] static std::optional<AffineTransform3> scale(Vector3 factors) noexcept {
        if (!is_finite(factors)) {
            return std::nullopt;
        }
        auto transform = AffineTransform3{};
        transform.m_values[0] = factors.x;
        transform.m_values[5] = factors.y;
        transform.m_values[10] = factors.z;
        return transform;
    }

    [[nodiscard]] static std::optional<AffineTransform3> rotation(
        Vector3 axis,
        double radians
    ) noexcept {
        const auto unit_axis = normalized(axis);
        if (!unit_axis || !std::isfinite(radians)) {
            return std::nullopt;
        }

        const double cosine = std::cos(radians);
        const double sine = std::sin(radians);
        const double one_minus_cosine = 1.0 - cosine;
        const double x = unit_axis->x;
        const double y = unit_axis->y;
        const double z = unit_axis->z;
        return AffineTransform3({
            cosine + x * x * one_minus_cosine,
            x * y * one_minus_cosine - z * sine,
            x * z * one_minus_cosine + y * sine,
            0.0,
            y * x * one_minus_cosine + z * sine,
            cosine + y * y * one_minus_cosine,
            y * z * one_minus_cosine - x * sine,
            0.0,
            z * x * one_minus_cosine - y * sine,
            z * y * one_minus_cosine + x * sine,
            cosine + z * z * one_minus_cosine,
            0.0
        });
    }

    [[nodiscard]] static std::optional<AffineTransform3> reflection(const Plane& plane) noexcept {
        const Vector3 normal = plane.normal();
        const Point3 origin = plane.origin();
        const double x = normal.x;
        const double y = normal.y;
        const double z = normal.z;
        const long double projection =
            static_cast<long double>(x) * origin.x +
            static_cast<long double>(y) * origin.y +
            static_cast<long double>(z) * origin.z;
        const std::array<long double, 3> translation{
            2.0L * projection * x,
            2.0L * projection * y,
            2.0L * projection * z
        };
        for (const long double component : translation) {
            if (!std::isfinite(component) ||
                component < std::numeric_limits<double>::lowest() ||
                component > std::numeric_limits<double>::max()) {
                return std::nullopt;
            }
        }
        AffineTransform3 transform({
            1.0 - 2.0 * x * x, -2.0 * x * y, -2.0 * x * z,
            static_cast<double>(translation[0]),
            -2.0 * y * x, 1.0 - 2.0 * y * y, -2.0 * y * z,
            static_cast<double>(translation[1]),
            -2.0 * z * x, -2.0 * z * y, 1.0 - 2.0 * z * z,
            static_cast<double>(translation[2])
        });
        // Anchor the matrix to the represented plane using the same arithmetic
        // transform_point() will use, avoiding cancellation drift at large coordinates.
        for (int iteration = 0; iteration < 2; ++iteration) {
            const Point3 mapped_origin = transform.transform_point(origin);
            if (!is_finite(mapped_origin)) {
                return std::nullopt;
            }
            transform.m_values[3] += origin.x - mapped_origin.x;
            transform.m_values[7] += origin.y - mapped_origin.y;
            transform.m_values[11] += origin.z - mapped_origin.z;
        }
        if (transform.transform_point(origin) != origin) {
            return std::nullopt;
        }
        return transform;
    }

    [[nodiscard]] Point3 transform_point(Point3 point) const noexcept {
        return {
            m_values[0] * point.x + m_values[1] * point.y + m_values[2] * point.z + m_values[3],
            m_values[4] * point.x + m_values[5] * point.y + m_values[6] * point.z + m_values[7],
            m_values[8] * point.x + m_values[9] * point.y + m_values[10] * point.z + m_values[11]
        };
    }

    [[nodiscard]] Vector3 transform_vector(Vector3 vector) const noexcept {
        return {
            m_values[0] * vector.x + m_values[1] * vector.y + m_values[2] * vector.z,
            m_values[4] * vector.x + m_values[5] * vector.y + m_values[6] * vector.z,
            m_values[8] * vector.x + m_values[9] * vector.y + m_values[10] * vector.z
        };
    }

    // Composition follows function order: (left * right)(point) = left(right(point)).
    [[nodiscard]] friend AffineTransform3 operator*(
        const AffineTransform3& left,
        const AffineTransform3& right
    ) noexcept {
        std::array<double, 12> result{};
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t column = 0; column < 3; ++column) {
                result[row * 4 + column] =
                    left.m_values[row * 4] * right.m_values[column] +
                    left.m_values[row * 4 + 1] * right.m_values[4 + column] +
                    left.m_values[row * 4 + 2] * right.m_values[8 + column];
            }
            result[row * 4 + 3] =
                left.m_values[row * 4] * right.m_values[3] +
                left.m_values[row * 4 + 1] * right.m_values[7] +
                left.m_values[row * 4 + 2] * right.m_values[11] +
                left.m_values[row * 4 + 3];
        }
        return AffineTransform3(result);
    }

private:
    explicit constexpr AffineTransform3(std::array<double, 12> values) noexcept
        : m_values(values) {}

    std::array<double, 12> m_values{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0
    };
};

} // namespace cad
