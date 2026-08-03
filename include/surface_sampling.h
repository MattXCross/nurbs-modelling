#pragma once

#include "core.h"

#include <cstddef>
#include <expected>
#include <optional>
#include <span>
#include <vector>

class NurbsSurface;

enum class SurfaceSamplingError {
    invalid_segments_per_span,
    numeric_range_not_supported,
    surface_evaluation_failed
};

class NurbsSurfaceSampleGrid {
public:
    [[nodiscard]] std::span<const double> u_parameters() const noexcept {
        return m_u_parameters;
    }
    [[nodiscard]] std::span<const double> v_parameters() const noexcept {
        return m_v_parameters;
    }
    [[nodiscard]] std::span<const Point3D> points() const noexcept { return m_points; }
    [[nodiscard]] std::size_t u_count() const noexcept { return m_u_parameters.size(); }
    [[nodiscard]] std::size_t v_count() const noexcept { return m_v_parameters.size(); }
    [[nodiscard]] std::optional<Point3D> point(std::size_t u, std::size_t v) const noexcept {
        if (u >= u_count() || v >= v_count()) {
            return std::nullopt;
        }
        return m_points[u * v_count() + v];
    }

private:
    friend std::expected<NurbsSurfaceSampleGrid, SurfaceSamplingError> sample_surface_by_knot_spans(
        const NurbsSurface&,
        std::size_t
    );

    std::vector<double> m_u_parameters;
    std::vector<double> m_v_parameters;
    std::vector<Point3D> m_points;
};

[[nodiscard]] std::expected<NurbsSurfaceSampleGrid, SurfaceSamplingError>
sample_surface_by_knot_spans(
    const NurbsSurface& surface,
    std::size_t segments_per_span
);
