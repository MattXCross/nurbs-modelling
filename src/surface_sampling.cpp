#include "surface_sampling.h"

#include "nurbs_surface.h"

#include <cmath>
#include <limits>
#include <span>
#include <utility>
#include <vector>

namespace {

std::expected<std::vector<double>, SurfaceSamplingError> sample_parameters(
    std::size_t control_count,
    std::size_t degree,
    std::span<const double> knots,
    std::size_t segments_per_span
) {
    if (segments_per_span == 0) {
        return std::unexpected(SurfaceSamplingError::invalid_segments_per_span);
    }

    std::size_t nonempty_span_count = 0;
    for (std::size_t index = degree; index < control_count; ++index) {
        if (knots[index] < knots[index + 1]) {
            ++nonempty_span_count;
        }
    }
    if (nonempty_span_count == 0 ||
        nonempty_span_count > (std::numeric_limits<std::size_t>::max() - 1) /
            segments_per_span) {
        return std::unexpected(SurfaceSamplingError::numeric_range_not_supported);
    }
    const std::size_t sample_count = nonempty_span_count * segments_per_span + 1;
    std::vector<double> result;
    if (sample_count > result.max_size()) {
        return std::unexpected(SurfaceSamplingError::numeric_range_not_supported);
    }
    result.reserve(sample_count);

    for (std::size_t index = degree; index < control_count; ++index) {
        const double start = knots[index];
        const double end = knots[index + 1];
        if (start >= end) {
            continue;
        }
        if (result.empty()) {
            result.push_back(start);
        }
        for (std::size_t segment = 1; segment <= segments_per_span; ++segment) {
            if (segment == segments_per_span) {
                result.push_back(end);
                continue;
            }
            const long double fraction = static_cast<long double>(segment) /
                static_cast<long double>(segments_per_span);
            const long double parameter = static_cast<long double>(start) +
                (static_cast<long double>(end) - static_cast<long double>(start)) * fraction;
            if (!std::isfinite(parameter) ||
                parameter < std::numeric_limits<double>::lowest() ||
                parameter > std::numeric_limits<double>::max()) {
                return std::unexpected(SurfaceSamplingError::numeric_range_not_supported);
            }
            const double sampled = static_cast<double>(parameter);
            if (sampled <= result.back() || sampled >= end) {
                return std::unexpected(SurfaceSamplingError::numeric_range_not_supported);
            }
            result.push_back(sampled);
        }
    }
    return result;
}

} // namespace

std::expected<NurbsSurfaceSampleGrid, SurfaceSamplingError> sample_surface_by_knot_spans(
    const NurbsSurface& surface,
    std::size_t segments_per_span
) {
    auto u_parameters = sample_parameters(
        surface.u_count(),
        surface.u_degree(),
        surface.u_knots(),
        segments_per_span
    );
    if (!u_parameters) {
        return std::unexpected(u_parameters.error());
    }
    auto v_parameters = sample_parameters(
        surface.v_count(),
        surface.v_degree(),
        surface.v_knots(),
        segments_per_span
    );
    if (!v_parameters) {
        return std::unexpected(v_parameters.error());
    }
    if (u_parameters->size() >
        std::numeric_limits<std::size_t>::max() / v_parameters->size()) {
        return std::unexpected(SurfaceSamplingError::numeric_range_not_supported);
    }

    NurbsSurfaceSampleGrid result;
    result.m_u_parameters = std::move(*u_parameters);
    result.m_v_parameters = std::move(*v_parameters);
    const std::size_t point_count = result.u_count() * result.v_count();
    if (point_count > result.m_points.max_size()) {
        return std::unexpected(SurfaceSamplingError::numeric_range_not_supported);
    }
    result.m_points.reserve(point_count);
    for (const double u : result.m_u_parameters) {
        for (const double v : result.m_v_parameters) {
            const auto point = surface.evaluate(u, v);
            if (!point) {
                return std::unexpected(SurfaceSamplingError::surface_evaluation_failed);
            }
            result.m_points.push_back(*point);
        }
    }
    return result;
}
