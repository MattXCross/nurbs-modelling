#include "surface_tessellation.h"

#include "nurbs_surface.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <numbers>
#include <span>
#include <utility>

namespace cad {
namespace {

constexpr std::size_t maximum_supported_depth = 30;

bool valid_settings(const SurfaceTessellationSettings& settings,
                    SurfaceTessellationErrorCode& error) {
    if (!std::isfinite(settings.chordal_tolerance) || settings.chordal_tolerance <= 0.0) {
        error = SurfaceTessellationErrorCode::invalid_chordal_tolerance;
        return false;
    }
    if (!std::isfinite(settings.normal_angle_tolerance_radians) ||
        settings.normal_angle_tolerance_radians <= 0.0 ||
        settings.normal_angle_tolerance_radians > std::numbers::pi) {
        error = SurfaceTessellationErrorCode::invalid_normal_angle_tolerance;
        return false;
    }
    if (settings.max_refinement_depth > maximum_supported_depth) {
        error = SurfaceTessellationErrorCode::invalid_max_refinement_depth;
        return false;
    }
    if (settings.max_vertices < 4 ||
        settings.max_vertices > std::numeric_limits<std::uint32_t>::max()) {
        error = SurfaceTessellationErrorCode::invalid_max_vertices;
        return false;
    }
    return true;
}

std::expected<std::vector<double>, SurfaceTessellationError> parameters_at_depth(
    std::size_t control_count,
    std::size_t degree,
    std::span<const double> knots,
    std::size_t depth
) {
    const std::size_t segments = std::size_t{1} << depth;
    std::size_t span_count = 0;
    for (std::size_t i = degree; i < control_count; ++i) {
        span_count += knots[i] < knots[i + 1] ? 1 : 0;
    }
    if (span_count == 0 || span_count > (std::numeric_limits<std::size_t>::max() - 1) / segments) {
        return std::unexpected(SurfaceTessellationError{
            SurfaceTessellationErrorCode::numeric_range_not_supported, depth
        });
    }

    std::vector<double> result;
    result.reserve(span_count * segments + 1);
    for (std::size_t i = degree; i < control_count; ++i) {
        const double start = knots[i];
        const double end = knots[i + 1];
        if (start >= end) continue;
        if (result.empty()) result.push_back(start);
        for (std::size_t segment = 1; segment <= segments; ++segment) {
            if (segment == segments) {
                result.push_back(end);
                continue;
            }
            const long double value = static_cast<long double>(start) +
                (static_cast<long double>(end) - start) * segment / segments;
            const double parameter = static_cast<double>(value);
            if (!std::isfinite(value) || parameter <= result.back() || parameter >= end) {
                return std::unexpected(SurfaceTessellationError{
                    SurfaceTessellationErrorCode::numeric_range_not_supported, depth
                });
            }
            result.push_back(parameter);
        }
    }
    return result;
}

Point3 midpoint(Point3 a, Point3 b) {
    return {std::midpoint(a.x, b.x), std::midpoint(a.y, b.y), std::midpoint(a.z, b.z)};
}

Point3 bilinear_center(Point3 p00, Point3 p10, Point3 p01, Point3 p11) {
    return midpoint(midpoint(p00, p11), midpoint(p10, p01));
}

std::optional<Vector3> regular_normal(const NurbsSurface& surface, double u, double v) {
    const auto derivatives = surface.evaluate_derivatives(u, v);
    if (!derivatives) return std::nullopt;
    return normalized(cross(derivatives->u, derivatives->v));
}

double normal_angle(Vector3 a, Vector3 b) {
    return std::acos(std::clamp(dot(a, b), -1.0, 1.0));
}

struct GridSamples {
    std::vector<double> u;
    std::vector<double> v;
    std::vector<Point3> positions;
    std::vector<std::optional<Vector3>> normals;
};

std::expected<GridSamples, SurfaceTessellationError> evaluate_grid(
    const NurbsSurface& surface,
    std::size_t depth,
    const SurfaceTessellationSettings& settings
) {
    auto u = parameters_at_depth(
        surface.u_count(), surface.u_degree(), surface.u_knots(), depth
    );
    if (!u) return std::unexpected(u.error());
    auto v = parameters_at_depth(
        surface.v_count(), surface.v_degree(), surface.v_knots(), depth
    );
    if (!v) return std::unexpected(v.error());
    if (u->size() > settings.max_vertices / v->size()) {
        return std::unexpected(SurfaceTessellationError{
            SurfaceTessellationErrorCode::resource_limit_exceeded, depth
        });
    }

    GridSamples grid{std::move(*u), std::move(*v), {}, {}};
    const std::size_t count = grid.u.size() * grid.v.size();
    grid.positions.reserve(count);
    grid.normals.reserve(count);
    for (double parameter_u : grid.u) {
        for (double parameter_v : grid.v) {
            const auto point = surface.evaluate(parameter_u, parameter_v);
            if (!point || !is_finite(*point)) {
                return std::unexpected(SurfaceTessellationError{
                    SurfaceTessellationErrorCode::surface_evaluation_failed, depth
                });
            }
            grid.positions.push_back(*point);
            grid.normals.push_back(regular_normal(surface, parameter_u, parameter_v));
        }
    }
    return grid;
}

bool cell_meets_tolerance(
    const NurbsSurface& surface,
    const GridSamples& grid,
    std::size_t ui,
    std::size_t vi,
    const SurfaceTessellationSettings& settings
) {
    const std::size_t row = grid.v.size();
    const std::array<std::size_t, 4> indices{
        ui * row + vi, (ui + 1) * row + vi,
        ui * row + vi + 1, (ui + 1) * row + vi + 1
    };
    const Point3 p00 = grid.positions[indices[0]];
    const Point3 p10 = grid.positions[indices[1]];
    const Point3 p01 = grid.positions[indices[2]];
    const Point3 p11 = grid.positions[indices[3]];
    const double u0 = grid.u[ui];
    const double u1 = grid.u[ui + 1];
    const double v0 = grid.v[vi];
    const double v1 = grid.v[vi + 1];
    const double um = std::midpoint(u0, u1);
    const double vm = std::midpoint(v0, v1);
    const std::array<std::pair<double, double>, 5> parameters{{
        {um, v0}, {um, v1}, {u0, vm}, {u1, vm}, {um, vm}
    }};
    const std::array<Point3, 5> approximations{
        midpoint(p00, p10), midpoint(p01, p11), midpoint(p00, p01),
        midpoint(p10, p11), bilinear_center(p00, p10, p01, p11)
    };

    std::vector<Vector3> normals;
    normals.reserve(9);
    for (std::size_t index : indices) {
        if (grid.normals[index]) normals.push_back(*grid.normals[index]);
    }
    for (std::size_t sample = 0; sample < parameters.size(); ++sample) {
        const auto [u, v] = parameters[sample];
        const auto point = surface.evaluate(u, v);
        if (!point || !is_finite(*point) ||
            distance(*point, approximations[sample]) > settings.chordal_tolerance) {
            return false;
        }
        if (const auto normal = regular_normal(surface, u, v)) normals.push_back(*normal);
    }
    for (std::size_t i = 0; i < normals.size(); ++i) {
        for (std::size_t j = i + 1; j < normals.size(); ++j) {
            if (normal_angle(normals[i], normals[j]) > settings.normal_angle_tolerance_radians) {
                return false;
            }
        }
    }
    return true;
}

bool grid_meets_tolerance(
    const NurbsSurface& surface,
    const GridSamples& grid,
    const SurfaceTessellationSettings& settings
) {
    for (std::size_t u = 0; u + 1 < grid.u.size(); ++u) {
        for (std::size_t v = 0; v + 1 < grid.v.size(); ++v) {
            if (!cell_meets_tolerance(surface, grid, u, v, settings)) return false;
        }
    }
    return true;
}

SurfaceMesh make_mesh(const GridSamples& grid) {
    SurfaceMesh mesh;
    mesh.positions = grid.positions;
    mesh.normals.resize(mesh.positions.size());
    mesh.uvs.reserve(mesh.positions.size());
    const double u_min = grid.u.front();
    const double u_range = grid.u.back() - u_min;
    const double v_min = grid.v.front();
    const double v_range = grid.v.back() - v_min;
    for (double u : grid.u) {
        for (double v : grid.v) {
            mesh.uvs.push_back({(u - u_min) / u_range, (v - v_min) / v_range});
        }
    }

    const std::size_t row = grid.v.size();
    std::vector<Vector3> fallback_sums(mesh.positions.size());
    const auto append_triangle = [&](std::size_t a, std::size_t b, std::size_t c) {
        const auto face = normalized(cross(mesh.positions[b] - mesh.positions[a],
                                          mesh.positions[c] - mesh.positions[a]));
        if (!face) return;
        mesh.triangle_indices.push_back(static_cast<std::uint32_t>(a));
        mesh.triangle_indices.push_back(static_cast<std::uint32_t>(b));
        mesh.triangle_indices.push_back(static_cast<std::uint32_t>(c));
        fallback_sums[a] = fallback_sums[a] + *face;
        fallback_sums[b] = fallback_sums[b] + *face;
        fallback_sums[c] = fallback_sums[c] + *face;
    };
    for (std::size_t u = 0; u + 1 < grid.u.size(); ++u) {
        for (std::size_t v = 0; v + 1 < grid.v.size(); ++v) {
            const std::size_t p00 = u * row + v;
            const std::size_t p10 = (u + 1) * row + v;
            const std::size_t p01 = p00 + 1;
            const std::size_t p11 = p10 + 1;
            append_triangle(p00, p10, p11);
            append_triangle(p00, p11, p01);
        }
    }
    for (std::size_t i = 0; i < mesh.normals.size(); ++i) {
        // A collapsed patch has no geometric direction; +Z is the deterministic fallback.
        mesh.normals[i] = grid.normals[i].value_or(
            normalized(fallback_sums[i]).value_or(Vector3{0.0, 0.0, 1.0})
        );
    }
    return mesh;
}

void hash_combine(std::size_t& seed, std::size_t value) noexcept {
    seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
}

} // namespace

std::expected<SurfaceMesh, SurfaceTessellationError> tessellate_surface(
    const NurbsSurface& surface,
    const SurfaceTessellationSettings& settings
) {
    SurfaceTessellationErrorCode settings_error{};
    if (!valid_settings(settings, settings_error)) {
        return std::unexpected(SurfaceTessellationError{settings_error});
    }
    std::optional<GridSamples> last_grid;
    for (std::size_t depth = 0; depth <= settings.max_refinement_depth; ++depth) {
        auto grid = evaluate_grid(surface, depth, settings);
        if (!grid) {
            if (settings.best_effort && last_grid &&
                grid.error().code == SurfaceTessellationErrorCode::resource_limit_exceeded) {
                return make_mesh(*last_grid);
            }
            return std::unexpected(grid.error());
        }
        if (grid_meets_tolerance(surface, *grid, settings)) return make_mesh(*grid);
        if (depth == settings.max_refinement_depth) {
            if (settings.best_effort) return make_mesh(*grid);
            return std::unexpected(SurfaceTessellationError{
                SurfaceTessellationErrorCode::refinement_limit_reached, depth
            });
        }
        last_grid = std::move(*grid);
    }
    return std::unexpected(SurfaceTessellationError{
        SurfaceTessellationErrorCode::refinement_limit_reached,
        settings.max_refinement_depth
    });
}

std::size_t SurfaceTessellationCache::KeyHash::operator()(const Key& key) const noexcept {
    std::size_t seed = std::hash<std::uint64_t>{}(key.surface_identity);
    hash_combine(seed, std::hash<std::uint64_t>{}(key.geometry_revision));
    hash_combine(seed, std::hash<std::uint64_t>{}(
        std::bit_cast<std::uint64_t>(key.settings.chordal_tolerance)
    ));
    hash_combine(seed, std::hash<std::uint64_t>{}(
        std::bit_cast<std::uint64_t>(key.settings.normal_angle_tolerance_radians)
    ));
    hash_combine(seed, std::hash<std::size_t>{}(key.settings.max_refinement_depth));
    hash_combine(seed, std::hash<std::size_t>{}(key.settings.max_vertices));
    hash_combine(seed, std::hash<bool>{}(key.settings.best_effort));
    return seed;
}

std::expected<std::shared_ptr<const SurfaceMesh>, SurfaceTessellationError>
SurfaceTessellationCache::get(
    const NurbsSurface& surface,
    std::uint64_t geometry_revision,
    const SurfaceTessellationSettings& settings
) {
    const Key key{surface.identity(), geometry_revision, settings};
    if (const auto found = m_entries.find(key); found != m_entries.end()) return found->second;
    auto mesh = tessellate_surface(surface, settings);
    if (!mesh) return std::unexpected(mesh.error());
    auto stored = std::make_shared<const SurfaceMesh>(std::move(*mesh));
    m_entries.emplace(key, stored);
    return stored;
}

void SurfaceTessellationCache::clear(const NurbsSurface& surface) {
    std::erase_if(m_entries, [&surface](const auto& entry) {
        return entry.first.surface_identity == surface.identity();
    });
}

} // namespace cad
