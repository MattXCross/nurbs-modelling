#pragma once

#include "kernel_math.h"

#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <unordered_map>
#include <vector>

class NurbsSurface;

namespace cad {

struct SurfaceTessellationSettings {
    double chordal_tolerance{0.01};
    double normal_angle_tolerance_radians{0.08726646259971647};
    std::size_t max_refinement_depth{8};
    std::size_t max_vertices{1'000'000};

    bool operator==(const SurfaceTessellationSettings&) const = default;
};

enum class SurfaceTessellationErrorCode {
    invalid_chordal_tolerance,
    invalid_normal_angle_tolerance,
    invalid_max_refinement_depth,
    invalid_max_vertices,
    numeric_range_not_supported,
    resource_limit_exceeded,
    refinement_limit_reached,
    surface_evaluation_failed
};

struct SurfaceTessellationError {
    SurfaceTessellationErrorCode code;
    std::size_t refinement_depth{0};
};

struct SurfaceMesh {
    std::vector<Point3> positions;
    std::vector<Vector3> normals;
    // UVs are normalized to [0, 1] over the surface's actual parameter domain.
    std::vector<Point2> uvs;
    std::vector<std::uint32_t> triangle_indices;
};

[[nodiscard]] std::expected<SurfaceMesh, SurfaceTessellationError> tessellate_surface(
    const NurbsSurface& surface,
    const SurfaceTessellationSettings& settings = {}
);

class SurfaceTessellationCache {
public:
    [[nodiscard]] std::expected<std::shared_ptr<const SurfaceMesh>, SurfaceTessellationError> get(
        const NurbsSurface& surface,
        std::uint64_t geometry_revision,
        const SurfaceTessellationSettings& settings = {}
    );

    void clear(const NurbsSurface& surface);
    void clear() noexcept { m_entries.clear(); }
    [[nodiscard]] std::size_t size() const noexcept { return m_entries.size(); }

private:
    struct Key {
        std::uint64_t surface_identity;
        std::uint64_t geometry_revision;
        SurfaceTessellationSettings settings;

        bool operator==(const Key&) const = default;
    };

    struct KeyHash {
        [[nodiscard]] std::size_t operator()(const Key& key) const noexcept;
    };

    std::unordered_map<Key, std::shared_ptr<const SurfaceMesh>, KeyHash> m_entries;
};

} // namespace cad
