#include "viewport_display_settings.h"

#include <cstdint>

DisplayColor object_display_color(EntityId entity) noexcept {
    std::uint64_t value = entity.value + 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
    value ^= value >> 31;

    const float shade = 0.66f + static_cast<float>(value & 0xffffU) / 65535.0f * 0.14f;
    return {shade, shade, shade + 0.015f};
}

cad::SurfaceTessellationSettings viewport_tessellation_settings(
    ViewportQuality quality
) noexcept {
    switch (quality) {
        case ViewportQuality::low:
            return {
                .chordal_tolerance = 0.08,
                .normal_angle_tolerance_radians = 0.3,
                .max_refinement_depth = 5,
                .max_vertices = 65'536,
                .best_effort = true
            };
        case ViewportQuality::medium:
            return {
                .chordal_tolerance = 0.025,
                .normal_angle_tolerance_radians = 0.15,
                .max_refinement_depth = 7,
                .max_vertices = 262'144,
                .best_effort = true
            };
        case ViewportQuality::high:
            return {
                .chordal_tolerance = 0.01,
                .normal_angle_tolerance_radians = 0.08726646259971647,
                .max_refinement_depth = 8,
                .max_vertices = 1'000'000,
                .best_effort = true
            };
    }
    return {};
}
