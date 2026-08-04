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
