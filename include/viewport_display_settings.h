#pragma once

#include "entity_id.h"
#include "surface_tessellation.h"

enum class SurfaceDisplayMode {
    shaded,
    wireframe,
    shaded_with_edges
};

enum class ViewportQuality {
    low,
    medium,
    high
};

struct ViewportDisplaySettings {
    SurfaceDisplayMode surface_mode{SurfaceDisplayMode::shaded};
    ViewportQuality quality{ViewportQuality::high};
    bool show_control_net{true};
    bool show_control_points{true};

    bool operator==(const ViewportDisplaySettings&) const = default;
};

struct DisplayColor {
    float red;
    float green;
    float blue;

    bool operator==(const DisplayColor&) const = default;
};

[[nodiscard]] DisplayColor object_display_color(EntityId entity) noexcept;
[[nodiscard]] cad::SurfaceTessellationSettings viewport_tessellation_settings(
    ViewportQuality quality
) noexcept;
