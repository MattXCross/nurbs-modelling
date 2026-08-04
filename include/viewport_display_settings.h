#pragma once

#include "entity_id.h"

enum class SurfaceDisplayMode {
    shaded,
    wireframe,
    shaded_with_edges
};

struct ViewportDisplaySettings {
    SurfaceDisplayMode surface_mode{SurfaceDisplayMode::shaded_with_edges};
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
