#include "viewport_display_settings.h"

#include <cstdlib>
#include <iostream>

namespace {

void expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

bool valid(DisplayColor color) {
    return color.red >= 0.0f && color.red <= 1.0f &&
        color.green >= 0.0f && color.green <= 1.0f &&
        color.blue >= 0.0f && color.blue <= 1.0f;
}

} // namespace

int main() {
    const DisplayColor first = object_display_color(EntityId{42});
    expect(first == object_display_color(EntityId{42}), "entity color is deterministic");
    expect(first != object_display_color(EntityId{43}), "nearby entity IDs differ");
    expect(valid(first), "entity color is normalized");
    expect(first.red == first.green && first.blue - first.red <= 0.02f,
           "entity color uses a neutral gray palette");

    const ViewportDisplaySettings defaults;
    expect(defaults.surface_mode == SurfaceDisplayMode::shaded_with_edges,
           "combined display mode is the default");
    expect(defaults.show_control_net, "control net is visible by default");
    expect(defaults.show_control_points, "control points are visible by default");
}
