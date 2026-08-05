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
    expect(defaults.surface_mode == SurfaceDisplayMode::shaded,
           "plain surface display is the default");
    expect(defaults.quality == ViewportQuality::high, "high viewport quality is the default");
    expect(defaults.show_control_net, "control net is visible by default");
    expect(defaults.show_control_points, "control points are visible by default");

    const auto low = viewport_tessellation_settings(ViewportQuality::low);
    const auto medium = viewport_tessellation_settings(ViewportQuality::medium);
    const auto high = viewport_tessellation_settings(ViewportQuality::high);
    expect(low.chordal_tolerance > medium.chordal_tolerance &&
            medium.chordal_tolerance > high.chordal_tolerance,
        "higher quality tightens chordal tolerance");
    expect(low.normal_angle_tolerance_radians > medium.normal_angle_tolerance_radians &&
            medium.normal_angle_tolerance_radians > high.normal_angle_tolerance_radians,
        "higher quality tightens normal tolerance");
    expect(low.max_refinement_depth < medium.max_refinement_depth &&
            medium.max_refinement_depth < high.max_refinement_depth,
        "higher quality raises refinement depth");
    expect(low.max_vertices < medium.max_vertices && medium.max_vertices < high.max_vertices,
        "higher quality raises the vertex budget");
    expect(low.best_effort && medium.best_effort && high.best_effort,
        "viewport presets always retain a renderable best-effort mesh");
}
