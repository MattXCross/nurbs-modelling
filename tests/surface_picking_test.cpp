#include "surface_picking.h"

#include "core.h"
#include "input_frame.h"
#include "input_tools.h"
#include "nurbs_surface.h"
#include "orbit_camera.h"
#include "scene.h"
#include "selection.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <string_view>
#include <utility>

namespace {

int failures = 0;

void expect(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

std::unique_ptr<NurbsSurface> make_plane(double z) {
    auto surface = NurbsSurface::create(2, 2, 1, 1, {
        {{-2.0, -2.0, z}, 1.0},
        {{-2.0, 2.0, z}, 1.0},
        {{2.0, -2.0, z}, 1.0},
        {{2.0, 2.0, z}, 1.0}
    });
    expect(surface.has_value(), "construct picking plane");
    return surface ? std::move(*surface) : nullptr;
}

CameraState test_camera() {
    return {
        .position = {0.0, 0.0, 10.0},
        .target = {0.0, 0.0, 0.0},
        .up = {0.0, 1.0, 0.0},
        .vertical_fov_degrees = 45.0f
    };
}

void test_viewport_ray() {
    const auto center = make_viewport_ray({400.0f, 300.0f}, 800, 600, test_camera());
    expect(center.has_value(), "construct center viewport ray");
    if (center) {
        expect(center->origin() == cad::Point3{0.0, 0.0, 10.0}, "ray starts at camera");
        expect(std::abs(center->direction().x) < 1e-15, "center ray has no X component");
        expect(std::abs(center->direction().y) < 1e-15, "center ray has no Y component");
        expect(std::abs(center->direction().z + 1.0) < 1e-15, "center ray points forward");
    }
    expect(!make_viewport_ray({}, 0, 600, test_camera()), "zero-width viewport is rejected");
    CameraState degenerate = test_camera();
    degenerate.target = degenerate.position;
    expect(!make_viewport_ray({}, 800, 600, degenerate), "degenerate camera is rejected");
}

void test_nearest_and_hidden_surface_picking() {
    Scene scene;
    expect(scene.add_entity(EntityId{10}, "Far", true, make_plane(0.0)).has_value(), "add far plane");
    expect(scene.add_entity(EntityId{20}, "Near", true, make_plane(2.0)).has_value(), "add near plane");
    const auto ray = make_viewport_ray({400.0f, 300.0f}, 800, 600, test_camera());
    if (!ray) {
        return;
    }
    auto hits = pick_surfaces(scene, *ray, 2);
    expect(hits.size() == 2, "ray hits both overlapping surfaces");
    if (hits.size() == 2) {
        expect(hits[0].entity == EntityId{20}, "nearest surface is first");
        expect(hits[1].entity == EntityId{10}, "farther surface is second");
        expect(hits[0].distance < hits[1].distance, "hits are sorted by distance");
        expect(std::abs(hits[0].position.z - 2.0) < 1e-12, "hit position is on surface");
    }

    expect(scene.set_entity_visibility(EntityId{20}, false).has_value(), "hide near plane");
    hits = pick_surfaces(scene, *ray, 2);
    expect(hits.size() == 1 && hits.front().entity == EntityId{10},
        "hidden surface cannot be picked");

    const auto miss = make_viewport_ray({0.0f, 0.0f}, 800, 600, test_camera());
    expect(miss && pick_surfaces(scene, *miss, 2).empty(), "ray outside surface misses");
    expect(pick_surfaces(scene, *ray, 0).empty(), "invalid sampling count returns no hits");
}

void test_hover_and_selection_cycling() {
    Scene scene;
    expect(scene.add_entity(EntityId{1}, "Far", true, make_plane(0.0)).has_value(), "add cycle far plane");
    expect(scene.add_entity(EntityId{2}, "Near", true, make_plane(2.0)).has_value(), "add cycle near plane");
    OrbitCameraController camera({0.0, 0.0, 10.0}, {0.0, 0.0, 0.0});
    std::optional<EntityId> selected;
    std::optional<EntityId> hovered;
    bool cleared = false;
    SurfaceSelectionTool tool(
        [] { return true; },
        [&selected](EntitySelection selection) { selected = selection.entity; },
        [&hovered](std::optional<EntityId> entity) { hovered = entity; },
        [&cleared] { cleared = true; }
    );
    InputFrameSnapshot input{
        .mouse_position = {400.0f, 300.0f},
        .screen_width = 800,
        .screen_height = 600,
        .modifiers = {}
    };
    tool.process_input(input, camera, scene);
    expect(hovered == EntityId{2}, "hover chooses nearest hit");
    expect(!selected.has_value(), "hover does not select");

    input.left_mouse_pressed = true;
    tool.process_input(input, camera, scene);
    expect(selected == EntityId{2}, "first click selects nearest surface");
    tool.process_input(input, camera, scene);
    expect(selected == EntityId{1}, "second click cycles to overlapping surface");
    tool.process_input(input, camera, scene);
    expect(selected == EntityId{2}, "cycling wraps to nearest surface");

    input.mouse_position = {0.0f, 0.0f};
    tool.process_input(input, camera, scene);
    expect(cleared, "clicking empty space clears selection");
    expect(!hovered.has_value(), "empty space clears hover");
}

} // namespace

int main() {
    test_viewport_ray();
    test_nearest_and_hidden_surface_picking();
    test_hover_and_selection_cycling();

    if (failures != 0) {
        std::cerr << failures << " surface picking test(s) failed\n";
        return 1;
    }
    std::cout << "All surface picking tests passed\n";
    return 0;
}
