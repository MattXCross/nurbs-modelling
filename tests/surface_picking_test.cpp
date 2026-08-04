#include "surface_picking.h"

#include "core.h"
#include "input_frame.h"
#include "input_tools.h"
#include "nurbs_surface.h"
#include "orbit_camera.h"
#include "scene.h"
#include "selection.h"
#include "transform_gizmos.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <string_view>
#include <utility>
#include <variant>

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

void test_orthographic_viewport_rays_and_clipping() {
    CameraState camera = test_camera();
    camera.projection = ProjectionMode::orthographic;
    camera.orthographic_vertical_size = 6.0;
    const auto center = make_viewport_ray({400.0f, 300.0f}, 800, 600, camera);
    const auto corner = make_viewport_ray({800.0f, 0.0f}, 800, 600, camera);
    expect(center && corner, "orthographic rays are valid");
    if (center && corner) {
        expect(center->direction() == corner->direction(), "orthographic rays are parallel");
        expect(center->origin() != corner->origin(), "orthographic rays have screen-space origins");
        expect(corner->origin() == cad::Point3{4.0, 3.0, 10.0},
            "orthographic ray origin uses aspect and vertical size");
    }
    cad::Aabb3 visible;
    (void)visible.expand({-1.0, -1.0, -1e6});
    (void)visible.expand({1.0, 1.0, 1.0});
    const ClipPlanes clipping = derive_clip_planes(camera, visible);
    expect(clipping.near_plane > 0.0 && clipping.far_plane > clipping.near_plane,
        "visible bounds produce valid clipping planes");
    expect(clipping.far_plane > 1e6, "large visible bounds remain inside far clipping");
    cad::Aabb3 tiny;
    (void)tiny.expand({-1e-9, -1e-9, -1e-9});
    (void)tiny.expand({1e-9, 1e-9, 1e-9});
    const ClipPlanes tiny_clipping = derive_clip_planes(camera, tiny);
    expect(tiny_clipping.near_plane > 0.0 && tiny_clipping.far_plane > tiny_clipping.near_plane,
        "tiny visible bounds produce valid clipping planes");
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

void test_control_point_rectangle_selection() {
    Scene scene;
    expect(scene.add_entity(EntityId{5}, "Grid", true, make_plane(0.0)).has_value(),
        "add rectangle-selection plane");
    OrbitCameraController camera({0.0, 0.0, 10.0}, {0.0, 0.0, 0.0});
    std::vector<ControlPointSelection> rectangle_selection;
    ModifierKeys rectangle_modifiers;
    ControlPointSelectionTool tool(
        [] { return true; },
        [](ControlPointSelection, ModifierKeys) {},
        [&rectangle_selection, &rectangle_modifiers](
            std::vector<ControlPointSelection> selections,
            ModifierKeys modifiers
        ) {
            rectangle_selection = std::move(selections);
            rectangle_modifiers = modifiers;
        }
    );
    InputFrameSnapshot press{
        .mouse_position = {150.0f, 75.0f},
        .screen_width = 800,
        .screen_height = 600,
        .left_mouse = true,
        .left_mouse_pressed = true,
        .modifiers = {.shift = true}
    };
    tool.process_input(press, camera, scene);
    InputFrameSnapshot release{
        .mouse_position = {650.0f, 525.0f},
        .screen_width = 800,
        .screen_height = 600,
        .left_mouse_released = true,
        .modifiers = {.shift = true}
    };
    tool.process_input(release, camera, scene);
    expect(rectangle_selection.size() == 4, "rectangle selects every enclosed control point");
    expect(rectangle_modifiers.shift, "rectangle preserves additive modifier");
}

void test_translation_gizmo_screen_drag_and_cancel() {
    Scene scene;
    OrbitCameraController camera({0.0, 0.0, 10.0}, {0.0, 0.0, 0.0});
    bool active = false;
    bool finished = false;
    bool canceled = false;
    std::optional<cad::Vector3> preview;
    std::optional<TranslationConstraint> started_constraint;
    TranslationGizmo tool(
        [&active, &started_constraint] {
            return active ? started_constraint : std::nullopt;
        },
        [] { return std::optional{TransformFrame{.pivot = {0.0, 0.0, 0.0}}}; },
        [&active, &started_constraint](TranslationConstraint constraint) {
            active = true;
            started_constraint = constraint;
            return true;
        },
        [&preview](cad::Vector3 delta) {
            preview = delta;
            return true;
        },
        [&active, &finished] {
            active = false;
            finished = true;
        },
        [&active, &canceled] {
            active = false;
            canceled = true;
        }
    );
    InputFrameSnapshot press{
        .mouse_position = {400.0f, 300.0f},
        .screen_width = 800,
        .screen_height = 600,
        .left_mouse = true,
        .left_mouse_pressed = true,
        .modifiers = {}
    };
    expect(tool.process_input(press, camera.camera()),
        "gizmo consumes the press that starts a drag");
    expect(active, "center gizmo handle begins screen-plane drag");
    expect(started_constraint == TranslationConstraint::screen, "center handle uses screen plane");
    InputFrameSnapshot move{
        .mouse_position = {440.0f, 300.0f},
        .screen_width = 800,
        .screen_height = 600,
        .left_mouse = true,
        .modifiers = {.ctrl = true}
    };
    (void)tool.process_input(move, camera.camera());
    expect(preview.has_value(), "gizmo drag produces translation preview");
    if (preview) {
        expect(preview->x == 0.5, "Ctrl snaps screen drag to configured increment");
        expect(preview->y == 0.0 && preview->z == 0.0, "screen drag follows camera plane");
    }
    InputFrameSnapshot release{
        .mouse_position = {440.0f, 300.0f},
        .screen_width = 800,
        .screen_height = 600,
        .left_mouse_released = true,
        .modifiers = {}
    };
    (void)tool.process_input(release, camera.camera());
    expect(finished && !active, "mouse release commits gizmo drag");

    InputFrameSnapshot axis_press = press;
    axis_press.mouse_position = {460.0f, 300.0f};
    (void)tool.process_input(axis_press, camera.camera());
    expect(started_constraint == TranslationConstraint::x, "X axis handle is pickable");
    InputFrameSnapshot axis_move = move;
    axis_move.mouse_position = {500.0f, 300.0f};
    (void)tool.process_input(axis_move, camera.camera());
    expect(preview && preview->x == 0.5 && preview->y == 0.0 && preview->z == 0.0,
        "axis drag is constrained and increment-snapped");
    (void)tool.process_input(release, camera.camera());

    InputFrameSnapshot plane_press = press;
    plane_press.mouse_position = {422.0f, 278.0f};
    (void)tool.process_input(plane_press, camera.camera());
    expect(started_constraint == TranslationConstraint::xy, "XY plane handle is pickable");
    InputFrameSnapshot plane_move = move;
    plane_move.mouse_position = {442.0f, 268.0f};
    plane_move.modifiers = {};
    (void)tool.process_input(plane_move, camera.camera());
    expect(preview && preview->x > 0.0 && preview->y > 0.0 && preview->z == 0.0,
        "plane drag remains in selected plane");
    (void)tool.process_input(release, camera.camera());

    finished = false;
    (void)tool.process_input(press, camera.camera());
    InputFrameSnapshot escape{.escape_pressed = true, .modifiers = {}};
    (void)tool.process_input(escape, camera.camera());
    expect(canceled && !active && !finished, "Escape cancels gizmo drag");
}

void test_rotation_and_scale_gizmo_controls() {
    Scene scene;
    OrbitCameraController camera({0.0, 0.0, 10.0}, {0.0, 0.0, 0.0});
    bool rotation_active = false;
    std::optional<RotationConstraint> rotation_constraint;
    std::optional<double> angle;
    RotationGizmo rotation(
        [&rotation_active, &rotation_constraint] {
            return rotation_active ? rotation_constraint : std::nullopt;
        },
        [] { return std::optional{TransformFrame{.pivot = {0, 0, 0}}}; },
        [&rotation_active, &rotation_constraint](RotationConstraint constraint, cad::Vector3) {
            rotation_active = true;
            rotation_constraint = constraint;
            return true;
        },
        [&angle](double value) { angle = value; return true; },
        [&rotation_active] { rotation_active = false; },
        [&rotation_active] { rotation_active = false; }
    );
    InputFrameSnapshot ring_press{
        .mouse_position = {494.0f, 300.0f},
        .screen_width = 800,
        .screen_height = 600,
        .left_mouse = true,
        .left_mouse_pressed = true,
        .modifiers = {}
    };
    (void)rotation.process_input(ring_press, camera.camera());
    expect(rotation_constraint == RotationConstraint::screen,
        "outer rotation ring selects camera-plane rotation");
    InputFrameSnapshot ring_move{
        .mouse_position = {400.0f, 394.0f},
        .screen_width = 800,
        .screen_height = 600,
        .left_mouse = true,
        .modifiers = {.ctrl = true}
    };
    (void)rotation.process_input(ring_move, camera.camera());
    expect(angle && std::abs(*angle - std::numbers::pi / 2.0) < 1e-12,
        "rotation ring drag snaps to 15-degree increments");

    bool scale_active = false;
    std::optional<ScaleConstraint> scale_constraint;
    std::optional<double> factor;
    ScaleGizmo scale(
        [&scale_active, &scale_constraint] {
            return scale_active ? scale_constraint : std::nullopt;
        },
        [] { return std::optional{TransformFrame{.pivot = {0, 0, 0}}}; },
        [&scale_active, &scale_constraint](ScaleConstraint constraint) {
            scale_active = true;
            scale_constraint = constraint;
            return true;
        },
        [&factor](double value) { factor = value; return true; },
        [&scale_active] { scale_active = false; },
        [&scale_active] { scale_active = false; }
    );
    InputFrameSnapshot scale_press{
        .mouse_position = {460.0f, 300.0f},
        .screen_width = 800,
        .screen_height = 600,
        .left_mouse = true,
        .left_mouse_pressed = true,
        .modifiers = {}
    };
    (void)scale.process_input(scale_press, camera.camera());
    expect(scale_constraint == ScaleConstraint::x, "box-ended X scale handle is pickable");
    InputFrameSnapshot scale_move{
        .mouse_position = {500.0f, 300.0f},
        .screen_width = 800,
        .screen_height = 600,
        .left_mouse = true,
        .modifiers = {.ctrl = true}
    };
    (void)scale.process_input(scale_move, camera.camera());
    expect(factor && *factor == 1.5, "axis scale drag snaps factor to tenths");
}

void test_translation_uses_oriented_frame() {
    Scene scene;
    OrbitCameraController camera({0.0, 0.0, 10.0}, {0.0, 0.0, 0.0});
    bool active = false;
    std::optional<cad::Vector3> preview;
    const TransformFrame frame{
        .pivot = {0, 0, 0},
        .x = {0, 1, 0},
        .y = {1, 0, 0},
        .z = {0, 0, -1}
    };
    TranslationGizmo tool(
        [&active] {
            return active
                ? std::optional{TranslationConstraint::x}
                : std::nullopt;
        },
        [frame] { return std::optional{frame}; },
        [&active](TranslationConstraint constraint) {
            active = constraint == TranslationConstraint::x;
            return active;
        },
        [&preview](cad::Vector3 delta) { preview = delta; return true; },
        [&active] { active = false; },
        [&active] { active = false; }
    );
    InputFrameSnapshot press{
        .mouse_position = {400.0f, 240.0f},
        .screen_width = 800,
        .screen_height = 600,
        .left_mouse = true,
        .left_mouse_pressed = true,
        .modifiers = {}
    };
    (void)tool.process_input(press, camera.camera());
    expect(active, "rotated local X handle is pickable");
    InputFrameSnapshot move{
        .mouse_position = {400.0f, 200.0f},
        .screen_width = 800,
        .screen_height = 600,
        .left_mouse = true,
        .modifiers = {.ctrl = true}
    };
    (void)tool.process_input(move, camera.camera());
    expect(preview && preview->x == 0.0 && preview->y == 0.5 && preview->z == 0.0,
        "local X drag follows oriented axis and snaps in local coordinates");
}

void test_gizmo_draw_data() {
    const TransformFrame frame{.pivot = {1.0, 2.0, 3.0}};
    std::optional<TranslationConstraint> active = TranslationConstraint::x;
    TranslationGizmo translation(
        [&active] { return active; },
        [frame] { return std::optional{frame}; },
        [](TranslationConstraint) { return true; },
        [](cad::Vector3) { return true; },
        [] {},
        [] {}
    );
    GizmoDrawList draw_list;
    translation.append_draw_data(draw_list, test_camera(), 600);
    expect(draw_list.size() == 7, "translation gizmo emits center, arrows, and planes");
    expect(std::holds_alternative<GizmoCenterHandle>(draw_list[0]),
        "translation draw list begins with a center handle");
    expect(std::holds_alternative<GizmoArrow>(draw_list[1]),
        "translation draw list contains backend-neutral arrows");
    expect(std::holds_alternative<GizmoPlaneHandle>(draw_list[4]),
        "translation draw list contains backend-neutral plane handles");
    if (const auto* arrow = std::get_if<GizmoArrow>(&draw_list[1])) {
        expect(arrow->color.red == 253 && arrow->color.green == 249,
            "active constraint is highlighted in draw data");
    }

    RotationGizmo rotation(
        [] { return std::optional<RotationConstraint>{}; },
        [frame] { return std::optional{frame}; },
        [](RotationConstraint, cad::Vector3) { return true; },
        [](double) { return true; },
        [] {},
        [] {}
    );
    draw_list.clear();
    rotation.append_draw_data(draw_list, test_camera(), 600);
    expect(draw_list.size() == 4 &&
           std::ranges::all_of(draw_list, [](const GizmoPrimitive& primitive) {
               return std::holds_alternative<GizmoRing>(primitive);
           }), "rotation gizmo emits three axis rings and a camera ring");

    ScaleGizmo scale(
        [] { return std::optional<ScaleConstraint>{}; },
        [frame] { return std::optional{frame}; },
        [](ScaleConstraint) { return true; },
        [](double) { return true; },
        [] {},
        [] {}
    );
    draw_list.clear();
    scale.append_draw_data(draw_list, test_camera(), 600);
    expect(draw_list.size() == 7 && std::holds_alternative<GizmoCenterHandle>(draw_list[0]),
        "scale gizmo emits center, stems, and box endpoints");
}

} // namespace

int main() {
    test_viewport_ray();
    test_orthographic_viewport_rays_and_clipping();
    test_nearest_and_hidden_surface_picking();
    test_hover_and_selection_cycling();
    test_control_point_rectangle_selection();
    test_translation_gizmo_screen_drag_and_cancel();
    test_rotation_and_scale_gizmo_controls();
    test_translation_uses_oriented_frame();
    test_gizmo_draw_data();

    if (failures != 0) {
        std::cerr << failures << " surface picking test(s) failed\n";
        return 1;
    }
    std::cout << "All surface picking tests passed\n";
    return 0;
}
