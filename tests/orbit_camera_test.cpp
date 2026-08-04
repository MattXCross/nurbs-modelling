#include "orbit_camera.h"

#include <array>
#include <cmath>
#include <iostream>
#include <string_view>

namespace {

int failures = 0;

void expect(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

bool nearly_equal(double left, double right, double tolerance = 1e-12) {
    return std::abs(left - right) <= tolerance;
}

bool nearly_equal(cad::Vector3 left, cad::Vector3 right, double tolerance = 1e-12) {
    return nearly_equal(left.x, right.x, tolerance) &&
        nearly_equal(left.y, right.y, tolerance) &&
        nearly_equal(left.z, right.z, tolerance);
}

void test_orbit_preserves_distance() {
    OrbitCameraController controller({0.0, 0.0, 10.0}, {0.0, 0.0, 0.0});
    controller.orbit({20.0f, -10.0f});

    const CameraState& camera = controller.camera();
    expect(nearly_equal(cad::distance(camera.position, camera.target), 10.0),
           "orbit preserves camera distance");
    expect(camera.target == cad::Point3{}, "orbit preserves target");
}

void test_pan_moves_position_and_target_together() {
    OrbitCameraController controller({0.0, 0.0, 10.0}, {0.0, 0.0, 0.0});
    const cad::Vector3 initial_offset =
        controller.camera().position - controller.camera().target;

    controller.pan({10.0f, 5.0f}, 800.0f);

    const CameraState& camera = controller.camera();
    expect(camera.target != cad::Point3{}, "pan moves target");
    expect(camera.position - camera.target == initial_offset,
           "pan preserves camera-to-target offset");
}

void test_zoom_changes_distance() {
    OrbitCameraController controller({0.0, 0.0, 10.0}, {0.0, 0.0, 0.0});
    controller.zoom(1.0f);
    expect(nearly_equal(cad::distance(controller.camera().position, controller.camera().target), 9.0),
           "positive wheel movement zooms in");
}

void test_degenerate_initial_view_is_repaired() {
    OrbitCameraController controller({1.0, 2.0, 3.0}, {1.0, 2.0, 3.0});
    const CameraState& camera = controller.camera();
    expect(nearly_equal(cad::distance(camera.position, camera.target), 1.0),
           "degenerate initial view uses minimum distance");
    expect(cad::is_finite(camera.position), "repaired camera position is finite");
}

cad::Aabb3 bounds(cad::Point3 minimum, cad::Point3 maximum) {
    cad::Aabb3 result;
    (void)result.expand(minimum);
    (void)result.expand(maximum);
    return result;
}

void test_standard_views() {
    OrbitCameraController controller({0.0, 0.0, 10.0}, {});
    const std::array views{
        std::pair{StandardView::front, cad::Vector3{0.0, 0.0, 1.0}},
        std::pair{StandardView::back, cad::Vector3{0.0, 0.0, -1.0}},
        std::pair{StandardView::top, cad::Vector3{0.0, 1.0, 0.0}},
        std::pair{StandardView::bottom, cad::Vector3{0.0, -1.0, 0.0}},
        std::pair{StandardView::left, cad::Vector3{-1.0, 0.0, 0.0}},
        std::pair{StandardView::right, cad::Vector3{1.0, 0.0, 0.0}}
    };
    for (const auto& [view, expected] : views) {
        controller.set_standard_view(view);
        const auto direction = cad::normalized(
            controller.camera().position - controller.camera().target
        );
        expect(direction && nearly_equal(*direction, expected),
            "standard view has predictable direction");
        expect(cad::normalized(cad::cross(
            controller.camera().target - controller.camera().position,
            controller.camera().up
        )).has_value(), "standard view has a valid up vector");
    }
}

void test_fit_bounds_at_model_scales() {
    OrbitCameraController controller({10.0, 10.0, 10.0}, {});
    expect(controller.frame_bounds(bounds({-1e-9, -2e-9, -3e-9}, {1e-9, 2e-9, 3e-9}), 16.0 / 9.0),
        "tiny bounds can be framed");
    expect(cad::distance(controller.camera().position, controller.camera().target) < 1e-6,
        "tiny fit uses a tiny camera distance");
    expect(controller.frame_bounds(bounds({-1e9, -2e9, -3e9}, {1e9, 2e9, 3e9}), 16.0 / 9.0),
        "large bounds can be framed");
    expect(cad::distance(controller.camera().position, controller.camera().target) > 1e9,
        "large fit uses a large camera distance");
    expect(controller.frame_bounds(bounds({4.0, 5.0, 6.0}, {4.0, 5.0, 6.0}), 1.0),
        "degenerate bounds can be framed");
    expect(controller.camera().target == cad::Point3{4.0, 5.0, 6.0},
        "degenerate fit still centers its point");
    expect(!controller.frame_bounds(cad::Aabb3{}, 1.0), "empty bounds are rejected");
}

void test_projection_and_orthographic_fit() {
    OrbitCameraController controller({0.0, 0.0, 10.0}, {});
    controller.set_projection(ProjectionMode::orthographic);
    expect(controller.camera().projection == ProjectionMode::orthographic,
        "projection switches to orthographic");
    expect(controller.frame_bounds(bounds({-4.0, -2.0, 0.0}, {4.0, 2.0, 0.0}), 2.0),
        "orthographic bounds can be framed");
    expect(nearly_equal(controller.camera().orthographic_vertical_size, 4.4),
        "orthographic fit respects aspect and margin");
    controller.set_projection(ProjectionMode::perspective);
    expect(controller.camera().projection == ProjectionMode::perspective,
        "projection switches back to perspective");
}

void test_cursor_zoom_and_large_delta_clamp() {
    OrbitCameraController controller({0.0, 0.0, 10.0}, {});
    constexpr double tangent = 0.4142135623730950488;
    const double point_before = 10.0 * tangent * 4.0 / 3.0;
    controller.zoom(1.0f, {800.0f, 300.0f}, 800, 600);
    const double point_after = controller.camera().target.x + 9.0 * tangent * 4.0 / 3.0;
    expect(nearly_equal(point_before, point_after, 1e-10),
        "perspective zoom preserves cursor point on target plane");
    const double distance_before = cad::distance(controller.camera().position, controller.camera().target);
    controller.zoom(1e9f, {400.0f, 300.0f}, 800, 600);
    const double distance_after = cad::distance(controller.camera().position, controller.camera().target);
    expect(distance_after >= distance_before * 0.19 && distance_after > 0.0,
        "large wheel delta cannot collapse or invert distance");

    controller.set_projection(ProjectionMode::orthographic);
    const double size_before = controller.camera().orthographic_vertical_size;
    const double target_before = controller.camera().target.x;
    controller.zoom(1.0f, {800.0f, 300.0f}, 800, 600);
    expect(nearly_equal(controller.camera().orthographic_vertical_size, size_before * 0.9),
        "orthographic wheel changes view size");
    expect(nearly_equal(
        controller.camera().target.x + controller.camera().orthographic_vertical_size * 2.0 / 3.0,
        target_before + size_before * 2.0 / 3.0,
        1e-10
    ), "orthographic zoom preserves cursor point");
    const double orthographic_size = controller.camera().orthographic_vertical_size;
    controller.zoom(1e9f, {400.0f, 300.0f}, 800, 600);
    expect(controller.camera().orthographic_vertical_size >= orthographic_size * 0.19,
        "large wheel delta cannot collapse orthographic size");
}

} // namespace

int main() {
    test_orbit_preserves_distance();
    test_pan_moves_position_and_target_together();
    test_zoom_changes_distance();
    test_degenerate_initial_view_is_repaired();
    test_standard_views();
    test_fit_bounds_at_model_scales();
    test_projection_and_orthographic_fit();
    test_cursor_zoom_and_large_delta_clamp();

    if (failures != 0) {
        std::cerr << failures << " test assertion(s) failed\n";
        return 1;
    }
    return 0;
}
