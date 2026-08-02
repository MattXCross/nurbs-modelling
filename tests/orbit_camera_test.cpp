#include "orbit_camera.h"

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

} // namespace

int main() {
    test_orbit_preserves_distance();
    test_pan_moves_position_and_target_together();
    test_zoom_changes_distance();
    test_degenerate_initial_view_is_repaired();

    if (failures != 0) {
        std::cerr << failures << " test assertion(s) failed\n";
        return 1;
    }
    return 0;
}
