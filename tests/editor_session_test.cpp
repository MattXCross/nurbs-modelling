#include "editor_session.h"

#include "core.h"
#include "nurbs_surface.h"
#include "scene.h"
#include "selection.h"

#include <iostream>
#include <limits>
#include <numbers>
#include <memory>
#include <string_view>
#include <utility>
#include <vector>

namespace {

int failures = 0;

void expect(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

std::unique_ptr<NurbsSurface> make_surface(double x_offset = 0.0) {
    auto surface = NurbsSurface::create(2, 2, 1, 1, {
        {{x_offset, 0.0, 0.0}, 1.0},
        {{x_offset, 0.0, 1.0}, 1.0},
        {{x_offset + 1.0, 0.0, 0.0}, 1.0},
        {{x_offset + 1.0, 0.0, 1.0}, 1.0}
    });
    expect(surface.has_value(), "construct editor test surface");
    return surface ? std::move(*surface) : nullptr;
}

void test_commit_pending_edit() {
    EditorSession session;
    const SceneNode& node = session.scene().nodes().front();
    const ControlPointSelection selection{node.id, 0, 0};
    expect(
        session.set_selection_mode(EditorSession::SelectionMode::control_point),
        "enter control-point mode"
    );
    expect(session.select_control_point(selection), "select control point");
    expect(
        session.begin_control_point_edit(EditorSession::ControlPointField::position_x),
        "begin inspector edit"
    );
    expect(
        session.preview_control_point_edit(EditorSession::ControlPointField::position_x, 12.5),
        "preview inspector edit"
    );
    expect(session.is_dirty(), "pending preview is dirty");
    session.commit_pending_edit();
    expect(session.can_undo(), "committed preview creates undo entry");
    expect(session.undo_description() == "Edit X Position", "inspector entry is named");
    expect(session.undo(), "undo committed inspector edit");
    expect(session.selected_control_point() != nullptr, "selection remains valid after undo");
    if (session.selected_control_point() != nullptr) {
        expect(session.selected_control_point()->position.x == -3.0, "undo restores exact value");
    }
}

void test_selection_modes() {
    EditorSession session;
    const EntityId entity = session.scene().nodes().front().id;
    const ControlPointSelection point{entity, 1, 1};
    expect(
        session.selection_mode() == EditorSession::SelectionMode::object,
        "object mode is the default"
    );
    expect(!session.select_control_point(point), "object mode rejects control-point selection");
    expect(
        session.set_selection_mode(EditorSession::SelectionMode::control_point),
        "switch to control-point mode"
    );
    expect(session.select_control_point(point), "control-point mode accepts point selection");
    expect(
        session.set_selection_mode(EditorSession::SelectionMode::object),
        "switch to object mode"
    );
    expect(session.selection().control_point() == nullptr, "object mode removes point selection");
    expect(
        session.selection().entity() != nullptr &&
            session.selection().entity()->entity == entity,
        "object mode promotes point selection to its owner"
    );
    expect(
        session.set_selection_mode(EditorSession::SelectionMode::control_point),
        "return to control-point mode"
    );
    expect(
        session.selection().entity() != nullptr &&
            session.selection().entity()->entity == entity,
        "entity remains the control-point editing context"
    );
}

void test_mode_switch_cancels_preview() {
    EditorSession session;
    const EntityId entity = session.scene().nodes().front().id;
    const ControlPointSelection point{entity, 0, 0};
    (void)session.set_selection_mode(EditorSession::SelectionMode::control_point);
    expect(session.select_control_point(point), "select preview point");
    expect(
        session.begin_control_point_edit(EditorSession::ControlPointField::position_y),
        "begin preview before mode switch"
    );
    expect(
        session.preview_control_point_edit(EditorSession::ControlPointField::position_y, 99.0),
        "apply preview before mode switch"
    );
    expect(
        session.set_selection_mode(EditorSession::SelectionMode::object),
        "switch mode during preview"
    );
    const ControlPoint* restored = session.scene().resolve(point);
    expect(restored != nullptr && restored->position.y == 0.0, "mode switch cancels preview");
    expect(!session.can_undo(), "canceled preview creates no history entry");
}

void test_multi_selection_operations() {
    EditorSession session;
    const EntityId entity = session.scene().nodes().front().id;
    (void)session.set_selection_mode(EditorSession::SelectionMode::control_point);
    const ControlPointSelection first{entity, 0, 0};
    const ControlPointSelection second{entity, 1, 1};
    const ControlPointSelection third{entity, 2, 2};
    expect(session.select_control_point(first), "replace control-point selection");
    expect(
        session.select_control_point(second, EditorSession::SelectionOperation::add),
        "add control point with Shift semantics"
    );
    expect(session.selection().control_points().size() == 2, "two points are selected");
    expect(
        session.select_control_point(first, EditorSession::SelectionOperation::toggle),
        "toggle selected point with Ctrl semantics"
    );
    expect(
        session.selection().control_points().size() == 1 &&
            session.selection().control_point() != nullptr &&
            *session.selection().control_point() == second,
        "toggle removes an existing point"
    );
    expect(
        session.select_control_point(third, EditorSession::SelectionOperation::toggle),
        "toggle adds an unselected point"
    );
    expect(session.selection().control_points().size() == 2, "toggle adds to selection");

    expect(session.select_control_point_row(), "select primary control-point row");
    expect(session.selection().control_points().size() == 3, "row selects all V points");
    expect(session.select_control_point_column(), "select primary control-point column");
    expect(session.selection().control_points().size() == 3, "column selects all U points");
    expect(session.select_all_control_points(), "select all control points");
    expect(session.selection().control_points().size() == 9, "select all covers control net");
    expect(session.shrink_control_point_selection(), "shrink full control net");
    expect(session.selection().control_points().size() == 1, "shrink keeps interior point");
    expect(session.grow_control_point_selection(), "grow center control point");
    expect(session.selection().control_points().size() == 5, "grow adds orthogonal neighbors");
}

void test_multi_point_edit_is_atomic() {
    EditorSession session;
    const EntityId entity = session.scene().nodes().front().id;
    (void)session.set_selection_mode(EditorSession::SelectionMode::control_point);
    const ControlPointSelection first{entity, 0, 0};
    const ControlPointSelection second{entity, 1, 1};
    expect(session.select_control_points({first, second}), "select points for atomic edit");
    const double first_initial = session.scene().resolve(first)->position.x;
    const double second_initial = session.scene().resolve(second)->position.x;
    expect(
        session.begin_control_point_edit(EditorSession::ControlPointField::position_x),
        "begin multi-point edit"
    );
    expect(
        session.preview_control_point_edit(EditorSession::ControlPointField::position_x, 7.25),
        "preview multi-point edit"
    );
    session.finish_control_point_edit(EditorSession::ControlPointField::position_x);
    expect(session.scene().resolve(first)->position.x == 7.25, "edit changes first point");
    expect(session.scene().resolve(second)->position.x == 7.25, "edit changes second point");
    expect(session.undo_description() == "Edit X Position", "multi-point edit has one name");
    expect(session.undo(), "undo multi-point edit once");
    expect(session.scene().resolve(first)->position.x == first_initial, "undo restores first point");
    expect(session.scene().resolve(second)->position.x == second_initial, "undo restores second point");
    expect(session.redo(), "redo multi-point edit once");
    expect(session.scene().resolve(first)->position.x == 7.25, "redo changes first point");
    expect(session.scene().resolve(second)->position.x == 7.25, "redo changes second point");
}

void test_control_point_translation_transaction() {
    EditorSession session;
    const EntityId entity = session.scene().nodes().front().id;
    (void)session.set_selection_mode(EditorSession::SelectionMode::control_point);
    const ControlPointSelection first{entity, 0, 0};
    const ControlPointSelection second{entity, 1, 1};
    expect(session.select_control_points({first, second}), "select translation points");
    const Point3D first_initial = session.scene().resolve(first)->position;
    const Point3D second_initial = session.scene().resolve(second)->position;
    expect(
        session.begin_translation(EditorSession::TranslationConstraint::xy),
        "begin point translation"
    );
    expect(session.preview_translation({2.0, -3.0, 0.0}), "preview point translation");
    expect(session.scene().resolve(first)->position == first_initial + cad::Vector3{2.0, -3.0, 0.0},
        "preview moves first point");
    expect(session.scene().resolve(second)->position == second_initial + cad::Vector3{2.0, -3.0, 0.0},
        "preview moves second point");
    expect(session.cancel_translation(), "cancel point translation");
    expect(session.scene().resolve(first)->position == first_initial, "cancel restores first point");
    expect(session.scene().resolve(second)->position == second_initial, "cancel restores second point");
    expect(!session.can_undo(), "canceled translation creates no history entry");

    expect(session.translate_selection({1.5, 2.5, -4.0}), "apply numeric translation");
    expect(session.undo_description() == "Translate Selection", "translation is named");
    expect(session.undo(), "undo translation once");
    expect(session.scene().resolve(first)->position == first_initial, "undo restores first point");
    expect(session.scene().resolve(second)->position == second_initial, "undo restores second point");
    expect(session.redo(), "redo translation once");
    expect(session.scene().resolve(first)->position == first_initial + cad::Vector3{1.5, 2.5, -4.0},
        "redo moves first point");
    expect(session.scene().resolve(second)->position == second_initial + cad::Vector3{1.5, 2.5, -4.0},
        "redo moves second point");
}

void test_object_translation_and_invalid_delta() {
    EditorSession session;
    const EntityId entity = session.scene().nodes().front().id;
    expect(session.select_entity(EntitySelection{entity}), "select object for translation");
    const ControlPointSelection first{entity, 0, 0};
    const ControlPointSelection last{entity, 2, 2};
    const Point3D first_initial = session.scene().resolve(first)->position;
    const Point3D last_initial = session.scene().resolve(last)->position;
    const auto pivot = session.selection_pivot();
    expect(pivot.has_value(), "object selection has translation pivot");
    const std::uint64_t initial_revision = session.scene().nodes().front().geometry_revision;
    expect(
        session.begin_translation(EditorSession::TranslationConstraint::y),
        "begin GPU object translation preview"
    );
    expect(session.preview_translation({0.0, 2.0, 0.0}), "preview GPU object translation");
    expect(session.scene().resolve(first)->position == first_initial,
        "object preview leaves exact geometry unchanged");
    expect(session.scene().nodes().front().geometry_revision == initial_revision,
        "object preview preserves geometry revision");
    expect(!session.interactive_geometry_edit(),
        "object preview keeps the full-quality mesh cache active");
    const auto preview = session.entity_transform_preview();
    expect(preview && preview->entity == entity, "object preview exposes its entity transform");
    if (preview) {
        expect(preview->transform.transform_point(first_initial) ==
                first_initial + cad::Vector3{0.0, 2.0, 0.0},
            "object preview transform maps the cached mesh position");
    }
    expect(session.selection_pivot() == *pivot + cad::Vector3{0.0, 2.0, 0.0},
        "object preview moves the displayed gizmo pivot");
    expect(session.cancel_translation(), "cancel GPU object translation preview");
    expect(!session.entity_transform_preview(), "cancel clears object preview transform");
    expect(session.scene().nodes().front().geometry_revision == initial_revision,
        "canceling object preview does not invalidate geometry");
    expect(!session.can_undo(), "canceling object preview creates no history entry");

    expect(session.translate_selection({0.0, 1.0, 0.0}), "translate whole object");
    expect(session.scene().resolve(first)->position == first_initial + cad::Vector3{0.0, 1.0, 0.0},
        "object translation moves first control point");
    expect(session.scene().resolve(last)->position == last_initial + cad::Vector3{0.0, 1.0, 0.0},
        "object translation moves last control point");
    expect(session.undo(), "undo object translation");
    expect(session.scene().resolve(first)->position == first_initial, "object undo is exact");

    expect(
        !session.translate_selection({std::numeric_limits<double>::infinity(), 0.0, 0.0}),
        "non-finite translation is rejected"
    );
    expect(session.scene().resolve(first)->position == first_initial,
        "invalid translation leaves geometry unchanged");
}

void test_rotation_and_scale_transactions() {
    EditorSession session;
    Scene scene;
    expect(scene.add_entity(EntityId{8}, "Transform Plane", true, make_surface()).has_value(),
        "construct transform scene");
    session.replace_document(std::move(scene));
    const EntityId entity{8};
    expect(session.select_entity(EntitySelection{entity}), "select transform object");
    const ControlPointSelection corner{entity, 0, 0};
    const Point3D initial = session.scene().resolve(corner)->position;
    const std::uint64_t rotation_revision = session.scene().nodes().front().geometry_revision;
    expect(session.begin_rotation(
        EditorSession::RotationConstraint::y, {0.0, 1.0, 0.0}), "begin object rotation");
    expect(session.preview_rotation(std::numbers::pi / 2.0), "preview object rotation");
    expect(session.scene().resolve(corner)->position == initial,
        "rotation preview leaves persistent object geometry unchanged");
    expect(session.scene().nodes().front().geometry_revision == rotation_revision,
        "rotation preview reuses the cached mesh revision");
    const auto rotation_preview = session.entity_transform_preview();
    expect(rotation_preview.has_value(), "rotation exposes a GPU preview transform");
    expect(session.finish_rotation(), "commit object rotation");
    const Point3D rotated = session.scene().resolve(corner)->position;
    expect(std::abs(rotated.x - 0.0) < 1e-12, "rotation preserves expected X");
    expect(std::abs(rotated.z - 1.0) < 1e-12, "rotation produces expected Z");
    expect(session.undo_description() == "Rotate Selection", "rotation is named");
    expect(session.undo(), "undo rotation");
    expect(session.scene().resolve(corner)->position == initial, "rotation undo is exact");
    expect(session.redo(), "redo rotation");
    expect(session.undo(), "return to initial before scale");

    (void)session.set_selection_mode(EditorSession::SelectionMode::control_point);
    const ControlPointSelection primary{entity, 0, 0};
    const ControlPointSelection other{entity, 1, 1};
    expect(session.select_control_points({other, primary}), "select points with primary pivot");
    expect(
        session.set_pivot_mode(EditorSession::PivotMode::primary_control_point),
        "use primary point pivot"
    );
    const Point3D primary_initial = session.scene().resolve(primary)->position;
    const Point3D other_initial = session.scene().resolve(other)->position;
    expect(
        session.scale_selection(EditorSession::ScaleConstraint::uniform, 2.0),
        "uniformly scale point selection"
    );
    expect(session.scene().resolve(primary)->position == primary_initial,
        "primary pivot remains fixed while scaling");
    expect(
        session.scene().resolve(other)->position ==
            primary_initial + (other_initial - primary_initial) * 2.0,
        "uniform scale doubles offset from pivot"
    );
    expect(session.undo_description() == "Scale Selection", "scale is named");
    expect(session.undo(), "undo scale");
    expect(session.scene().resolve(other)->position == other_initial, "scale undo is exact");

    expect(session.set_pivot_mode(EditorSession::PivotMode::world_origin), "use world pivot");
    expect(session.selection_pivot() == cad::Point3{}, "world-origin pivot is exact");
    expect(
        session.scale_selection(EditorSession::ScaleConstraint::x, 3.0),
        "axis-constrained scale"
    );
    expect(session.scene().resolve(other)->position.x == other_initial.x * 3.0,
        "X scale changes only X offset");
    expect(session.scene().resolve(other)->position.y == other_initial.y &&
        session.scene().resolve(other)->position.z == other_initial.z,
        "X scale preserves Y and Z");
}

void test_rotation_and_scale_cancel() {
    EditorSession session;
    const EntityId entity = session.scene().nodes().front().id;
    (void)session.set_selection_mode(EditorSession::SelectionMode::control_point);
    const ControlPointSelection point{entity, 0, 0};
    expect(session.select_control_point(point), "select cancel-transform point");
    const Point3D initial = session.scene().resolve(point)->position;
    expect(session.begin_rotation(EditorSession::RotationConstraint::z, {0, 0, 1}),
        "begin rotation preview");
    expect(session.preview_rotation(0.75), "preview rotation");
    expect(session.cancel_rotation(), "cancel rotation preview");
    expect(session.scene().resolve(point)->position == initial, "rotation cancel is exact");
    expect(session.begin_scale(EditorSession::ScaleConstraint::uniform), "begin scale preview");
    expect(session.preview_scale(1.75), "preview scale");
    expect(session.cancel_scale(), "cancel scale preview");
    expect(session.scene().resolve(point)->position == initial, "scale cancel is exact");
}

void test_local_transform_frame() {
    EditorSession session;
    Scene scene;
    expect(scene.add_entity(EntityId{12}, "Local Plane", true, make_surface()).has_value(),
        "construct local-frame scene");
    session.replace_document(std::move(scene));
    expect(session.select_entity(EntitySelection{EntityId{12}}), "select local-frame object");
    expect(
        session.set_transform_orientation(EditorSession::TransformOrientation::local),
        "switch to local transform orientation"
    );
    const auto frame = session.transform_frame();
    expect(frame.has_value(), "derive local surface frame");
    if (!frame) {
        return;
    }
    expect(frame->x == cad::Vector3{1.0, 0.0, 0.0}, "local X follows surface U");
    expect(frame->y == cad::Vector3{0.0, 0.0, 1.0}, "local Y follows surface V");
    expect(frame->z == cad::Vector3{0.0, -1.0, 0.0}, "local Z completes right-handed frame");

    const ControlPointSelection corner{EntityId{12}, 0, 0};
    const Point3D initial = session.scene().resolve(corner)->position;
    expect(
        session.scale_selection(EditorSession::ScaleConstraint::y, 2.0),
        "scale along local Y"
    );
    const Point3D scaled = session.scene().resolve(corner)->position;
    expect(scaled.x == initial.x && scaled.y == initial.y, "local Y scale preserves world X/Y");
    expect(scaled.z == -0.5, "local Y scale changes surface-V world coordinate");
    expect(session.undo(), "undo local scale");
    expect(session.scene().resolve(corner)->position == initial, "local scale undo is exact");

    expect(
        session.set_transform_orientation(EditorSession::TransformOrientation::world),
        "switch back to world orientation"
    );
    const auto world = session.transform_frame();
    expect(world && world->x == cad::Vector3{1, 0, 0} &&
        world->y == cad::Vector3{0, 1, 0} && world->z == cad::Vector3{0, 0, 1},
        "world orientation restores canonical axes");
}

void test_replace_document_clears_transient_state() {
    EditorSession session;
    const EntityId old_id = session.scene().nodes().front().id;
    expect(session.select_entity(EntitySelection{old_id}), "select old entity");
    expect(session.rename_entity(old_id, "Changed").has_value(), "mutate old document");
    expect(session.is_dirty(), "old document is dirty");

    Scene replacement;
    expect(
        replacement.add_entity(EntityId{77}, "Loaded", false, make_surface(20.0)).has_value(),
        "construct replacement scene"
    );
    session.replace_document(std::move(replacement));

    expect(session.scene().nodes().size() == 1, "replacement scene installed");
    expect(session.scene().nodes().front().id == EntityId{77}, "replacement keeps entity ID");
    expect(!session.scene().nodes().front().visible, "replacement keeps visibility");
    expect(session.selection().empty(), "replacement clears selection");
    expect(!session.can_undo() && !session.can_redo(), "replacement clears history");
    expect(!session.is_dirty(), "replacement establishes clean saved state");
}

void test_new_blank_document() {
    EditorSession session;
    session.replace_document(Scene{});
    expect(session.scene().nodes().empty(), "new document is blank");
    expect(!session.is_dirty(), "new document starts clean");
}

void test_camera_framing_brokerage() {
    EditorSession session;
    Scene scene;
    const auto visible = scene.add_entity("Visible", make_surface(0.0));
    const auto hidden = scene.add_entity("Hidden", make_surface(100.0));
    expect(visible && hidden, "create framing scene");
    if (!visible || !hidden) return;
    expect(scene.set_entity_visibility(*hidden, false).has_value(), "hide framing entity");
    session.replace_document(std::move(scene));

    int camera_notifications = 0;
    session.set_change_handler([&camera_notifications](EditorChange change) {
        if (change.camera) ++camera_notifications;
    });
    expect(session.fit_all(800, 600), "session fits visible geometry");
    expect(session.camera().target == cad::Point3{0.5, 0.0, 0.5},
        "fit-all excludes hidden geometry");
    expect(session.select_entity(EntitySelection{*hidden}), "select hidden entity for framing");
    expect(session.frame_selection(800, 600), "session frames selected object");
    expect(session.camera().target == cad::Point3{100.5, 0.0, 0.5},
        "object framing uses selected bounds");

    expect(session.set_selection_mode(EditorSession::SelectionMode::control_point),
        "enter point mode for point framing");
    expect(session.select_control_points({{*visible, 0, 0}, {*visible, 1, 1}}),
        "select points for framing");
    expect(session.frame_selection(800, 600), "session frames selected points");
    expect(session.camera().target == cad::Point3{0.5, 0.0, 0.5},
        "point framing uses only selected positions");
    expect(session.set_camera_projection(ProjectionMode::orthographic),
        "session switches projection");
    expect(!session.set_camera_projection(ProjectionMode::orthographic),
        "unchanged projection is ignored");
    session.set_standard_view(StandardView::top);
    expect(camera_notifications == 5, "camera commands publish camera notifications");

    session.replace_document(Scene{});
    expect(!session.fit_all(800, 600), "empty scene cannot be fitted");
    expect(!session.frame_selection(800, 600), "empty selection cannot be framed");
}

} // namespace

int main() {
    test_commit_pending_edit();
    test_selection_modes();
    test_mode_switch_cancels_preview();
    test_multi_selection_operations();
    test_multi_point_edit_is_atomic();
    test_control_point_translation_transaction();
    test_object_translation_and_invalid_delta();
    test_rotation_and_scale_transactions();
    test_rotation_and_scale_cancel();
    test_local_transform_frame();
    test_replace_document_clears_transient_state();
    test_new_blank_document();
    test_camera_framing_brokerage();

    if (failures != 0) {
        std::cerr << failures << " editor session test(s) failed\n";
        return 1;
    }
    std::cout << "All editor session tests passed\n";
    return 0;
}
