#include "editor_session.h"

#include "core.h"
#include "nurbs_surface.h"
#include "scene.h"
#include "selection.h"

#include <iostream>
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

} // namespace

int main() {
    test_commit_pending_edit();
    test_replace_document_clears_transient_state();
    test_new_blank_document();

    if (failures != 0) {
        std::cerr << failures << " editor session test(s) failed\n";
        return 1;
    }
    std::cout << "All editor session tests passed\n";
    return 0;
}
