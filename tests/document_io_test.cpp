#include "document_io.h"

#include "core.h"
#include "nurbs_surface.h"
#include "scene.h"

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
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

std::unique_ptr<NurbsSurface> make_surface(double offset = 0.0) {
    const double low = std::nextafter(0.25, 0.0);
    const double high = std::nextafter(0.75, 1.0);
    std::vector<ControlPoint> points = {
        {{offset - 2.25, -0.0, 1.0 / 3.0}, 1.0},
        {{offset - 1.0, 2.0, 3.0}, std::nextafter(1.0, 2.0)},
        {{offset, -4.0, 5.0}, 2.5},
        {{offset + 1.0, 6.0, -7.0}, 0.75},
        {{offset + 2.0, 8.0, 9.0}, 4.0},
        {{offset + 3.0, -10.0, 11.0}, 1.25}
    };
    auto surface = NurbsSurface::create(
        3,
        2,
        1,
        1,
        std::move(points),
        {low, low, 0.5, high, high},
        {-2.0, -2.0, 3.0, 3.0}
    );
    expect(surface.has_value(), "construct test surface");
    return surface ? std::move(*surface) : nullptr;
}

bool same_double(double left, double right) {
    return std::bit_cast<std::uint64_t>(left) == std::bit_cast<std::uint64_t>(right);
}

bool same_surface(const NurbsSurface& left, const NurbsSurface& right) {
    if (left.u_count() != right.u_count() || left.v_count() != right.v_count() ||
        left.u_degree() != right.u_degree() || left.v_degree() != right.v_degree() ||
        left.u_knots().size() != right.u_knots().size() ||
        left.v_knots().size() != right.v_knots().size()) {
        return false;
    }
    for (std::size_t index = 0; index < left.u_knots().size(); ++index) {
        if (!same_double(left.u_knots()[index], right.u_knots()[index])) {
            return false;
        }
    }
    for (std::size_t index = 0; index < left.v_knots().size(); ++index) {
        if (!same_double(left.v_knots()[index], right.v_knots()[index])) {
            return false;
        }
    }
    const auto left_net = left.control_net_2d();
    const auto right_net = right.control_net_2d();
    for (std::size_t u = 0; u < left.u_count(); ++u) {
        for (std::size_t v = 0; v < left.v_count(); ++v) {
            const ControlPoint& a = left_net[u, v];
            const ControlPoint& b = right_net[u, v];
            if (!same_double(a.position.x, b.position.x) ||
                !same_double(a.position.y, b.position.y) ||
                !same_double(a.position.z, b.position.z) ||
                !same_double(a.weight, b.weight)) {
                return false;
            }
        }
    }
    return true;
}

bool same_scene(const Scene& left, const Scene& right) {
    if (left.nodes().size() != right.nodes().size()) {
        return false;
    }
    for (std::size_t index = 0; index < left.nodes().size(); ++index) {
        const SceneNode& a = left.nodes()[index];
        const SceneNode& b = right.nodes()[index];
        if (a.id != b.id || a.name != b.name || a.visible != b.visible ||
            a.surface == nullptr || b.surface == nullptr ||
            !same_surface(*a.surface, *b.surface)) {
            return false;
        }
    }
    return true;
}

std::string minimal_document(std::string geometry_override = {}) {
    const std::string geometry = geometry_override.empty()
        ? R"({
          "type": "nurbs_surface",
          "u_degree": 1,
          "v_degree": 1,
          "u_count": 2,
          "v_count": 2,
          "u_knots": [0, 0, 1, 1],
          "v_knots": [0, 0, 1, 1],
          "control_points": [
            {"x": 0, "y": 0, "z": 0, "weight": 1},
            {"x": 0, "y": 0, "z": 1, "weight": 1},
            {"x": 1, "y": 0, "z": 0, "weight": 1},
            {"x": 1, "y": 0, "z": 1, "weight": 1}
          ]
        })"
        : std::move(geometry_override);
    return R"({
      "format": "nurbsman",
      "version": 1,
      "units": {"length": "millimeter"},
      "coordinates": {"handedness": "right", "up_axis": "y", "front_axis": "-z"},
      "entities": [{"id": "1", "name": "Surface", "visible": true, "geometry": )" +
        geometry + "}]}";
}

void test_empty_round_trip() {
    const Scene empty;
    const auto serialized = serialize_document(empty, "empty.nurbsman");
    expect(serialized.has_value(), "serialize empty scene");
    if (!serialized) {
        return;
    }
    const auto loaded = deserialize_document(*serialized, "empty.nurbsman");
    expect(loaded.has_value(), "deserialize empty scene");
    expect(loaded && loaded->nodes().empty(), "empty scene remains empty");
}

void test_single_and_multi_entity_round_trip() {
    Scene single;
    expect(
        single.add_entity(EntityId{42}, "Precise surface", false, make_surface()).has_value(),
        "add explicit-ID entity"
    );
    const auto serialized = serialize_document(single);
    expect(serialized.has_value(), "serialize single-entity scene");
    if (serialized) {
        const auto loaded = deserialize_document(*serialized);
        expect(loaded.has_value(), "deserialize single-entity scene");
        expect(loaded && same_scene(single, *loaded), "single entity round-trips exactly");
    }

    Scene multiple;
    expect(multiple.add_entity(EntityId{9}, "First", true, make_surface()).has_value(), "add first entity");
    expect(multiple.add_entity(EntityId{3}, "Second", false, make_surface(20.0)).has_value(), "add second entity");
    const auto multi_text = serialize_document(multiple);
    expect(multi_text.has_value(), "serialize multi-entity scene");
    if (multi_text) {
        const auto multi_loaded = deserialize_document(*multi_text);
        expect(multi_loaded.has_value(), "deserialize multi-entity scene");
        expect(
            multi_loaded && same_scene(multiple, *multi_loaded),
            "entity order and metadata round-trip"
        );
    }
}

void test_file_round_trip_and_context() {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "nurbsman-document-io-test.nurbsman";
    Scene scene;
    expect(scene.add_entity("File surface", make_surface()).has_value(), "add file entity");
    const auto saved = save_document(scene, path);
    expect(saved.has_value(), "save document file");
    const auto loaded = load_document(path);
    expect(loaded.has_value(), "load document file");
    expect(loaded && same_scene(scene, *loaded), "file round-trip is exact");
    std::error_code ignored;
    std::filesystem::remove(path, ignored);

    const std::filesystem::path missing = path.string() + ".missing";
    const auto failed = load_document(missing);
    expect(!failed.has_value(), "missing file is rejected");
    if (!failed) {
        expect(failed.error().code == DocumentErrorCode::io_error, "missing file error code");
        expect(failed.error().file == missing, "missing file error includes path");
    }
}

void test_malformed_and_unsupported_documents() {
    const auto malformed = deserialize_document("{", "broken.nurbsman");
    expect(!malformed.has_value(), "malformed JSON is rejected");
    if (!malformed) {
        expect(malformed.error().code == DocumentErrorCode::malformed_json, "malformed JSON code");
        expect(malformed.error().file == "broken.nurbsman", "malformed JSON file context");
    }

    const auto duplicate_key = deserialize_document(R"({
      "format":"nurbsman", "format":"nurbsman", "version":1,
      "units":{"length":"millimeter"},
      "coordinates":{"handedness":"right","up_axis":"y","front_axis":"-z"},
      "entities":[]
    })");
    expect(!duplicate_key.has_value(), "duplicate JSON key is rejected");
    expect(
        !duplicate_key && duplicate_key.error().code == DocumentErrorCode::malformed_json,
        "duplicate key error code"
    );

    std::string future = minimal_document();
    future.replace(future.find("\"version\": 1"), 12, "\"version\": 2");
    const auto unsupported = deserialize_document(future, "future.nurbsman");
    expect(!unsupported.has_value(), "future version is rejected");
    if (!unsupported) {
        expect(unsupported.error().code == DocumentErrorCode::unsupported_version, "future version code");
        expect(unsupported.error().field == "$.version", "future version field context");
        expect(unsupported.error().message.find("supports through version 1") != std::string::npos,
            "future version error is actionable");
    }

    std::string unknown = minimal_document();
    unknown.replace(unknown.find("\"version\": 1") + 12, 0, ", \"mystery\": true");
    const auto unknown_result = deserialize_document(unknown);
    expect(!unknown_result.has_value(), "unknown version-1 field is rejected");
    expect(!unknown_result && unknown_result.error().field == "$.mystery", "unknown field context");
}

void test_invalid_geometry_and_duplicate_ids() {
    std::string invalid = minimal_document();
    const std::string valid_weight = "\"weight\": 1";
    invalid.replace(invalid.find(valid_weight), valid_weight.size(), "\"weight\": 0");
    const auto invalid_result = deserialize_document(invalid, "invalid.nurbsman");
    expect(!invalid_result.has_value(), "invalid geometry is rejected");
    if (!invalid_result) {
        expect(invalid_result.error().code == DocumentErrorCode::invalid_geometry, "invalid geometry code");
        expect(
            invalid_result.error().field == "$.entities[0].geometry.control_points[0]",
            "invalid geometry field context"
        );
    }

    std::string duplicate = minimal_document();
    const std::size_t entity_end = duplicate.rfind("]}");
    const std::size_t first_entity = duplicate.find("{\"id\": \"1\"");
    const std::string entity = duplicate.substr(first_entity, entity_end - first_entity);
    duplicate.insert(entity_end, "," + entity);
    const auto duplicate_result = deserialize_document(duplicate);
    expect(!duplicate_result.has_value(), "duplicate entity ID is rejected");
    expect(
        !duplicate_result && duplicate_result.error().code == DocumentErrorCode::duplicate_entity_id,
        "duplicate entity ID code"
    );
    expect(
        !duplicate_result && duplicate_result.error().field == "$.entities[1].id",
        "duplicate entity ID field context"
    );
}

void test_failed_load_does_not_mutate_scene() {
    Scene open_scene;
    expect(open_scene.add_entity("Existing", make_surface()).has_value(), "create open scene");
    const std::size_t original_count = open_scene.nodes().size();
    const EntityId original_id = open_scene.nodes().front().id;
    const auto attempted = deserialize_document("not JSON", "bad.nurbsman");
    expect(!attempted.has_value(), "failed temporary load");
    expect(open_scene.nodes().size() == original_count, "failed load preserves entity count");
    expect(open_scene.nodes().front().id == original_id, "failed load preserves existing entity");
}

} // namespace

int main() {
    test_empty_round_trip();
    test_single_and_multi_entity_round_trip();
    test_file_round_trip_and_context();
    test_malformed_and_unsupported_documents();
    test_invalid_geometry_and_duplicate_ids();
    test_failed_load_does_not_mutate_scene();

    if (failures != 0) {
        std::cerr << failures << " document I/O test(s) failed\n";
        return 1;
    }
    std::cout << "All document I/O tests passed\n";
    return 0;
}
