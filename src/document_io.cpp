#include "document_io.h"

#include "core.h"
#include "nurbs_surface.h"

#include <nlohmann/json.hpp>

#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <optional>
#include <ranges>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

using Json = nlohmann::json;

struct ParseFailure final : std::exception {
    DocumentErrorCode code;
    std::string field;
    std::string detail;

    ParseFailure(DocumentErrorCode code, std::string field, std::string detail)
        : code(code), field(std::move(field)), detail(std::move(detail)) {}

    [[nodiscard]] const char* what() const noexcept override { return detail.c_str(); }
};

[[noreturn]] void fail(
    DocumentErrorCode code,
    std::string field,
    std::string detail
) {
    throw ParseFailure{code, std::move(field), std::move(detail)};
}

DocumentError error_with_file(
    const std::filesystem::path& file,
    DocumentErrorCode code,
    std::string field,
    std::string message
) {
    return {code, file, std::move(field), std::move(message)};
}

void require_object(const Json& value, const std::string& field) {
    if (!value.is_object()) {
        fail(DocumentErrorCode::invalid_field, field, "expected an object");
    }
}

void require_fields(
    const Json& value,
    const std::string& field,
    std::initializer_list<std::string_view> required,
    std::initializer_list<std::string_view> optional = {}
) {
    require_object(value, field);
    for (const std::string_view name : required) {
        if (!value.contains(name)) {
            fail(
                DocumentErrorCode::invalid_field,
                field + "." + std::string(name),
                "required field is missing"
            );
        }
    }
    for (const auto& [name, unused] : value.items()) {
        (void)unused;
        const auto known = [name = std::string_view(name)](std::string_view candidate) {
            return candidate == name;
        };
        if (std::ranges::none_of(required, known) && std::ranges::none_of(optional, known)) {
            fail(
                DocumentErrorCode::invalid_field,
                field + "." + name,
                "unknown field in document version 1"
            );
        }
    }
}

const Json& field(const Json& object, std::string_view name) {
    return object.at(name);
}

std::string require_string(const Json& value, const std::string& path) {
    if (!value.is_string()) {
        fail(DocumentErrorCode::invalid_field, path, "expected a string");
    }
    return value.get<std::string>();
}

bool require_boolean(const Json& value, const std::string& path) {
    if (!value.is_boolean()) {
        fail(DocumentErrorCode::invalid_field, path, "expected a boolean");
    }
    return value.get<bool>();
}

std::size_t require_size(const Json& value, const std::string& path) {
    if (!value.is_number_unsigned()) {
        fail(DocumentErrorCode::invalid_field, path, "expected a non-negative integer");
    }
    const std::uint64_t number = value.get<std::uint64_t>();
    if (number > std::numeric_limits<std::size_t>::max()) {
        fail(DocumentErrorCode::invalid_field, path, "integer is too large for this build");
    }
    return static_cast<std::size_t>(number);
}

double require_number(const Json& value, const std::string& path) {
    if (!value.is_number()) {
        fail(DocumentErrorCode::invalid_field, path, "expected a finite number");
    }
    try {
        const double number = value.get<double>();
        if (!std::isfinite(number)) {
            fail(DocumentErrorCode::invalid_field, path, "number is not finite");
        }
        return number;
    } catch (const Json::exception&) {
        fail(DocumentErrorCode::invalid_field, path, "number is outside binary64 range");
    }
}

EntityId require_entity_id(const Json& value, const std::string& path) {
    const std::string text = require_string(value, path);
    if (text.empty() || text.front() == '0') {
        fail(DocumentErrorCode::invalid_field, path, "expected a canonical positive uint64 string");
    }
    std::uint64_t id = 0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), id);
    if (error != std::errc{} || end != text.data() + text.size() || id == 0) {
        fail(DocumentErrorCode::invalid_field, path, "expected a canonical positive uint64 string");
    }
    return EntityId{id};
}

std::vector<double> require_number_array(const Json& value, const std::string& path) {
    if (!value.is_array()) {
        fail(DocumentErrorCode::invalid_field, path, "expected an array");
    }
    std::vector<double> result;
    result.reserve(value.size());
    for (std::size_t index = 0; index < value.size(); ++index) {
        result.push_back(require_number(value[index], path + "[" + std::to_string(index) + "]"));
    }
    return result;
}

std::string geometry_error_message(const NurbsSurfaceError& error) {
    switch (error.code) {
        case NurbsSurfaceErrorCode::invalid_control_net_dimensions:
            return "invalid control-net dimensions";
        case NurbsSurfaceErrorCode::control_point_count_mismatch:
            return "control-point count does not match dimensions";
        case NurbsSurfaceErrorCode::degree_out_of_range: return "degree is out of range";
        case NurbsSurfaceErrorCode::knot_count_mismatch: return "knot count is invalid";
        case NurbsSurfaceErrorCode::non_finite_knot: return "knot is not finite";
        case NurbsSurfaceErrorCode::knots_not_nondecreasing:
            return "knots are not nondecreasing";
        case NurbsSurfaceErrorCode::knot_range_not_finite:
            return "knot range is not supported";
        case NurbsSurfaceErrorCode::knot_multiplicity_exceeded:
            return "knot multiplicity exceeds degree plus one";
        case NurbsSurfaceErrorCode::empty_parameter_domain:
            return "parameter domain is empty";
        case NurbsSurfaceErrorCode::non_finite_control_point:
            return "control point is not finite";
        case NurbsSurfaceErrorCode::non_positive_weight:
            return "control-point weight must be positive";
        case NurbsSurfaceErrorCode::numeric_range_not_supported:
            return "geometry numeric range is not supported";
        case NurbsSurfaceErrorCode::control_point_out_of_range:
            return "control point is out of range";
    }
    return "invalid NURBS surface";
}

std::string geometry_error_path(const std::string& base, const NurbsSurfaceError& error) {
    if (error.direction.has_value()) {
        const std::string direction = *error.direction == NurbsParameterDirection::u ? "u" : "v";
        if (error.code == NurbsSurfaceErrorCode::degree_out_of_range) {
            return base + "." + direction + "_degree";
        }
        std::string result = base + "." + direction + "_knots";
        if (error.index != NurbsSurfaceError::no_index) {
            result += "[" + std::to_string(error.index) + "]";
        }
        return result;
    }
    if (error.index != NurbsSurfaceError::no_index) {
        return base + ".control_points[" + std::to_string(error.index) + "]";
    }
    return base;
}

std::unique_ptr<NurbsSurface> parse_surface(const Json& value, const std::string& path) {
    require_fields(
        value,
        path,
        {"type", "u_degree", "v_degree", "u_count", "v_count", "u_knots", "v_knots", "control_points"}
    );
    const std::string type = require_string(field(value, "type"), path + ".type");
    if (type != "nurbs_surface") {
        fail(DocumentErrorCode::invalid_field, path + ".type", "unsupported geometry type '" + type + "'");
    }

    const std::size_t u_count = require_size(field(value, "u_count"), path + ".u_count");
    const std::size_t v_count = require_size(field(value, "v_count"), path + ".v_count");
    const std::size_t u_degree = require_size(field(value, "u_degree"), path + ".u_degree");
    const std::size_t v_degree = require_size(field(value, "v_degree"), path + ".v_degree");
    std::vector<double> u_knots = require_number_array(field(value, "u_knots"), path + ".u_knots");
    std::vector<double> v_knots = require_number_array(field(value, "v_knots"), path + ".v_knots");

    const Json& control_points = field(value, "control_points");
    if (!control_points.is_array()) {
        fail(DocumentErrorCode::invalid_field, path + ".control_points", "expected an array");
    }
    std::vector<ControlPoint> points;
    points.reserve(control_points.size());
    for (std::size_t index = 0; index < control_points.size(); ++index) {
        const std::string point_path = path + ".control_points[" + std::to_string(index) + "]";
        const Json& point = control_points[index];
        require_fields(point, point_path, {"x", "y", "z", "weight"});
        points.push_back(ControlPoint{
            .position = {
                require_number(field(point, "x"), point_path + ".x"),
                require_number(field(point, "y"), point_path + ".y"),
                require_number(field(point, "z"), point_path + ".z")
            },
            .weight = require_number(field(point, "weight"), point_path + ".weight")
        });
    }

    auto surface = NurbsSurface::create(
        u_count,
        v_count,
        u_degree,
        v_degree,
        std::move(points),
        std::move(u_knots),
        std::move(v_knots)
    );
    if (!surface) {
        fail(
            DocumentErrorCode::invalid_geometry,
            geometry_error_path(path, surface.error()),
            geometry_error_message(surface.error())
        );
    }
    return std::move(*surface);
}

Json serialize_surface(const NurbsSurface& surface) {
    Json points = Json::array();
    const auto net = surface.control_net_2d();
    for (std::size_t u = 0; u < surface.u_count(); ++u) {
        for (std::size_t v = 0; v < surface.v_count(); ++v) {
            const ControlPoint& point = net[u, v];
            points.push_back({
                {"x", point.position.x},
                {"y", point.position.y},
                {"z", point.position.z},
                {"weight", point.weight}
            });
        }
    }
    return {
        {"type", "nurbs_surface"},
        {"u_degree", surface.u_degree()},
        {"v_degree", surface.v_degree()},
        {"u_count", surface.u_count()},
        {"v_count", surface.v_count()},
        {"u_knots", surface.u_knots()},
        {"v_knots", surface.v_knots()},
        {"control_points", std::move(points)}
    };
}

Json parse_strict_json(std::string_view contents) {
    std::vector<std::unordered_set<std::string>> object_keys;
    const auto callback = [&object_keys](int, Json::parse_event_t event, Json& parsed) {
        if (event == Json::parse_event_t::object_start) {
            object_keys.emplace_back();
        } else if (event == Json::parse_event_t::key) {
            const std::string key = parsed.get<std::string>();
            if (object_keys.empty() || !object_keys.back().insert(key).second) {
                fail(DocumentErrorCode::malformed_json, "$", "duplicate object key '" + key + "'");
            }
        } else if (event == Json::parse_event_t::object_end) {
            object_keys.pop_back();
        }
        return true;
    };
    return Json::parse(contents.begin(), contents.end(), callback, true, false);
}

} // namespace

std::expected<std::string, DocumentError> serialize_document(
    const Scene& scene,
    std::filesystem::path file
) {
    try {
        Json entities = Json::array();
        for (const SceneNode& node : scene.nodes()) {
            if (node.surface == nullptr) {
                return std::unexpected(error_with_file(
                    file,
                    DocumentErrorCode::invalid_geometry,
                    "$.entities",
                    "scene entity has no geometry"
                ));
            }
            entities.push_back({
                {"id", std::to_string(node.id.value)},
                {"name", node.name},
                {"visible", node.visible},
                {"geometry", serialize_surface(*node.surface)}
            });
        }
        const Json document = {
            {"format", "nurbsman"},
            {"version", 1},
            {"generator", "Nurbsman"},
            {"units", {{"length", "millimeter"}}},
            {"coordinates", {
                {"handedness", "right"},
                {"up_axis", "y"},
                {"front_axis", "-z"}
            }},
            {"entities", std::move(entities)}
        };
        return document.dump(2) + '\n';
    } catch (const Json::exception& error) {
        return std::unexpected(error_with_file(
            file,
            DocumentErrorCode::invalid_field,
            "$",
            error.what()
        ));
    }
}

std::expected<Scene, DocumentError> deserialize_document(
    std::string_view contents,
    std::filesystem::path file
) {
    try {
        const Json document = parse_strict_json(contents);
        require_fields(
            document,
            "$",
            {"format", "version", "units", "coordinates", "entities"},
            {"generator"}
        );
        if (require_string(field(document, "format"), "$.format") != "nurbsman") {
            fail(DocumentErrorCode::invalid_field, "$.format", "expected 'nurbsman'");
        }
        if (!field(document, "version").is_number_integer()) {
            fail(DocumentErrorCode::invalid_field, "$.version", "expected an integer");
        }
        const std::int64_t version = field(document, "version").get<std::int64_t>();
        if (version != 1) {
            fail(
                DocumentErrorCode::unsupported_version,
                "$.version",
                "unsupported document version " + std::to_string(version) +
                    "; this build supports through version 1"
            );
        }
        if (document.contains("generator")) {
            (void)require_string(field(document, "generator"), "$.generator");
        }

        const Json& units = field(document, "units");
        require_fields(units, "$.units", {"length"});
        if (require_string(field(units, "length"), "$.units.length") != "millimeter") {
            fail(DocumentErrorCode::invalid_field, "$.units.length", "expected 'millimeter'");
        }
        const Json& coordinates = field(document, "coordinates");
        require_fields(coordinates, "$.coordinates", {"handedness", "up_axis", "front_axis"});
        if (require_string(field(coordinates, "handedness"), "$.coordinates.handedness") != "right") {
            fail(DocumentErrorCode::invalid_field, "$.coordinates.handedness", "expected 'right'");
        }
        if (require_string(field(coordinates, "up_axis"), "$.coordinates.up_axis") != "y") {
            fail(DocumentErrorCode::invalid_field, "$.coordinates.up_axis", "expected 'y'");
        }
        if (require_string(field(coordinates, "front_axis"), "$.coordinates.front_axis") != "-z") {
            fail(DocumentErrorCode::invalid_field, "$.coordinates.front_axis", "expected '-z'");
        }

        const Json& entities = field(document, "entities");
        if (!entities.is_array()) {
            fail(DocumentErrorCode::invalid_field, "$.entities", "expected an array");
        }
        Scene scene;
        for (std::size_t index = 0; index < entities.size(); ++index) {
            const std::string path = "$.entities[" + std::to_string(index) + "]";
            const Json& entity = entities[index];
            require_fields(entity, path, {"id", "name", "visible", "geometry"});
            const EntityId id = require_entity_id(field(entity, "id"), path + ".id");
            auto added = scene.add_entity(
                id,
                require_string(field(entity, "name"), path + ".name"),
                require_boolean(field(entity, "visible"), path + ".visible"),
                parse_surface(field(entity, "geometry"), path + ".geometry")
            );
            if (!added) {
                const DocumentErrorCode code = added.error() == SceneMutationError::duplicate_entity
                    ? DocumentErrorCode::duplicate_entity_id
                    : DocumentErrorCode::invalid_field;
                fail(code, path + ".id", "entity ID is duplicated or invalid");
            }
        }
        return scene;
    } catch (const ParseFailure& error) {
        return std::unexpected(error_with_file(file, error.code, error.field, error.detail));
    } catch (const Json::parse_error& error) {
        return std::unexpected(error_with_file(
            file,
            DocumentErrorCode::malformed_json,
            "$",
            error.what()
        ));
    } catch (const Json::exception& error) {
        return std::unexpected(error_with_file(
            file,
            DocumentErrorCode::invalid_field,
            "$",
            error.what()
        ));
    }
}

std::expected<void, DocumentError> save_document(
    const Scene& scene,
    const std::filesystem::path& file
) {
    auto contents = serialize_document(scene, file);
    if (!contents) {
        return std::unexpected(contents.error());
    }
    std::ofstream output(file, std::ios::binary | std::ios::trunc);
    if (!output || !output.write(contents->data(), static_cast<std::streamsize>(contents->size()))) {
        return std::unexpected(error_with_file(
            file,
            DocumentErrorCode::io_error,
            "$",
            "could not write document"
        ));
    }
    return {};
}

std::expected<Scene, DocumentError> load_document(const std::filesystem::path& file) {
    std::ifstream input(file, std::ios::binary);
    if (!input) {
        return std::unexpected(error_with_file(
            file,
            DocumentErrorCode::io_error,
            "$",
            "could not open document"
        ));
    }
    std::ostringstream contents;
    contents << input.rdbuf();
    if (input.bad()) {
        return std::unexpected(error_with_file(
            file,
            DocumentErrorCode::io_error,
            "$",
            "could not read document"
        ));
    }
    return deserialize_document(std::move(contents).str(), file);
}
