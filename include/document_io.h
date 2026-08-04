#pragma once

#include "scene.h"

#include <expected>
#include <filesystem>
#include <string>
#include <string_view>

enum class DocumentErrorCode {
    io_error,
    malformed_json,
    invalid_field,
    unsupported_version,
    duplicate_entity_id,
    invalid_geometry
};

struct DocumentError {
    DocumentErrorCode code;
    std::filesystem::path file;
    std::string field;
    std::string message;
};

[[nodiscard]] std::expected<std::string, DocumentError> serialize_document(
    const Scene& scene,
    std::filesystem::path file = {}
);

[[nodiscard]] std::expected<Scene, DocumentError> deserialize_document(
    std::string_view contents,
    std::filesystem::path file = {}
);

[[nodiscard]] std::expected<void, DocumentError> save_document(
    const Scene& scene,
    const std::filesystem::path& file
);

[[nodiscard]] std::expected<Scene, DocumentError> load_document(
    const std::filesystem::path& file
);
