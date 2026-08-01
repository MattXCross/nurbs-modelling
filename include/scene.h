#pragma once

#include <expected>
#include <memory>
#include <vector>
#include <string>

#include "entity_id.h"
#include "nurbs_surface.h"
#include "selection.h"

struct SceneNode {
    EntityId id;
    std::string name;
    bool visible{true};
    std::uint64_t geometry_revision{1};
    std::unique_ptr<NurbsSurface> surface;
};

enum class SceneMutationError {
    entity_not_found,
    geometry_not_found,
    invalid_control_point
};

class Scene {
private:
    [[nodiscard]] SceneNode* find_entity_mutable(EntityId id);

    std::vector<SceneNode> m_nodes;
    std::uint64_t m_next_entity_id{1};

public:
    [[nodiscard]] EntityId add_entity(std::string name, std::unique_ptr<NurbsSurface> surface);
    [[nodiscard]] const SceneNode* find_entity(EntityId id) const;
    [[nodiscard]] const ControlPoint* resolve(ControlPointSelection selection) const;
    [[nodiscard]] std::expected<bool, SceneMutationError> set_control_point(
        ControlPointSelection selection,
        ControlPoint point
    );
    void render_visible_nodes() const;
    [[nodiscard]] const std::vector<SceneNode>& nodes() const { return m_nodes; }
};
