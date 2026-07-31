#pragma once

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

class Scene {
private:
    std::vector<SceneNode> m_nodes;
    std::uint64_t m_next_entity_id{1};

public:
    [[nodiscard]] EntityId add_entity(std::string name, std::unique_ptr<NurbsSurface> surface);
    [[nodiscard]] SceneNode* find_entity(EntityId id);
    [[nodiscard]] const SceneNode* find_entity(EntityId id) const;
    [[nodiscard]] bool mark_geometry_modified(EntityId id);
    [[nodiscard]] ControlPoint* resolve(ControlPointSelection selection);
    [[nodiscard]] const ControlPoint* resolve(ControlPointSelection selection) const;
    void render_visible_nodes() const;
    [[nodiscard]] const std::vector<SceneNode>& nodes() const { return m_nodes; }
};
