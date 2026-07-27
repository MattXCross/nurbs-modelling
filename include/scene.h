#pragma once

#include <memory>
#include <vector>
#include <string>

#include "nurbs_surface.h"

struct SceneNode {
    std::string name;
    bool visible{true};
    std::unique_ptr<NurbsSurface> surface;
};

class Scene {
private:
    std::vector<SceneNode> m_nodes;

public:
    void add_entity(std::string name, std::unique_ptr<NurbsSurface> surface);
    void render_visible_nodes() const;
    [[nodiscard]] const std::vector<SceneNode>& nodes() const { return m_nodes; }
};