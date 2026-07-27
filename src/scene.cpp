#include "scene.h"
#include "nurbs_surface.h"

#include <memory>
#include <print>

void Scene::add_entity(std::string name, std::unique_ptr<NurbsSurface> surface) {
    m_nodes.push_back(SceneNode{std::move(name), true, std::move(surface)});
}

void Scene::render_visible_nodes() const {
    auto visible_surfaces = m_nodes | std::views::filter(
            [](const SceneNode& node) { return node.visible && node.surface != nullptr; }
        );

    for (const auto& node : visible_surfaces) {
        std::println("Node: {}", node.name);
        auto point = node.surface->evaluate(0.5, 0.5);

        if (point) {
            std::println("Center: ({}, {}, {})", point->x, point->y, point->z);
        }
    }
}