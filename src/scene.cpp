#include "scene.h"
#include "nurbs_surface.h"

#include <algorithm>
#include <memory>
#include <print>
#include <ranges>
#include <utility>

EntityId Scene::add_entity(std::string name, std::unique_ptr<NurbsSurface> surface) {
    const EntityId id{m_next_entity_id++};
    m_nodes.push_back(SceneNode{id, std::move(name), true, 1, std::move(surface)});
    return id;
}

SceneNode* Scene::find_entity_mutable(EntityId id) {
    const auto node = std::ranges::find(m_nodes, id, &SceneNode::id);
    return node == m_nodes.end() ? nullptr : &*node;
}

const SceneNode* Scene::find_entity(EntityId id) const {
    const auto node = std::ranges::find(m_nodes, id, &SceneNode::id);
    return node == m_nodes.end() ? nullptr : &*node;
}

const ControlPoint* Scene::resolve(ControlPointSelection selection) const {
    const SceneNode* node = find_entity(selection.entity);
    if (node == nullptr || node->surface == nullptr) {
        return nullptr;
    }

    const NurbsSurface& surface = *node->surface;
    auto control_net = surface.control_net_2d();
    if (selection.u >= control_net.extent(0) || selection.v >= control_net.extent(1)) {
        return nullptr;
    }
    return &control_net[selection.u, selection.v];
}

std::expected<bool, SceneMutationError> Scene::set_control_point(
    ControlPointSelection selection,
    ControlPoint point
) {
    SceneNode* node = find_entity_mutable(selection.entity);
    if (node == nullptr) {
        return std::unexpected(SceneMutationError::entity_not_found);
    }
    if (node->surface == nullptr) {
        return std::unexpected(SceneMutationError::geometry_not_found);
    }

    const auto changed = node->surface->set_control_point(
        selection.u,
        selection.v,
        point
    );
    if (!changed.has_value()) {
        return std::unexpected(SceneMutationError::invalid_control_point);
    }
    if (*changed) {
        ++node->geometry_revision;
    }
    return *changed;
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
