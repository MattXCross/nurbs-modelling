#include "scene.h"
#include "nurbs_surface.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <print>
#include <ranges>
#include <utility>

std::expected<EntityId, SceneMutationError> Scene::add_entity(
    std::string name,
    std::unique_ptr<NurbsSurface> surface
) {
    if (surface == nullptr) {
        return std::unexpected(SceneMutationError::invalid_entity);
    }
    if (m_next_entity_id == 0) {
        return std::unexpected(SceneMutationError::entity_id_exhausted);
    }

    const EntityId id{m_next_entity_id};
    m_next_entity_id = m_next_entity_id == std::numeric_limits<std::uint64_t>::max()
        ? 0
        : m_next_entity_id + 1;
    m_nodes.push_back(SceneNode{id, std::move(name), true, 1, std::move(surface)});
    return id;
}

std::expected<RemovedSceneNode, SceneMutationError> Scene::remove_entity(EntityId id) {
    const auto node = std::ranges::find(m_nodes, id, &SceneNode::id);
    if (node == m_nodes.end()) {
        return std::unexpected(SceneMutationError::entity_not_found);
    }

    const std::size_t index = static_cast<std::size_t>(std::distance(m_nodes.begin(), node));
    RemovedSceneNode removed{*this, std::move(*node), index};
    m_nodes.erase(node);
    return removed;
}

std::expected<void, SceneMutationError> Scene::restore_entity(RemovedSceneNode& removed) {
    if (removed.m_owner != this || !removed.m_node.id || removed.m_node.surface == nullptr ||
        removed.m_index > m_nodes.size()) {
        return std::unexpected(SceneMutationError::invalid_entity);
    }
    if (find_entity(removed.m_node.id) != nullptr) {
        return std::unexpected(SceneMutationError::duplicate_entity);
    }

    m_nodes.insert(
        m_nodes.begin() + static_cast<std::ptrdiff_t>(removed.m_index),
        std::move(removed.m_node)
    );
    const std::uint64_t restored_id = m_nodes[removed.m_index].id.value;
    if (m_next_entity_id != 0 && restored_id >= m_next_entity_id) {
        m_next_entity_id = restored_id == std::numeric_limits<std::uint64_t>::max()
            ? 0
            : restored_id + 1;
    }
    removed.m_owner = nullptr;
    return {};
}

SceneNode* Scene::find_entity_mutable(EntityId id) {
    const auto node = std::ranges::find(m_nodes, id, &SceneNode::id);
    return node == m_nodes.end() ? nullptr : &*node;
}

const SceneNode* Scene::find_entity(EntityId id) const {
    const auto node = std::ranges::find(m_nodes, id, &SceneNode::id);
    return node == m_nodes.end() ? nullptr : &*node;
}

std::expected<bool, SceneMutationError> Scene::rename_entity(
    EntityId id,
    std::string name
) {
    SceneNode* node = find_entity_mutable(id);
    if (node == nullptr) {
        return std::unexpected(SceneMutationError::entity_not_found);
    }
    if (node->name == name) {
        return false;
    }

    node->name = std::move(name);
    return true;
}

std::expected<bool, SceneMutationError> Scene::set_entity_visibility(
    EntityId id,
    bool visible
) {
    SceneNode* node = find_entity_mutable(id);
    if (node == nullptr) {
        return std::unexpected(SceneMutationError::entity_not_found);
    }
    if (node->visible == visible) {
        return false;
    }

    node->visible = visible;
    return true;
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
