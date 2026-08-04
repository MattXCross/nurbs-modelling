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

class Scene;

class RemovedSceneNode {
public:
    RemovedSceneNode(RemovedSceneNode&&) noexcept = default;
    RemovedSceneNode& operator=(RemovedSceneNode&&) noexcept = default;
    RemovedSceneNode(const RemovedSceneNode&) = delete;
    RemovedSceneNode& operator=(const RemovedSceneNode&) = delete;

    [[nodiscard]] EntityId entity() const { return m_node.id; }

private:
    friend class Scene;

    RemovedSceneNode(Scene& owner, SceneNode node, std::size_t index)
        : m_owner(&owner), m_node(std::move(node)), m_index(index) {}

    Scene* m_owner;
    SceneNode m_node;
    std::size_t m_index{0};
};

enum class SceneMutationError {
    entity_not_found,
    geometry_not_found,
    invalid_control_point,
    invalid_entity,
    duplicate_entity,
    entity_id_exhausted
};

class Scene {
private:
    [[nodiscard]] SceneNode* find_entity_mutable(EntityId id);

    std::vector<SceneNode> m_nodes;
    std::uint64_t m_next_entity_id{1};

public:
    [[nodiscard]] std::expected<EntityId, SceneMutationError> add_entity(
        std::string name,
        std::unique_ptr<NurbsSurface> surface
    );
    [[nodiscard]] std::expected<EntityId, SceneMutationError> add_entity(
        EntityId id,
        std::string name,
        bool visible,
        std::unique_ptr<NurbsSurface> surface
    );
    [[nodiscard]] std::expected<RemovedSceneNode, SceneMutationError> remove_entity(EntityId id);
    [[nodiscard]] std::expected<void, SceneMutationError> restore_entity(
        RemovedSceneNode& removed
    );
    [[nodiscard]] const SceneNode* find_entity(EntityId id) const;
    [[nodiscard]] std::expected<bool, SceneMutationError> rename_entity(
        EntityId id,
        std::string name
    );
    [[nodiscard]] std::expected<bool, SceneMutationError> set_entity_visibility(
        EntityId id,
        bool visible
    );
    [[nodiscard]] const ControlPoint* resolve(ControlPointSelection selection) const;
    [[nodiscard]] std::expected<bool, SceneMutationError> set_control_point(
        ControlPointSelection selection,
        ControlPoint point
    );
    void render_visible_nodes() const;
    [[nodiscard]] const std::vector<SceneNode>& nodes() const { return m_nodes; }
};
