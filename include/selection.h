#pragma once

#include "entity_id.h"

#include <algorithm>
#include <cstddef>
#include <span>
#include <variant>
#include <vector>

struct EntitySelection {
    EntityId entity;
    bool operator==(const EntitySelection&) const = default;
};

struct ControlPointSelection {
    EntityId entity;
    std::size_t u{0};
    std::size_t v{0};
    bool operator==(const ControlPointSelection&) const = default;
};

using ControlPointSelections = std::vector<ControlPointSelection>;
using Selection = std::variant<std::monostate, EntitySelection, ControlPointSelections>;

class SelectionModel {
public:
    void select(EntitySelection selection) { m_selection = selection; }
    void select(ControlPointSelection selection) {
        m_selection = ControlPointSelections{selection};
    }
    void select(ControlPointSelections selections) {
        ControlPointSelections unique;
        unique.reserve(selections.size());
        for (const ControlPointSelection selection : selections) {
            if (std::ranges::find(unique, selection) == unique.end()) {
                unique.push_back(selection);
            }
        }
        m_selection = unique.empty() ? Selection{std::monostate{}} : Selection{std::move(unique)};
    }
    bool add(ControlPointSelection selection) {
        ControlPointSelections* points = std::get_if<ControlPointSelections>(&m_selection);
        if (points == nullptr) {
            select(selection);
            return true;
        }
        if (std::ranges::find(*points, selection) != points->end()) {
            return false;
        }
        points->push_back(selection);
        return true;
    }
    bool toggle(ControlPointSelection selection) {
        ControlPointSelections* points = std::get_if<ControlPointSelections>(&m_selection);
        if (points == nullptr) {
            select(selection);
            return true;
        }
        const auto found = std::ranges::find(*points, selection);
        if (found == points->end()) {
            points->push_back(selection);
        } else {
            points->erase(found);
            if (points->empty()) {
                clear();
            }
        }
        return true;
    }
    bool remove_entity(EntityId entity) {
        if (const auto* selected_entity = std::get_if<EntitySelection>(&m_selection)) {
            if (selected_entity->entity == entity) {
                clear();
                return true;
            }
            return false;
        }
        auto* points = std::get_if<ControlPointSelections>(&m_selection);
        if (points == nullptr) {
            return false;
        }
        const std::size_t original_size = points->size();
        std::erase_if(*points, [entity](ControlPointSelection point) {
            return point.entity == entity;
        });
        const bool changed = points->size() != original_size;
        if (points->empty()) {
            clear();
        }
        return changed;
    }
    void clear() { m_selection = std::monostate{}; }

    [[nodiscard]] bool empty() const {
        return std::holds_alternative<std::monostate>(m_selection);
    }

    [[nodiscard]] const Selection& current() const { return m_selection; }

    [[nodiscard]] const ControlPointSelection* control_point() const {
        const auto* points = std::get_if<ControlPointSelections>(&m_selection);
        return points == nullptr || points->empty() ? nullptr : &points->back();
    }

    [[nodiscard]] std::span<const ControlPointSelection> control_points() const {
        const auto* points = std::get_if<ControlPointSelections>(&m_selection);
        return points == nullptr ? std::span<const ControlPointSelection>{} :
            std::span<const ControlPointSelection>{*points};
    }

    [[nodiscard]] const EntitySelection* entity() const {
        return std::get_if<EntitySelection>(&m_selection);
    }

private:
    Selection m_selection;
};
