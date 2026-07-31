#pragma once

#include "entity_id.h"

#include <cstddef>
#include <variant>

struct EntitySelection {
    EntityId entity;
};

struct ControlPointSelection {
    EntityId entity;
    std::size_t u{0};
    std::size_t v{0};
};

using Selection = std::variant<std::monostate, EntitySelection, ControlPointSelection>;

class SelectionModel {
public:
    void select(EntitySelection selection) { m_selection = selection; }
    void select(ControlPointSelection selection) { m_selection = selection; }
    void clear() { m_selection = std::monostate{}; }

    [[nodiscard]] bool empty() const {
        return std::holds_alternative<std::monostate>(m_selection);
    }

    [[nodiscard]] const Selection& current() const { return m_selection; }

    [[nodiscard]] const ControlPointSelection* control_point() const {
        return std::get_if<ControlPointSelection>(&m_selection);
    }

private:
    Selection m_selection;
};
