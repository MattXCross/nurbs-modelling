#pragma once

#include <compare>
#include <cstdint>

struct EntityId {
    std::uint64_t value{0};

    auto operator<=>(const EntityId&) const = default;
    [[nodiscard]] explicit operator bool() const { return value != 0; }
};
