#pragma once

#include "ui/ui_element.h"

#include <concepts>
#include <memory>
#include <utility>
#include <vector>

class UILayer {
public:
    template<typename ElementType, typename... Args>
        requires std::derived_from<ElementType, IUIElement>
    ElementType* add_element(Args&&... args) {
        auto element = std::make_unique<ElementType>(std::forward<Args>(args)...);
        auto* result = element.get();
        m_elements.push_back(std::move(element));
        return result;
    }

    [[nodiscard]] bool handle_input(const InputFrameSnapshot& input);
    void render() const;
    void clear();

private:
    std::vector<std::unique_ptr<IUIElement>> m_elements;
};