#pragma once

#include "ui/ui_element.h"

#include <concepts>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class UIPanel final : public IUIElement {
public:
    UIPanel(Rectangle bounds, std::string title);

    template<typename WidgetType, typename... Args>
        requires std::derived_from<WidgetType, IUIElement>
    WidgetType* add_child(Args&&... args) {
        auto widget = std::make_unique<WidgetType>(std::forward<Args>(args)...);
        auto* result = widget.get();
        m_children.push_back(std::move(widget));
        return result;
    }

    void clear_children();

    [[nodiscard]] bool handle_input(const InputFrameSnapshot& input) override;
    [[nodiscard]] bool has_pointer_capture() const override;
    void render() const override;
    void set_position(Vector2 position) override;
    [[nodiscard]] Rectangle bounds() const override;

private:
    Rectangle m_bounds{};
    std::string m_title;
    std::vector<std::unique_ptr<IUIElement>> m_children;
};