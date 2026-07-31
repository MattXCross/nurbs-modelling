#pragma once

#include "ui/ui_element.h"

#include <functional>
#include <string>

class UILabel final : public IUIElement {
public:
    UILabel(Vec2 position, std::string text, int font_size = 16, Rgba color = {80, 80, 80, 255});

    void set_text(std::string text);

    [[nodiscard]] bool handle_input(const InputFrameSnapshot& input) override;
    void render(IUiRenderer& renderer) const override;
    void set_position(Vec2 position) override;
    [[nodiscard]] Rect bounds() const override;

private:
    mutable Rect m_bounds{};
    std::string m_text;
    Rgba m_color{80, 80, 80, 255};
    int m_font_size{16};
};

class UISlider final : public IUIElement {
public:
    UISlider(
        Rect bounds,
        std::string label,
        float minimum,
        float maximum,
        float initial_value,
        std::move_only_function<void(float)> on_value_changed,
        std::move_only_function<void()> on_edit_started = {},
        std::move_only_function<void(float)> on_edit_finished = {}
    );

    void set_value(float value);
    [[nodiscard]] float value() const;
    [[nodiscard]] bool is_dragging() const;

    [[nodiscard]] bool handle_input(const InputFrameSnapshot& input) override;
    [[nodiscard]] bool has_pointer_capture() const override;
    void render(IUiRenderer& renderer) const override;
    void set_position(Vec2 position) override;
    [[nodiscard]] Rect bounds() const override;

private:
    [[nodiscard]] float normalized_value() const;
    void update_from_mouse(float mouse_x);

    Rect m_bounds{};
    std::string m_label;
    float m_minimum{0.0f};
    float m_maximum{1.0f};
    float m_value{0.0f};
    bool m_is_dragging{false};
    std::move_only_function<void(float)> m_on_value_changed;
    std::move_only_function<void()> m_on_edit_started;
    std::move_only_function<void(float)> m_on_edit_finished;
};
