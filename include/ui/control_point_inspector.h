#pragma once

#include "core.h"
#include "ui/ui_element.h"
#include "ui/ui_panel.h"

#include <cstddef>

class ControlPointInspectorPanel final : public IUIElement {
public:
    explicit ControlPointInspectorPanel(Vec2 position);

    void inspect_point(size_t u, size_t v, ControlPoint* point);
    void clear_selection();
    [[nodiscard]] ControlPoint* selected_point() const;

    [[nodiscard]] bool handle_input(const InputFrameSnapshot& input) override;
    [[nodiscard]] bool has_pointer_capture() const override;
    void render() const override;
    void set_position(Vec2 position) override;
    [[nodiscard]] Rect bounds() const override;

private:
    void rebuild_ui();

    UIPanel m_panel;
    ControlPoint* m_selected_point{nullptr};
    size_t m_selected_u{0};
    size_t m_selected_v{0};
};
