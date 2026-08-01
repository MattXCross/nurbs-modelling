#pragma once

#include "core.h"
#include "editor_session.h"
#include "ui/ui_element.h"
#include "ui/ui_panel.h"

class ControlPointInspectorPanel final : public IUIElement {
public:
    ControlPointInspectorPanel(Vec2 position, EditorSession& session);

    void refresh();
    [[nodiscard]] const ControlPoint* selected_point() const;

    [[nodiscard]] bool handle_input(const InputFrameSnapshot& input) override;
    [[nodiscard]] bool has_pointer_capture() const override;
    void render(IUiRenderer& renderer) const override;
    void set_position(Vec2 position) override;
    [[nodiscard]] Rect bounds() const override;

private:
    void rebuild_ui();

    UIPanel m_panel;
    EditorSession& m_session;
};
