#pragma once

#include "core.h"
#include "scene.h"
#include "selection.h"
#include "ui/ui_element.h"
#include "ui/ui_panel.h"

#include <cstddef>

class ControlPointInspectorPanel final : public IUIElement {
public:
    ControlPointInspectorPanel(Vec2 position, Scene& scene, SelectionModel& selection);

    void inspect_point(ControlPointSelection selection);
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
    Scene& m_scene;
    SelectionModel& m_selection;
};
