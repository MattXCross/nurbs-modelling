#pragma once

#include "command_history.h"
#include "core.h"
#include "scene.h"
#include "selection.h"
#include "ui/ui_element.h"
#include "ui/ui_panel.h"

#include <cstddef>
#include <optional>

class ControlPointInspectorPanel final : public IUIElement {
public:
    ControlPointInspectorPanel(
        Vec2 position,
        Scene& scene,
        SelectionModel& selection,
        CommandHistory& history
    );

    void inspect_point(ControlPointSelection selection);
    void clear_selection();
    void refresh();
    [[nodiscard]] ControlPoint* selected_point() const;

    [[nodiscard]] bool handle_input(const InputFrameSnapshot& input) override;
    [[nodiscard]] bool has_pointer_capture() const override;
    void render(IUiRenderer& renderer) const override;
    void set_position(Vec2 position) override;
    [[nodiscard]] Rect bounds() const override;

private:
    enum class ControlPointField { position_x, position_y, position_z, weight };

    struct PendingEdit {
        ControlPointSelection selection;
        ControlPointField field;
        double initial_value{0.0};
    };

    void rebuild_ui();
    void begin_edit(ControlPointField field);
    void preview_edit(ControlPointField field, float value);
    void finish_edit(ControlPointField field);
    [[nodiscard]] static double field_value(const ControlPoint& point, ControlPointField field);
    static void set_field_value(ControlPoint& point, ControlPointField field, double value);

    UIPanel m_panel;
    Scene& m_scene;
    SelectionModel& m_selection;
    CommandHistory& m_history;
    std::optional<PendingEdit> m_pending_edit;
};
