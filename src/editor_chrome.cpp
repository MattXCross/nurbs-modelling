#include "editor_chrome.h"

#include <format>

namespace {

void render_toolbar_button(
    IUiRenderer& renderer,
    Rect bounds,
    const char* label,
    bool active = false
) {
    renderer.fill_rect(
        bounds,
        active ? Rgba{45, 108, 145, 255} : Rgba{37, 45, 57, 255}
    );
    renderer.stroke_rect(bounds, 1.0f, Rgba{69, 82, 99, 255});

    constexpr int font_size = 15;
    const Vec2 text_size = renderer.measure_text(label, font_size);
    renderer.draw_text(
        label,
        Vec2{
            bounds.x + (bounds.width - text_size.x) * 0.5f,
            bounds.y + (bounds.height - text_size.y) * 0.5f
        },
        font_size,
        active ? Rgba{245, 245, 245, 255} : Rgba{190, 199, 211, 255}
    );
}

} // namespace

void render_editor_chrome(
    IUiRenderer& renderer,
    const EditorLayout& layout,
    bool has_selection,
    int frames_per_second
) {
    renderer.fill_rect(layout.toolbar, Rgba{23, 28, 36, 255});
    renderer.fill_rect(
        Rect{0.0f, layout.toolbar.height - 1.0f, layout.toolbar.width, 1.0f},
        Rgba{69, 82, 99, 255}
    );

    renderer.draw_text("NURBSMAN", Vec2{16.0f, 15.0f}, 18, Rgba{126, 191, 236, 255});
    render_toolbar_button(renderer, {142.0f, 8.0f, 72.0f, 32.0f}, "Select", true);
    render_toolbar_button(renderer, {222.0f, 8.0f, 72.0f, 32.0f}, "Create");
    render_toolbar_button(renderer, {302.0f, 8.0f, 72.0f, 32.0f}, "Modify");
    render_toolbar_button(renderer, {382.0f, 8.0f, 72.0f, 32.0f}, "View");

    renderer.fill_rect(layout.inspector, Rgba{20, 25, 32, 255});
    renderer.fill_rect(
        Rect{
            layout.inspector.x + layout.inspector.width - 1.0f,
            layout.inspector.y,
            1.0f,
            layout.inspector.height
        },
        Rgba{69, 82, 99, 255}
    );
    renderer.draw_text("PROPERTIES", Vec2{16.0f, 66.0f}, 13, Rgba{126, 139, 156, 255});

    if (!has_selection) {
        renderer.draw_text("Nothing selected", Vec2{16.0f, 102.0f}, 16, Rgba{132, 143, 157, 255});
        renderer.draw_text(
            "Select a control point in the viewport",
            Vec2{16.0f, 128.0f},
            13,
            Rgba{91, 103, 118, 255}
        );
    }

    renderer.draw_text(
        "LMB Select   MMB Orbit   Shift + MMB Pan   Wheel Zoom",
        Vec2{layout.viewport.x + 14.0f, layout.viewport.y + layout.viewport.height - 28.0f},
        14,
        Rgba{154, 165, 179, 255}
    );

    const std::string fps_text = std::format("FPS: {}", frames_per_second);
    const Vec2 fps_size = renderer.measure_text(fps_text, 15);
    renderer.draw_text(
        fps_text,
        Vec2{layout.toolbar.width - fps_size.x - 16.0f, 17.0f},
        15,
        Rgba{154, 165, 179, 255}
    );
}
