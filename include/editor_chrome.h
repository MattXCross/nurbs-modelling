#pragma once

#include "editor_layout.h"
#include "ui/ui_renderer.h"

void render_editor_chrome(
    IUiRenderer& renderer,
    const EditorLayout& layout,
    bool has_selection,
    int frames_per_second
);
