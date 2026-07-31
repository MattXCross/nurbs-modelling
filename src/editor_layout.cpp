#include "editor_layout.h"

#include <algorithm>

EditorLayout EditorLayout::calculate(int window_width, int window_height) {
    constexpr float toolbar_height = 48.0f;
    constexpr float inspector_width = 320.0f;

    const float width = static_cast<float>(std::max(window_width, 1));
    const float height = static_cast<float>(std::max(window_height, 1));
    const float content_height = std::max(height - toolbar_height, 1.0f);
    const float viewport_width = std::max(width - inspector_width, 1.0f);

    return {
        .toolbar = {0.0f, 0.0f, width, toolbar_height},
        .inspector = {0.0f, toolbar_height, inspector_width, content_height},
        .viewport = {inspector_width, toolbar_height, viewport_width, content_height}
    };
}
