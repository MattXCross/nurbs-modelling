#include "ui/ui_layer.h"

bool UILayer::handle_input(const InputFrameSnapshot& input) {
    for (auto element = m_elements.rbegin(); element != m_elements.rend(); ++element) {
        if ((*element)->has_pointer_capture()) {
            return (*element)->handle_input(input);
        }
    }

    for (auto element = m_elements.rbegin(); element != m_elements.rend(); ++element) {
        if ((*element)->handle_input(input)) {
            return true;
        }
    }

    return false;
}

void UILayer::render(IUiRenderer& renderer) const {
    for (const auto& element : m_elements) {
        element->render(renderer);
    }
}

void UILayer::clear() {
    m_elements.clear();
}
