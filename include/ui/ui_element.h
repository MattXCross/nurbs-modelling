#pragma once

#include "input_frame.h"
#include "raylib.h"

class IUIElement {
public:
    virtual ~IUIElement() = default;

    [[nodiscard]] virtual bool handle_input(const InputFrameSnapshot& input) = 0;
    [[nodiscard]] virtual bool has_pointer_capture() const { return false; }
    virtual void render() const = 0;
    virtual void set_position(Vector2 position) = 0;
    [[nodiscard]] virtual Rectangle bounds() const = 0;
};