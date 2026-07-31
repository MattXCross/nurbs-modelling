#pragma once

#include "input_frame.h"
#include "math_types.h"

class IUIElement {
public:
    virtual ~IUIElement() = default;

    [[nodiscard]] virtual bool handle_input(const InputFrameSnapshot& input) = 0;
    [[nodiscard]] virtual bool has_pointer_capture() const { return false; }
    virtual void render() const = 0;
    virtual void set_position(Vec2 position) = 0;
    [[nodiscard]] virtual Rect bounds() const = 0;
};
