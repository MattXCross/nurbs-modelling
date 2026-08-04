#include "raylib_viewport_widget.h"

#include "editor_session.h"

#include "rlgl.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <QOpenGLContext>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLVersionFunctionsFactory>
#include <QWheelEvent>
#include <QtGlobal>

#include <algorithm>
#include <cmath>

namespace {

RaylibViewportWidget* rlgl_owner = nullptr;

void* load_opengl_function(const char* name) {
    QOpenGLContext* context = QOpenGLContext::currentContext();
    if (context == nullptr) {
        return nullptr;
    }
    return reinterpret_cast<void*>(context->getProcAddress(name));
}

ModifierKeys to_modifiers(Qt::KeyboardModifiers modifiers) {
    return {
        .shift = modifiers.testFlag(Qt::ShiftModifier),
        .ctrl = modifiers.testFlag(Qt::ControlModifier),
        .alt = modifiers.testFlag(Qt::AltModifier)
    };
}

InputFrameSnapshot pointer_input(
    QPointF position,
    QPointF delta,
    Qt::MouseButtons buttons,
    Qt::KeyboardModifiers modifiers,
    int viewport_width,
    int viewport_height
) {
    return {
        .mouse_position = {
            static_cast<float>(position.x()),
            static_cast<float>(position.y())
        },
        .mouse_delta = {
            static_cast<float>(delta.x()),
            static_cast<float>(delta.y())
        },
        .screen_width = viewport_width,
        .screen_height = viewport_height,
        .middle_mouse = buttons.testFlag(Qt::MiddleButton),
        .left_mouse = buttons.testFlag(Qt::LeftButton),
        .right_mouse = buttons.testFlag(Qt::RightButton),
        .modifiers = to_modifiers(modifiers)
    };
}

} // namespace

RaylibViewportWidget::RaylibViewportWidget(EditorSession& session, QWidget* parent)
    : QOpenGLWidget(parent),
      m_session(session) {
    setUpdateBehavior(QOpenGLWidget::NoPartialUpdate);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
}

RaylibViewportWidget::~RaylibViewportWidget() {
    cleanup_gl();
}

void RaylibViewportWidget::initializeGL() {
    if (rlgl_owner != nullptr && rlgl_owner != this) {
        qFatal("Only one RaylibViewportWidget can own raylib's global rlgl state");
    }

    const int framebuffer_width = std::max(
        1,
        static_cast<int>(std::lround(width() * devicePixelRatioF()))
    );
    const int framebuffer_height = std::max(
        1,
        static_cast<int>(std::lround(height() * devicePixelRatioF()))
    );

    m_gl = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_3_3_Core>(context());
    if (m_gl == nullptr) {
        qFatal("Nurbsman requires an OpenGL 3.3 core context");
    }

    rlLoadExtensions(reinterpret_cast<void*>(load_opengl_function));
    rlglInit(framebuffer_width, framebuffer_height);
    rlgl_owner = this;
    m_rlgl_initialized = true;

    connect(
        context(),
        &QOpenGLContext::aboutToBeDestroyed,
        this,
        [this] { cleanup_gl(); },
        Qt::DirectConnection
    );
}

void RaylibViewportWidget::paintGL() {
    if (!m_rlgl_initialized) {
        return;
    }

    const int framebuffer_width = std::max(
        1,
        static_cast<int>(std::lround(width() * devicePixelRatioF()))
    );
    const int framebuffer_height = std::max(
        1,
        static_cast<int>(std::lround(height() * devicePixelRatioF()))
    );

    rlEnableFramebuffer(defaultFramebufferObject());
    rlSetFramebufferWidth(framebuffer_width);
    rlSetFramebufferHeight(framebuffer_height);
    rlViewport(0, 0, framebuffer_width, framebuffer_height);

    m_gl->glDisable(GL_SCISSOR_TEST);
    m_gl->glDisable(GL_STENCIL_TEST);
    m_gl->glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    m_gl->glDepthMask(GL_TRUE);
    m_gl->glDepthFunc(GL_LEQUAL);
    m_gl->glEnable(GL_BLEND);
    m_gl->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    m_gl->glBlendEquation(GL_FUNC_ADD);
    m_gl->glEnable(GL_CULL_FACE);
    m_gl->glCullFace(GL_BACK);
    m_gl->glFrontFace(GL_CCW);
    m_gl->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    m_renderer.render(
        m_session.scene(),
        m_session.camera(),
        m_session.selection().control_points(),
        m_session.selected_entity_id(),
        m_session.hovered_entity_id(),
        m_session.transform_frame(),
        m_session.transform_mode(),
        m_session.active_translation_constraint(),
        m_session.active_rotation_constraint(),
        m_session.active_scale_constraint(),
        framebuffer_width,
        framebuffer_height
    );
}

void RaylibViewportWidget::mousePressEvent(QMouseEvent* event) {
    m_last_pointer_position = event->position();
    m_has_pointer_position = true;

    InputFrameSnapshot input = pointer_input(
        event->position(),
        {},
        event->buttons(),
        event->modifiers(),
        width(),
        height()
    );
    input.left_mouse_pressed = event->button() == Qt::LeftButton;
    dispatch_input(input);
    event->accept();
}

void RaylibViewportWidget::mouseReleaseEvent(QMouseEvent* event) {
    const QPointF delta = m_has_pointer_position
        ? event->position() - m_last_pointer_position
        : QPointF{};
    m_last_pointer_position = event->position();
    m_has_pointer_position = true;

    InputFrameSnapshot input = pointer_input(
        event->position(),
        delta,
        event->buttons(),
        event->modifiers(),
        width(),
        height()
    );
    input.left_mouse_released = event->button() == Qt::LeftButton;
    dispatch_input(input);
    event->accept();
}

void RaylibViewportWidget::mouseMoveEvent(QMouseEvent* event) {
    const QPointF delta = m_has_pointer_position
        ? event->position() - m_last_pointer_position
        : QPointF{};
    m_last_pointer_position = event->position();
    m_has_pointer_position = true;

    dispatch_input(pointer_input(
        event->position(),
        delta,
        event->buttons(),
        event->modifiers(),
        width(),
        height()
    ));
    event->accept();
}

void RaylibViewportWidget::wheelEvent(QWheelEvent* event) {
    InputFrameSnapshot input = pointer_input(
        event->position(),
        {},
        event->buttons(),
        event->modifiers(),
        width(),
        height()
    );
    if (!event->pixelDelta().isNull()) {
        constexpr float pixels_per_zoom_step = 15.0f;
        input.mouse_wheel_delta =
            static_cast<float>(event->pixelDelta().y()) / pixels_per_zoom_step;
    } else {
        input.mouse_wheel_delta = static_cast<float>(event->angleDelta().y()) / 120.0f;
    }
    dispatch_input(input);
    event->accept();
}

void RaylibViewportWidget::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape && m_session.transform_active()) {
        InputFrameSnapshot input;
        input.screen_width = width();
        input.screen_height = height();
        input.escape_pressed = true;
        dispatch_input(input);
        event->accept();
        return;
    }
    QOpenGLWidget::keyPressEvent(event);
}

void RaylibViewportWidget::dispatch_input(InputFrameSnapshot input) {
    m_session.process_viewport_input(input);
    update();
}

void RaylibViewportWidget::cleanup_gl() {
    if (!m_rlgl_initialized) {
        return;
    }

    makeCurrent();
    rlglClose();
    rlgl_owner = nullptr;
    m_gl = nullptr;
    m_rlgl_initialized = false;
    doneCurrent();
}
