#include "raylib_viewport_widget.h"

#include "editor_session.h"

#include "rlgl.h"

#include <QOpenGLContext>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLVersionFunctionsFactory>
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

} // namespace

RaylibViewportWidget::RaylibViewportWidget(EditorSession& session, QWidget* parent)
    : QOpenGLWidget(parent),
      m_session(session) {
    setUpdateBehavior(QOpenGLWidget::NoPartialUpdate);
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
        m_session.selected_control_point(),
        framebuffer_width,
        framebuffer_height
    );
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
