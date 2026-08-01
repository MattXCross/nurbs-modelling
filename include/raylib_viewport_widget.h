#pragma once

#include "raylib_viewport_renderer.h"

#include <QOpenGLWidget>

class EditorSession;
class QOpenGLFunctions_3_3_Core;

class RaylibViewportWidget final : public QOpenGLWidget {
public:
    explicit RaylibViewportWidget(EditorSession& session, QWidget* parent = nullptr);
    ~RaylibViewportWidget() override;

protected:
    void initializeGL() override;
    void paintGL() override;

private:
    void cleanup_gl();

    EditorSession& m_session;
    RaylibViewportRenderer m_renderer;
    QOpenGLFunctions_3_3_Core* m_gl{nullptr};
    bool m_rlgl_initialized{false};
};
