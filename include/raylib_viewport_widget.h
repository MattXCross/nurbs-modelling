#pragma once

#include "input_frame.h"
#include "raylib_viewport_renderer.h"

#include <QOpenGLWidget>
#include <QPointF>

class EditorSession;
class QKeyEvent;
class QMouseEvent;
class QOpenGLFunctions_3_3_Core;
class QWheelEvent;

class RaylibViewportWidget final : public QOpenGLWidget {
public:
    explicit RaylibViewportWidget(EditorSession& session, QWidget* parent = nullptr);
    ~RaylibViewportWidget() override;

protected:
    void initializeGL() override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    void cleanup_gl();
    void dispatch_input(InputFrameSnapshot input);

    EditorSession& m_session;
    RaylibViewportRenderer m_renderer;
    QOpenGLFunctions_3_3_Core* m_gl{nullptr};
    QPointF m_last_pointer_position;
    bool m_rlgl_initialized{false};
    bool m_has_pointer_position{false};
};
