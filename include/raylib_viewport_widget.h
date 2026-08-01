#pragma once

#include "input_frame.h"
#include "raylib_viewport_renderer.h"

#include <QOpenGLWidget>
#include <QPointF>

#include <functional>

class EditorSession;
class QMouseEvent;
class QOpenGLFunctions_3_3_Core;
class QWheelEvent;

class RaylibViewportWidget final : public QOpenGLWidget {
public:
    using SelectionChangedHandler = std::function<void()>;

    explicit RaylibViewportWidget(EditorSession& session, QWidget* parent = nullptr);
    ~RaylibViewportWidget() override;

    void set_selection_changed_handler(SelectionChangedHandler handler);

protected:
    void initializeGL() override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void cleanup_gl();
    void dispatch_input(InputFrameSnapshot input);

    EditorSession& m_session;
    RaylibViewportRenderer m_renderer;
    QOpenGLFunctions_3_3_Core* m_gl{nullptr};
    QPointF m_last_pointer_position;
    SelectionChangedHandler m_selection_changed_handler;
    bool m_rlgl_initialized{false};
    bool m_has_pointer_position{false};
};
