#pragma once

#include "input_frame.h"
#include "gizmo_controller.h"
#include "raylib_viewport_renderer.h"
#include "viewport_display_settings.h"

#include <QOpenGLWidget>
#include <QElapsedTimer>
#include <QPointF>

#include <functional>
class EditorSession;
class QKeyEvent;
class QMouseEvent;
class QOpenGLFunctions_3_3_Core;
class QWheelEvent;

class RaylibViewportWidget final : public QOpenGLWidget {
public:
    explicit RaylibViewportWidget(EditorSession& session, QWidget* parent = nullptr);
    ~RaylibViewportWidget() override;

    [[nodiscard]] const ViewportDisplaySettings& display_settings() const {
        return m_display_settings;
    }
    void set_display_settings(ViewportDisplaySettings settings);
    void set_fps_handler(std::move_only_function<void(double)> handler);

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
    GizmoController m_gizmos;
    RaylibViewportRenderer m_renderer;
    ViewportDisplaySettings m_display_settings;
    std::move_only_function<void(double)> m_fps_handler;
    QElapsedTimer m_fps_timer;
    int m_frames_in_window{0};
    QOpenGLFunctions_3_3_Core* m_gl{nullptr};
    QPointF m_last_pointer_position;
    bool m_rlgl_initialized{false};
    bool m_has_pointer_position{false};
};
