#pragma once

#include "editor_session.h"

#include <QMainWindow>

class ControlPointInspectorWidget;
class QDockWidget;
class RaylibViewportWidget;

class MainWindow final : public QMainWindow {
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private:
    EditorSession m_session;
    RaylibViewportWidget* m_viewport{nullptr};
    QDockWidget* m_inspector_dock{nullptr};
    ControlPointInspectorWidget* m_inspector{nullptr};
};
