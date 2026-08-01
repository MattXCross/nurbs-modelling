#pragma once

#include "editor_session.h"

#include <QMainWindow>

class ControlPointInspectorWidget;
class QAction;
class QDockWidget;
class QLabel;
class RaylibViewportWidget;

class MainWindow final : public QMainWindow {
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void undo();
    void redo();
    void handle_editor_change(EditorChange change);
    void refresh_ui_state();

    EditorSession m_session;
    RaylibViewportWidget* m_viewport{nullptr};
    QDockWidget* m_inspector_dock{nullptr};
    ControlPointInspectorWidget* m_inspector{nullptr};
    QAction* m_undo_action{nullptr};
    QAction* m_redo_action{nullptr};
    QLabel* m_selection_status{nullptr};
};
