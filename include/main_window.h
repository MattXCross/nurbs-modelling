#pragma once

#include "editor_session.h"

#include <QMainWindow>

#include <optional>
#include <string>

class ControlPointInspectorWidget;
class QAction;
class QDockWidget;
class QLabel;
class RaylibViewportWidget;
class SceneOutlinerWidget;

class MainWindow final : public QMainWindow {
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void undo();
    void redo();
    void create_surface();
    void delete_selected_entity();
    void rename_selected_entity();
    void toggle_selected_entity_visibility();
    void handle_editor_change(EditorChange change);
    void refresh_ui_state();
    [[nodiscard]] std::optional<EntityId> selected_entity_id() const;
    [[nodiscard]] std::string suggested_surface_name() const;

    EditorSession m_session;
    RaylibViewportWidget* m_viewport{nullptr};
    QDockWidget* m_outliner_dock{nullptr};
    SceneOutlinerWidget* m_outliner{nullptr};
    QDockWidget* m_inspector_dock{nullptr};
    ControlPointInspectorWidget* m_inspector{nullptr};
    QAction* m_undo_action{nullptr};
    QAction* m_redo_action{nullptr};
    QAction* m_create_surface_action{nullptr};
    QAction* m_delete_entity_action{nullptr};
    QAction* m_rename_entity_action{nullptr};
    QAction* m_toggle_visibility_action{nullptr};
    QLabel* m_selection_status{nullptr};
};
