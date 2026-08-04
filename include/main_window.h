#pragma once

#include "editor_session.h"
#include "document_io.h"

#include <QMainWindow>

#include <filesystem>
#include <optional>
#include <string>

class ControlPointInspectorWidget;
class QAction;
class QCloseEvent;
class QDockWidget;
class QLabel;
class QString;
class RaylibViewportWidget;
class SceneOutlinerWidget;

class MainWindow final : public QMainWindow {
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void closeEvent(QCloseEvent* event) override;

private:
    void new_document();
    void open_document();
    [[nodiscard]] bool save_document();
    [[nodiscard]] bool save_document_as();
    [[nodiscard]] bool save_document_to(const std::filesystem::path& path);
    [[nodiscard]] bool confirm_destructive_action();
    void show_document_error(QString title, const DocumentError& error);
    void refresh_window_title();
    void undo();
    void redo();
    void create_surface();
    void translate_selected_numeric();
    void delete_selected_entity();
    void rename_selected_entity();
    void toggle_selected_entity_visibility();
    void handle_editor_change(EditorChange change);
    void refresh_ui_state();
    [[nodiscard]] std::optional<EntityId> selected_entity_id() const;
    [[nodiscard]] std::string suggested_surface_name() const;

    EditorSession m_session;
    std::optional<std::filesystem::path> m_document_path;
    RaylibViewportWidget* m_viewport{nullptr};
    QDockWidget* m_outliner_dock{nullptr};
    SceneOutlinerWidget* m_outliner{nullptr};
    QDockWidget* m_inspector_dock{nullptr};
    ControlPointInspectorWidget* m_inspector{nullptr};
    QAction* m_undo_action{nullptr};
    QAction* m_redo_action{nullptr};
    QAction* m_object_mode_action{nullptr};
    QAction* m_control_point_mode_action{nullptr};
    QAction* m_select_all_points_action{nullptr};
    QAction* m_select_point_row_action{nullptr};
    QAction* m_select_point_column_action{nullptr};
    QAction* m_grow_point_selection_action{nullptr};
    QAction* m_shrink_point_selection_action{nullptr};
    QAction* m_create_surface_action{nullptr};
    QAction* m_translate_action{nullptr};
    QAction* m_delete_entity_action{nullptr};
    QAction* m_rename_entity_action{nullptr};
    QAction* m_toggle_visibility_action{nullptr};
    QLabel* m_mode_status{nullptr};
    QLabel* m_selection_status{nullptr};
};
