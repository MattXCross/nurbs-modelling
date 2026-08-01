#pragma once

#include "editor_session.h"

#include <QTreeWidget>

class QTreeWidgetItem;
class QContextMenuEvent;
class QEvent;
class QModelIndex;

class SceneOutlinerWidget final : public QTreeWidget {
public:
    explicit SceneOutlinerWidget(EditorSession& session, QWidget* parent = nullptr);

    void refresh();
    void edit_selected_name();

protected:
    bool edit(const QModelIndex& index, EditTrigger trigger, QEvent* event) override;
    void contextMenuEvent(QContextMenuEvent* event) override;

private:
    void current_item_changed(QTreeWidgetItem* current);
    void item_changed(QTreeWidgetItem* item, int column);
    void request_deferred_refresh();
    [[nodiscard]] static EntityId entity_id(const QTreeWidgetItem& item);

    EditorSession& m_session;
    bool m_handling_change{false};
    bool m_refresh_pending{false};
    bool m_refresh_queued{false};
};
