#include "qt_scene_outliner.h"

#include <QContextMenuEvent>
#include <QHeaderView>
#include <QMetaObject>
#include <QSignalBlocker>
#include <QTreeWidgetItem>
#include <QVariant>

#include <optional>

namespace {

constexpr int name_column = 0;
constexpr int visibility_column = 1;
constexpr int entity_id_role = Qt::UserRole;

std::optional<EntityId> selected_entity(const Selection& selection) {
    if (const auto* entity = std::get_if<EntitySelection>(&selection)) {
        return entity->entity;
    }
    if (const auto* points = std::get_if<ControlPointSelections>(&selection);
        points != nullptr && !points->empty()) {
        return points->back().entity;
    }
    return std::nullopt;
}

} // namespace

SceneOutlinerWidget::SceneOutlinerWidget(EditorSession& session, QWidget* parent)
    : QTreeWidget(parent),
      m_session(session) {
    setColumnCount(2);
    setHeaderLabels({"Name", "Visible"});
    setRootIsDecorated(false);
    setUniformRowHeights(true);
    setSelectionMode(QAbstractItemView::SingleSelection);
    setSelectionBehavior(QAbstractItemView::SelectRows);
    setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed);
    header()->setSectionResizeMode(name_column, QHeaderView::Stretch);
    header()->setSectionResizeMode(visibility_column, QHeaderView::ResizeToContents);

    connect(this, &QTreeWidget::currentItemChanged, this,
        [this](QTreeWidgetItem* current) { current_item_changed(current); });
    connect(this, &QTreeWidget::itemClicked, this,
        [this](QTreeWidgetItem* item) { current_item_changed(item); });
    connect(this, &QTreeWidget::itemChanged, this,
        [this](QTreeWidgetItem* item, int column) { item_changed(item, column); });

    refresh();
}

bool SceneOutlinerWidget::edit(
    const QModelIndex& index,
    EditTrigger trigger,
    QEvent* event
) {
    if (index.column() != name_column) {
        return false;
    }
    return QTreeWidget::edit(index, trigger, event);
}

void SceneOutlinerWidget::contextMenuEvent(QContextMenuEvent* event) {
    if (QTreeWidgetItem* item = itemAt(event->pos())) {
        setCurrentItem(item);
        current_item_changed(item);
    } else {
        setCurrentItem(nullptr);
        current_item_changed(nullptr);
    }
    QTreeWidget::contextMenuEvent(event);
}

void SceneOutlinerWidget::refresh() {
    if (m_handling_change) {
        request_deferred_refresh();
        return;
    }

    m_refresh_pending = false;
    const QSignalBlocker blocker(this);
    clear();

    const std::optional<EntityId> selected = selected_entity(m_session.selection().current());
    QTreeWidgetItem* selected_item = nullptr;
    for (const SceneNode& node : m_session.scene().nodes()) {
        auto* item = new QTreeWidgetItem(this);
        item->setData(
            name_column,
            entity_id_role,
            QVariant::fromValue(static_cast<qulonglong>(node.id.value))
        );
        item->setText(name_column, QString::fromStdString(node.name));
        item->setCheckState(
            visibility_column,
            node.visible ? Qt::Checked : Qt::Unchecked
        );
        item->setFlags(
            item->flags() |
            Qt::ItemIsEditable |
            Qt::ItemIsUserCheckable
        );

        if (selected.has_value() && *selected == node.id) {
            selected_item = item;
        }
    }

    setCurrentItem(selected_item);
    if (selected_item == nullptr) {
        clearSelection();
    }
}

void SceneOutlinerWidget::edit_selected_name() {
    if (QTreeWidgetItem* item = currentItem()) {
        editItem(item, name_column);
    }
}

void SceneOutlinerWidget::current_item_changed(QTreeWidgetItem* current) {
    if (m_handling_change) {
        return;
    }

    m_handling_change = true;
    if (current == nullptr) {
        (void)m_session.clear_selection();
    } else if (!m_session.select_entity(EntitySelection{entity_id(*current)})) {
        const EntitySelection* selection = m_session.selection().entity();
        if (selection == nullptr || selection->entity != entity_id(*current)) {
            request_deferred_refresh();
        }
    }
    m_handling_change = false;
}

void SceneOutlinerWidget::item_changed(QTreeWidgetItem* item, int column) {
    if (m_handling_change || item == nullptr) {
        return;
    }

    m_handling_change = true;
    const EntityId id = entity_id(*item);
    bool failed = false;
    if (column == name_column) {
        failed = !m_session.rename_entity(id, item->text(name_column).toStdString()).has_value();
    } else if (column == visibility_column) {
        failed = !m_session.set_entity_visibility(
            id,
            item->checkState(visibility_column) == Qt::Checked
        ).has_value();
    }
    if (failed) {
        request_deferred_refresh();
    }
    m_handling_change = false;
}

void SceneOutlinerWidget::request_deferred_refresh() {
    m_refresh_pending = true;
    if (m_refresh_queued) {
        return;
    }

    m_refresh_queued = true;
    QMetaObject::invokeMethod(this, [this] {
        m_refresh_queued = false;
        if (m_refresh_pending) {
            refresh();
        }
    }, Qt::QueuedConnection);
}

EntityId SceneOutlinerWidget::entity_id(const QTreeWidgetItem& item) {
    return EntityId{item.data(name_column, entity_id_role).toULongLong()};
}
