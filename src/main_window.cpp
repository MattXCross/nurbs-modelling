#include "main_window.h"

#include "qt_create_surface_dialog.h"
#include "qt_control_point_inspector.h"
#include "qt_scene_outliner.h"
#include "raylib_viewport_widget.h"

#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QDockWidget>
#include <QDialog>
#include <QEvent>
#include <QIcon>
#include <QKeySequence>
#include <QKeyEvent>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QStatusBar>
#include <QToolBar>

#include <algorithm>
#include <string>
#include <vector>

namespace {

std::vector<ControlPoint> make_flat_control_net(std::size_t u_count, std::size_t v_count) {
    constexpr double surface_size = 6.0;
    std::vector<ControlPoint> points;
    points.reserve(u_count * v_count);
    for (std::size_t u = 0; u < u_count; ++u) {
        const double x = -surface_size * 0.5 + surface_size *
            static_cast<double>(u) / static_cast<double>(u_count - 1);
        for (std::size_t v = 0; v < v_count; ++v) {
            const double z = -surface_size * 0.5 + surface_size *
                static_cast<double>(v) / static_cast<double>(v_count - 1);
            points.push_back(ControlPoint{{x, 0.0, z}, 1.0});
        }
    }
    return points;
}

} // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {
    setWindowTitle("Nurbsman");
    setMinimumSize(800, 500);
    resize(1280, 720);

    m_viewport = new RaylibViewportWidget(m_session, this);
    setCentralWidget(m_viewport);

    m_outliner_dock = new QDockWidget("Scene", this);
    m_outliner_dock->setObjectName("sceneOutlinerDock");
    m_outliner_dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    m_outliner = new SceneOutlinerWidget(m_session, m_outliner_dock);
    m_outliner_dock->setWidget(m_outliner);
    addDockWidget(Qt::LeftDockWidgetArea, m_outliner_dock);

    m_inspector_dock = new QDockWidget("Inspector", this);
    m_inspector_dock->setObjectName("inspectorDock");
    m_inspector_dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    m_inspector = new ControlPointInspectorWidget(m_session, m_inspector_dock);
    m_inspector_dock->setWidget(m_inspector);
    addDockWidget(Qt::LeftDockWidgetArea, m_inspector_dock);
    splitDockWidget(m_outliner_dock, m_inspector_dock, Qt::Vertical);

    auto* quit_action = new QAction("Quit", this);
    quit_action->setShortcuts(QKeySequence::keyBindings(QKeySequence::Quit));
    connect(quit_action, &QAction::triggered, this, &QWidget::close);

    m_undo_action = new QAction(QIcon::fromTheme("edit-undo"), "Undo", this);
    m_undo_action->setShortcuts(QKeySequence::keyBindings(QKeySequence::Undo));
    connect(m_undo_action, &QAction::triggered, this, [this] { undo(); });

    m_redo_action = new QAction(QIcon::fromTheme("edit-redo"), "Redo", this);
    m_redo_action->setShortcuts(QKeySequence::keyBindings(QKeySequence::Redo));
    connect(m_redo_action, &QAction::triggered, this, [this] { redo(); });

    auto* select_action = new QAction("Select", this);
    select_action->setCheckable(true);
    select_action->setChecked(true);
    auto* tool_group = new QActionGroup(this);
    tool_group->setExclusive(true);
    tool_group->addAction(select_action);

    m_create_surface_action = new QAction(
        QIcon::fromTheme("list-add"),
        "Create Surface",
        this
    );
    connect(m_create_surface_action, &QAction::triggered, this, [this] { create_surface(); });

    m_delete_entity_action = new QAction(
        QIcon::fromTheme("edit-delete"),
        "Delete",
        this
    );
    m_delete_entity_action->setShortcut(QKeySequence::Delete);
    connect(m_delete_entity_action, &QAction::triggered, this,
        [this] { delete_selected_entity(); });

    m_rename_entity_action = new QAction("Rename", this);
    m_rename_entity_action->setShortcut(QKeySequence(Qt::Key_F2));
    connect(m_rename_entity_action, &QAction::triggered, this,
        [this] { rename_selected_entity(); });

    m_toggle_visibility_action = new QAction("Hide", this);
    connect(m_toggle_visibility_action, &QAction::triggered, this,
        [this] { toggle_selected_entity_visibility(); });

    m_outliner->setContextMenuPolicy(Qt::ActionsContextMenu);
    m_outliner->addAction(m_create_surface_action);
    m_outliner->addAction(m_rename_entity_action);
    m_outliner->addAction(m_toggle_visibility_action);
    m_outliner->addAction(m_delete_entity_action);

    auto* about_action = new QAction("About Nurbsman", this);
    connect(about_action, &QAction::triggered, this, [this] {
        QMessageBox::about(
            this,
            "About Nurbsman",
            "Nurbsman is an experimental NURBS modelling application with a Qt UI and raylib viewport."
        );
    });

    auto* file_menu = menuBar()->addMenu("File");
    file_menu->addAction(quit_action);
    auto* edit_menu = menuBar()->addMenu("Edit");
    edit_menu->addAction(m_undo_action);
    edit_menu->addAction(m_redo_action);
    edit_menu->addSeparator();
    edit_menu->addAction(m_rename_entity_action);
    edit_menu->addAction(m_toggle_visibility_action);
    edit_menu->addAction(m_delete_entity_action);
    auto* create_menu = menuBar()->addMenu("Create");
    create_menu->addAction(m_create_surface_action);
    auto* view_menu = menuBar()->addMenu("View");
    view_menu->addAction(m_outliner_dock->toggleViewAction());
    view_menu->addAction(m_inspector_dock->toggleViewAction());
    auto* help_menu = menuBar()->addMenu("Help");
    help_menu->addAction(about_action);

    auto* toolbar = addToolBar("Main toolbar");
    toolbar->setObjectName("mainToolbar");
    toolbar->setMovable(false);
    toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    toolbar->addAction(select_action);
    toolbar->addAction(m_create_surface_action);
    toolbar->addAction(m_delete_entity_action);
    toolbar->addSeparator();
    toolbar->addAction(m_undo_action);
    toolbar->addAction(m_redo_action);
    toolbar->addSeparator();
    toolbar->addAction(m_outliner_dock->toggleViewAction());
    toolbar->addAction(m_inspector_dock->toggleViewAction());

    statusBar()->showMessage("LMB Select   MMB Orbit   Shift+MMB Pan   Wheel Zoom");
    m_selection_status = new QLabel(statusBar());
    statusBar()->addPermanentWidget(m_selection_status);

    m_session.set_change_handler([this](EditorChange change) {
        handle_editor_change(change);
    });

    refresh_ui_state();
    qApp->installEventFilter(this);
}

MainWindow::~MainWindow() {
    qApp->removeEventFilter(this);
    m_session.set_change_handler({});

    removeDockWidget(m_outliner_dock);
    delete m_outliner_dock;
    m_outliner_dock = nullptr;
    m_outliner = nullptr;

    removeDockWidget(m_inspector_dock);
    delete m_inspector_dock;
    m_inspector_dock = nullptr;
    m_inspector = nullptr;

    delete takeCentralWidget();
    m_viewport = nullptr;
}

bool MainWindow::eventFilter(QObject* watched, QEvent* event) {
    const QWidget* active_window = QApplication::activeWindow();
    const bool editor_window_active =
        active_window == this || active_window == m_outliner_dock ||
        active_window == m_inspector_dock;
    if (editor_window_active && event->type() == QEvent::KeyPress) {
        auto* key_event = static_cast<QKeyEvent*>(event);
        if (key_event->matches(QKeySequence::Undo) && m_undo_action->isEnabled()) {
            m_undo_action->trigger();
            return true;
        }

        const Qt::KeyboardModifiers modifiers = key_event->modifiers();
        const bool explicit_redo =
            (key_event->key() == Qt::Key_Y && modifiers == Qt::ControlModifier) ||
            (key_event->key() == Qt::Key_Z &&
             modifiers == (Qt::ControlModifier | Qt::ShiftModifier));
        if ((key_event->matches(QKeySequence::Redo) || explicit_redo) &&
            m_redo_action->isEnabled()) {
            m_redo_action->trigger();
            return true;
        }
    }

    return QMainWindow::eventFilter(watched, event);
}

void MainWindow::undo() {
    (void)m_session.undo();
}

void MainWindow::redo() {
    (void)m_session.redo();
}

void MainWindow::create_surface() {
    CreateSurfaceDialog dialog(suggested_surface_name(), this);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    const SurfaceCreationParameters parameters = dialog.parameters();
    auto surface = NurbsSurface::create(
        parameters.u_count,
        parameters.v_count,
        parameters.u_degree,
        parameters.v_degree,
        make_flat_control_net(parameters.u_count, parameters.v_count)
    );
    if (!surface.has_value()) {
        QMessageBox::warning(this, "Create Surface", "The requested surface is invalid.");
        return;
    }

    auto entity = m_session.create_surface_entity(parameters.name, std::move(*surface));
    if (!entity.has_value()) {
        QMessageBox::warning(this, "Create Surface", "The surface could not be added.");
        return;
    }
    (void)m_session.select_entity(EntitySelection{*entity});
}

void MainWindow::delete_selected_entity() {
    const std::optional<EntityId> entity = selected_entity_id();
    if (entity.has_value()) {
        (void)m_session.delete_entity(*entity);
    }
}

void MainWindow::rename_selected_entity() {
    if (!selected_entity_id().has_value()) {
        return;
    }
    m_outliner_dock->show();
    m_outliner_dock->raise();
    m_outliner->setFocus();
    m_outliner->edit_selected_name();
}

void MainWindow::toggle_selected_entity_visibility() {
    const std::optional<EntityId> entity = selected_entity_id();
    if (!entity.has_value()) {
        return;
    }
    const SceneNode* node = m_session.scene().find_entity(*entity);
    if (node != nullptr) {
        (void)m_session.set_entity_visibility(*entity, !node->visible);
    }
}

void MainWindow::handle_editor_change(EditorChange change) {
    if (change.selection || change.entities) {
        m_outliner->refresh();
    }
    if (change.selection || change.properties) {
        m_inspector->refresh();
    }
    if (change.selection || change.entities || change.geometry) {
        m_viewport->update();
    }
    if (change.selection || change.history) {
        refresh_ui_state();
    }
}

void MainWindow::refresh_ui_state() {
    m_undo_action->setEnabled(m_session.can_undo());
    m_redo_action->setEnabled(m_session.can_redo());

    const std::optional<EntityId> selected_entity = selected_entity_id();
    const SceneNode* selected_node = selected_entity.has_value()
        ? m_session.scene().find_entity(*selected_entity)
        : nullptr;
    const bool has_entity = selected_node != nullptr;
    m_delete_entity_action->setEnabled(has_entity);
    m_rename_entity_action->setEnabled(has_entity);
    m_toggle_visibility_action->setEnabled(has_entity);
    m_toggle_visibility_action->setText(
        has_entity && !selected_node->visible ? "Show" : "Hide"
    );

    const ControlPointSelection* selection = m_session.selection().control_point();
    if (selection != nullptr) {
        const SceneNode* node = m_session.scene().find_entity(selection->entity);
        const QString name = node == nullptr
            ? QString("Unknown entity")
            : QString::fromStdString(node->name);
        m_selection_status->setText(QString("%1 / Control vertex U%2 : V%3")
            .arg(name)
            .arg(static_cast<qulonglong>(selection->u))
            .arg(static_cast<qulonglong>(selection->v)));
        return;
    }

    const EntitySelection* entity = m_session.selection().entity();
    if (entity == nullptr) {
        m_selection_status->setText("Nothing selected");
        return;
    }

    const SceneNode* node = m_session.scene().find_entity(entity->entity);
    m_selection_status->setText(node == nullptr
        ? QString("Unknown entity")
        : QString::fromStdString(node->name));
}

std::optional<EntityId> MainWindow::selected_entity_id() const {
    if (const EntitySelection* entity = m_session.selection().entity()) {
        return entity->entity;
    }
    if (const ControlPointSelection* point = m_session.selection().control_point()) {
        return point->entity;
    }
    return std::nullopt;
}

std::string MainWindow::suggested_surface_name() const {
    for (std::size_t index = 1;; ++index) {
        const std::string candidate = "Surface " + std::to_string(index);
        const bool exists = std::ranges::any_of(
            m_session.scene().nodes(),
            [&candidate](const SceneNode& node) { return node.name == candidate; }
        );
        if (!exists) {
            return candidate;
        }
    }
}
