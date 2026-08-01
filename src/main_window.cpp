#include "main_window.h"

#include "qt_control_point_inspector.h"
#include "qt_scene_outliner.h"
#include "raylib_viewport_widget.h"

#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QDockWidget>
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

    auto* create_action = new QAction("Create", this);
    create_action->setEnabled(false);
    create_action->setToolTip("Create tools are not available yet");
    auto* modify_action = new QAction("Modify", this);
    modify_action->setEnabled(false);
    modify_action->setToolTip("Modify tools are not available yet");

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
    toolbar->addAction(create_action);
    toolbar->addAction(modify_action);
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
