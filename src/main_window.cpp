#include "main_window.h"

#include "qt_control_point_inspector.h"
#include "raylib_viewport_widget.h"

#include <QAction>
#include <QDockWidget>
#include <QPointer>
#include <QStatusBar>
#include <QToolBar>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {
    setWindowTitle("Nurbsman");
    setMinimumSize(800, 500);
    resize(1280, 720);

    auto* toolbar = addToolBar("Main toolbar");
    toolbar->setObjectName("mainToolbar");
    toolbar->setMovable(false);
    toolbar->addAction("Create");
    toolbar->addAction("Modify");
    toolbar->addAction("View");

    m_viewport = new RaylibViewportWidget(m_session, this);
    setCentralWidget(m_viewport);

    m_inspector_dock = new QDockWidget("Inspector", this);
    m_inspector_dock->setObjectName("inspectorDock");
    m_inspector_dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    m_inspector = new ControlPointInspectorWidget(m_session, m_inspector_dock);
    m_inspector_dock->setWidget(m_inspector);
    addDockWidget(Qt::LeftDockWidgetArea, m_inspector_dock);

    const QPointer inspector_guard(m_inspector);
    m_viewport->set_selection_changed_handler([inspector_guard] {
        if (inspector_guard != nullptr) {
            inspector_guard->refresh();
        }
    });
    const QPointer viewport_guard(m_viewport);
    m_inspector->set_change_handler([viewport_guard] {
        if (viewport_guard != nullptr) {
            viewport_guard->update();
        }
    });

    statusBar()->showMessage("Ready");
}

MainWindow::~MainWindow() {
    m_viewport->set_selection_changed_handler({});
    m_inspector->set_change_handler({});

    removeDockWidget(m_inspector_dock);
    delete m_inspector_dock;
    m_inspector_dock = nullptr;
    m_inspector = nullptr;

    delete takeCentralWidget();
    m_viewport = nullptr;
}
