#include "main_window.h"

#include <QAction>
#include <QDockWidget>
#include <QLabel>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWidget>

namespace {

QWidget* create_viewport_placeholder(QWidget* parent) {
    auto* viewport = new QWidget(parent);
    viewport->setObjectName("viewportPlaceholder");
    viewport->setStyleSheet(
        "#viewportPlaceholder { background-color: #10141a; }"
        "#viewportTitle { color: #d7dee8; font-size: 18px; font-weight: 600; }"
        "#viewportMessage { color: #8995a5; }"
    );

    auto* title = new QLabel("Raylib viewport", viewport);
    title->setObjectName("viewportTitle");
    title->setAlignment(Qt::AlignCenter);

    auto* message = new QLabel("Viewport integration pending", viewport);
    message->setObjectName("viewportMessage");
    message->setAlignment(Qt::AlignCenter);

    auto* layout = new QVBoxLayout(viewport);
    layout->addStretch();
    layout->addWidget(title);
    layout->addWidget(message);
    layout->addStretch();
    return viewport;
}

QWidget* create_inspector_placeholder(QWidget* parent) {
    auto* inspector = new QWidget(parent);
    auto* message = new QLabel("Select a control point to inspect its properties.", inspector);
    message->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    message->setWordWrap(true);

    auto* layout = new QVBoxLayout(inspector);
    layout->setContentsMargins(12, 12, 12, 12);
    layout->addWidget(message);
    return inspector;
}

} // namespace

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

    setCentralWidget(create_viewport_placeholder(this));

    auto* inspector_dock = new QDockWidget("Inspector", this);
    inspector_dock->setObjectName("inspectorDock");
    inspector_dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    inspector_dock->setWidget(create_inspector_placeholder(inspector_dock));
    addDockWidget(Qt::LeftDockWidgetArea, inspector_dock);

    statusBar()->showMessage("Ready");
}
