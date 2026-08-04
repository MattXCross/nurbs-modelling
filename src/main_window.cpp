#include "main_window.h"

#include "document_io.h"
#include "qt_create_surface_dialog.h"
#include "qt_control_point_inspector.h"
#include "qt_scene_outliner.h"
#include "raylib_viewport_widget.h"

#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QCloseEvent>
#include <QComboBox>
#include <QDockWidget>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QIcon>
#include <QKeySequence>
#include <QKeyEvent>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <filesystem>
#include <numbers>
#include <span>
#include <string>
#include <utility>
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

    auto* new_action = new QAction(QIcon::fromTheme("document-new"), "New", this);
    new_action->setShortcuts(QKeySequence::keyBindings(QKeySequence::New));
    connect(new_action, &QAction::triggered, this, [this] { new_document(); });

    auto* open_action = new QAction(QIcon::fromTheme("document-open"), "Open...", this);
    open_action->setShortcuts(QKeySequence::keyBindings(QKeySequence::Open));
    connect(open_action, &QAction::triggered, this, [this] { open_document(); });

    auto* save_action = new QAction(QIcon::fromTheme("document-save"), "Save", this);
    save_action->setShortcuts(QKeySequence::keyBindings(QKeySequence::Save));
    connect(save_action, &QAction::triggered, this, [this] { (void)save_document(); });

    auto* save_as_action = new QAction(
        QIcon::fromTheme("document-save-as"),
        "Save As...",
        this
    );
    save_as_action->setShortcuts(QKeySequence::keyBindings(QKeySequence::SaveAs));
    connect(save_as_action, &QAction::triggered, this, [this] { (void)save_document_as(); });

    m_undo_action = new QAction(QIcon::fromTheme("edit-undo"), "Undo", this);
    m_undo_action->setShortcuts(QKeySequence::keyBindings(QKeySequence::Undo));
    connect(m_undo_action, &QAction::triggered, this, [this] { undo(); });

    m_redo_action = new QAction(QIcon::fromTheme("edit-redo"), "Redo", this);
    m_redo_action->setShortcuts(QKeySequence::keyBindings(QKeySequence::Redo));
    connect(m_redo_action, &QAction::triggered, this, [this] { redo(); });

    m_select_all_points_action = new QAction("Select All Control Points", this);
    m_select_all_points_action->setShortcuts(QKeySequence::keyBindings(QKeySequence::SelectAll));
    connect(m_select_all_points_action, &QAction::triggered, this, [this] {
        (void)m_session.select_all_control_points();
    });
    m_select_point_row_action = new QAction("Select Control Point Row", this);
    connect(m_select_point_row_action, &QAction::triggered, this, [this] {
        (void)m_session.select_control_point_row();
    });
    m_select_point_column_action = new QAction("Select Control Point Column", this);
    connect(m_select_point_column_action, &QAction::triggered, this, [this] {
        (void)m_session.select_control_point_column();
    });
    m_grow_point_selection_action = new QAction("Grow Control Point Selection", this);
    connect(m_grow_point_selection_action, &QAction::triggered, this, [this] {
        (void)m_session.grow_control_point_selection();
    });
    m_shrink_point_selection_action = new QAction("Shrink Control Point Selection", this);
    connect(m_shrink_point_selection_action, &QAction::triggered, this, [this] {
        (void)m_session.shrink_control_point_selection();
    });

    m_object_mode_action = new QAction("Object", this);
    m_object_mode_action->setCheckable(true);
    m_object_mode_action->setChecked(true);
    m_control_point_mode_action = new QAction("Control Points", this);
    m_control_point_mode_action->setCheckable(true);
    auto* tool_group = new QActionGroup(this);
    tool_group->setExclusive(true);
    tool_group->addAction(m_object_mode_action);
    tool_group->addAction(m_control_point_mode_action);
    connect(m_object_mode_action, &QAction::triggered, this, [this] {
        (void)m_session.set_selection_mode(EditorSession::SelectionMode::object);
    });
    connect(m_control_point_mode_action, &QAction::triggered, this, [this] {
        (void)m_session.set_selection_mode(EditorSession::SelectionMode::control_point);
    });

    m_create_surface_action = new QAction(
        QIcon::fromTheme("list-add"),
        "Create Surface",
        this
    );
    connect(m_create_surface_action, &QAction::triggered, this, [this] { create_surface(); });

    m_translate_action = new QAction("Translate...", this);
    connect(m_translate_action, &QAction::triggered, this, [this] {
        translate_selected_numeric();
    });
    m_rotate_action = new QAction("Rotate...", this);
    connect(m_rotate_action, &QAction::triggered, this, [this] { rotate_selected_numeric(); });
    m_scale_action = new QAction("Scale...", this);
    connect(m_scale_action, &QAction::triggered, this, [this] { scale_selected_numeric(); });

    m_translate_mode_action = new QAction("Move", this);
    m_rotate_mode_action = new QAction("Rotate", this);
    m_scale_mode_action = new QAction("Scale", this);
    auto* transform_group = new QActionGroup(this);
    transform_group->setExclusive(true);
    for (QAction* action : {
             m_translate_mode_action,
             m_rotate_mode_action,
             m_scale_mode_action
         }) {
        action->setCheckable(true);
        transform_group->addAction(action);
    }
    m_translate_mode_action->setChecked(true);
    connect(m_translate_mode_action, &QAction::triggered, this, [this] {
        (void)m_session.set_transform_mode(EditorSession::TransformMode::translate);
    });
    connect(m_rotate_mode_action, &QAction::triggered, this, [this] {
        (void)m_session.set_transform_mode(EditorSession::TransformMode::rotate);
    });
    connect(m_scale_mode_action, &QAction::triggered, this, [this] {
        (void)m_session.set_transform_mode(EditorSession::TransformMode::scale);
    });

    m_center_pivot_action = new QAction("Selection Center", this);
    m_primary_pivot_action = new QAction("Primary Control Point", this);
    m_origin_pivot_action = new QAction("World Origin", this);
    auto* pivot_group = new QActionGroup(this);
    pivot_group->setExclusive(true);
    for (QAction* action : {
             m_center_pivot_action,
             m_primary_pivot_action,
             m_origin_pivot_action
         }) {
        action->setCheckable(true);
        pivot_group->addAction(action);
    }
    m_center_pivot_action->setChecked(true);
    connect(m_center_pivot_action, &QAction::triggered, this, [this] {
        (void)m_session.set_pivot_mode(EditorSession::PivotMode::selection_center);
    });
    connect(m_primary_pivot_action, &QAction::triggered, this, [this] {
        (void)m_session.set_pivot_mode(EditorSession::PivotMode::primary_control_point);
    });
    connect(m_origin_pivot_action, &QAction::triggered, this, [this] {
        (void)m_session.set_pivot_mode(EditorSession::PivotMode::world_origin);
    });

    m_world_orientation_action = new QAction("World", this);
    m_local_orientation_action = new QAction("Local", this);
    auto* orientation_group = new QActionGroup(this);
    orientation_group->setExclusive(true);
    for (QAction* action : {m_world_orientation_action, m_local_orientation_action}) {
        action->setCheckable(true);
        orientation_group->addAction(action);
    }
    m_world_orientation_action->setChecked(true);
    connect(m_world_orientation_action, &QAction::triggered, this, [this] {
        (void)m_session.set_transform_orientation(EditorSession::TransformOrientation::world);
    });
    connect(m_local_orientation_action, &QAction::triggered, this, [this] {
        (void)m_session.set_transform_orientation(EditorSession::TransformOrientation::local);
    });

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

    auto* fit_all_action = new QAction("Fit All", this);
    connect(fit_all_action, &QAction::triggered, this, [this] {
        (void)m_session.fit_all(m_viewport->width(), m_viewport->height());
    });
    m_frame_selection_action = new QAction("Frame Selection", this);
    connect(m_frame_selection_action, &QAction::triggered, this, [this] {
        (void)m_session.frame_selection(m_viewport->width(), m_viewport->height());
    });

    m_perspective_action = new QAction("Perspective", this);
    m_orthographic_action = new QAction("Orthographic", this);
    auto* projection_group = new QActionGroup(this);
    projection_group->setExclusive(true);
    for (QAction* action : {m_perspective_action, m_orthographic_action}) {
        action->setCheckable(true);
        projection_group->addAction(action);
    }
    m_perspective_action->setChecked(true);
    connect(m_perspective_action, &QAction::triggered, this, [this] {
        (void)m_session.set_camera_projection(ProjectionMode::perspective);
    });
    connect(m_orthographic_action, &QAction::triggered, this, [this] {
        (void)m_session.set_camera_projection(ProjectionMode::orthographic);
    });

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
    file_menu->addAction(new_action);
    file_menu->addAction(open_action);
    file_menu->addSeparator();
    file_menu->addAction(save_action);
    file_menu->addAction(save_as_action);
    file_menu->addSeparator();
    file_menu->addAction(quit_action);
    auto* edit_menu = menuBar()->addMenu("Edit");
    edit_menu->addAction(m_undo_action);
    edit_menu->addAction(m_redo_action);
    edit_menu->addSeparator();
    edit_menu->addAction(m_select_all_points_action);
    edit_menu->addAction(m_select_point_row_action);
    edit_menu->addAction(m_select_point_column_action);
    edit_menu->addAction(m_grow_point_selection_action);
    edit_menu->addAction(m_shrink_point_selection_action);
    edit_menu->addSeparator();
    edit_menu->addAction(m_translate_action);
    edit_menu->addAction(m_rotate_action);
    edit_menu->addAction(m_scale_action);
    edit_menu->addSeparator();
    edit_menu->addAction(m_rename_entity_action);
    edit_menu->addAction(m_toggle_visibility_action);
    edit_menu->addAction(m_delete_entity_action);
    auto* create_menu = menuBar()->addMenu("Create");
    create_menu->addAction(m_create_surface_action);
    auto* transform_menu = menuBar()->addMenu("Transform");
    transform_menu->addAction(m_translate_mode_action);
    transform_menu->addAction(m_rotate_mode_action);
    transform_menu->addAction(m_scale_mode_action);
    auto* orientation_menu = transform_menu->addMenu("Orientation");
    orientation_menu->addAction(m_world_orientation_action);
    orientation_menu->addAction(m_local_orientation_action);
    auto* pivot_menu = transform_menu->addMenu("Pivot");
    pivot_menu->addAction(m_center_pivot_action);
    pivot_menu->addAction(m_primary_pivot_action);
    pivot_menu->addAction(m_origin_pivot_action);
    auto* view_menu = menuBar()->addMenu("View");
    view_menu->addAction(fit_all_action);
    view_menu->addAction(m_frame_selection_action);
    auto* standard_view_menu = view_menu->addMenu("Standard View");
    for (const auto& [label, view] : std::array{
             std::pair{"Front", StandardView::front},
             std::pair{"Back", StandardView::back},
             std::pair{"Top", StandardView::top},
             std::pair{"Bottom", StandardView::bottom},
             std::pair{"Left", StandardView::left},
             std::pair{"Right", StandardView::right}
         }) {
        QAction* action = standard_view_menu->addAction(label);
        connect(action, &QAction::triggered, this, [this, view] {
            m_session.set_standard_view(view);
        });
    }
    auto* projection_menu = view_menu->addMenu("Projection");
    projection_menu->addAction(m_perspective_action);
    projection_menu->addAction(m_orthographic_action);
    view_menu->addSeparator();
    view_menu->addAction(m_outliner_dock->toggleViewAction());
    view_menu->addAction(m_inspector_dock->toggleViewAction());
    auto* help_menu = menuBar()->addMenu("Help");
    help_menu->addAction(about_action);

    auto* toolbar = addToolBar("Main toolbar");
    toolbar->setObjectName("mainToolbar");
    toolbar->setMovable(false);
    toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    toolbar->addAction(m_object_mode_action);
    toolbar->addAction(m_control_point_mode_action);
    toolbar->addAction(m_create_surface_action);
    toolbar->addAction(m_delete_entity_action);
    toolbar->addSeparator();
    toolbar->addAction(m_translate_mode_action);
    toolbar->addAction(m_rotate_mode_action);
    toolbar->addAction(m_scale_mode_action);
    toolbar->addSeparator();
    toolbar->addAction(m_world_orientation_action);
    toolbar->addAction(m_local_orientation_action);
    toolbar->addSeparator();
    toolbar->addAction(m_undo_action);
    toolbar->addAction(m_redo_action);
    toolbar->addSeparator();
    toolbar->addAction(m_outliner_dock->toggleViewAction());
    toolbar->addAction(m_inspector_dock->toggleViewAction());

    statusBar()->showMessage(
        "LMB Select/Move   Ctrl Increment 0.5   Shift Grid 0.5   Esc Cancel   MMB Orbit   Shift+MMB Pan"
    );
    m_mode_status = new QLabel(statusBar());
    statusBar()->addPermanentWidget(m_mode_status);
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

void MainWindow::closeEvent(QCloseEvent* event) {
    if (confirm_destructive_action()) {
        event->accept();
    } else {
        event->ignore();
    }
}

void MainWindow::new_document() {
    if (!confirm_destructive_action()) {
        return;
    }
    m_session.replace_document(Scene{});
    m_document_path.reset();
    refresh_window_title();
}

void MainWindow::open_document() {
    const QString selected = QFileDialog::getOpenFileName(
        this,
        "Open Nurbsman Document",
        {},
        "Nurbsman Documents (*.nurbsman);;All Files (*)"
    );
    if (selected.isEmpty() || !confirm_destructive_action()) {
        return;
    }

    const std::filesystem::path path = selected.toStdString();
    auto loaded = load_document(path);
    if (!loaded) {
        show_document_error("Open Document", loaded.error());
        return;
    }
    m_session.replace_document(std::move(*loaded));
    m_document_path = path;
    refresh_window_title();
}

bool MainWindow::save_document() {
    if (!m_document_path.has_value()) {
        return save_document_as();
    }
    return save_document_to(*m_document_path);
}

bool MainWindow::save_document_as() {
    const QString suggested = m_document_path.has_value()
        ? QString::fromStdString(m_document_path->string())
        : QString("Untitled.nurbsman");
    const QString selected = QFileDialog::getSaveFileName(
        this,
        "Save Nurbsman Document",
        suggested,
        "Nurbsman Documents (*.nurbsman);;All Files (*)"
    );
    if (selected.isEmpty()) {
        return false;
    }

    std::filesystem::path path = selected.toStdString();
    if (!path.has_extension()) {
        path += ".nurbsman";
    }
    return save_document_to(path);
}

bool MainWindow::save_document_to(const std::filesystem::path& path) {
    m_session.commit_pending_edit();
    const auto saved = ::save_document(m_session.scene(), path);
    if (!saved) {
        show_document_error("Save Document", saved.error());
        return false;
    }
    m_document_path = path;
    m_session.mark_saved();
    refresh_window_title();
    statusBar()->showMessage(
        QString("Saved %1").arg(QString::fromStdString(path.filename().string())),
        3000
    );
    return true;
}

bool MainWindow::confirm_destructive_action() {
    if (!m_session.is_dirty()) {
        return true;
    }
    const QMessageBox::StandardButton choice = QMessageBox::warning(
        this,
        "Unsaved Changes",
        "Save changes to the current document?",
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
        QMessageBox::Save
    );
    if (choice == QMessageBox::Save) {
        return save_document();
    }
    return choice == QMessageBox::Discard;
}

void MainWindow::show_document_error(QString title, const DocumentError& error) {
    const QString path = error.file.empty()
        ? QString("Document")
        : QString::fromStdString(error.file.string());
    const QString field = error.field.empty()
        ? QString()
        : QString("\nField: %1").arg(QString::fromStdString(error.field));
    QMessageBox::critical(
        this,
        std::move(title),
        QString("%1%2\n\n%3")
            .arg(path, field, QString::fromStdString(error.message))
    );
}

void MainWindow::refresh_window_title() {
    const std::string file_name = m_document_path.has_value()
        ? m_document_path->filename().string()
        : "Untitled";
    setWindowTitle(QString("%1%2 - Nurbsman")
        .arg(QString::fromStdString(file_name))
        .arg(m_session.is_dirty() ? "*" : ""));
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

void MainWindow::translate_selected_numeric() {
    if (!m_session.selection_pivot().has_value()) {
        return;
    }
    QDialog dialog(this);
    dialog.setWindowTitle("Translate Selection");
    auto* form = new QFormLayout;
    std::array<QDoubleSpinBox*, 3> fields{};
    for (std::size_t index = 0; index < fields.size(); ++index) {
        fields[index] = new QDoubleSpinBox(&dialog);
        fields[index]->setRange(-1'000'000'000.0, 1'000'000'000.0);
        fields[index]->setDecimals(6);
        fields[index]->setSingleStep(0.5);
    }
    form->addRow("Delta X", fields[0]);
    form->addRow("Delta Y", fields[1]);
    form->addRow("Delta Z", fields[2]);
    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
        &dialog
    );
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    auto* layout = new QVBoxLayout(&dialog);
    layout->addLayout(form);
    layout->addWidget(buttons);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }
    const auto frame = m_session.transform_frame();
    if (!frame) {
        return;
    }
    const cad::Vector3 delta = frame->x * fields[0]->value() +
        frame->y * fields[1]->value() + frame->z * fields[2]->value();
    if (!m_session.translate_selection(delta)) {
        QMessageBox::warning(this, "Translate Selection", "The translation could not be applied.");
    }
}

void MainWindow::rotate_selected_numeric() {
    if (!m_session.selection_pivot()) {
        return;
    }
    QDialog dialog(this);
    dialog.setWindowTitle("Rotate Selection");
    auto* axis = new QComboBox(&dialog);
    axis->addItems({"X Axis", "Y Axis", "Z Axis", "Camera Plane"});
    auto* angle = new QDoubleSpinBox(&dialog);
    angle->setRange(-36000.0, 36000.0);
    angle->setDecimals(4);
    angle->setSingleStep(15.0);
    auto* form = new QFormLayout;
    form->addRow("Axis", axis);
    form->addRow("Angle (degrees)", angle);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    auto* layout = new QVBoxLayout(&dialog);
    layout->addLayout(form);
    layout->addWidget(buttons);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }
    const auto frame = m_session.transform_frame();
    if (!frame) {
        return;
    }
    cad::Vector3 rotation_axis;
    switch (axis->currentIndex()) {
        case 0: rotation_axis = frame->x; break;
        case 1: rotation_axis = frame->y; break;
        case 2: rotation_axis = frame->z; break;
        default:
            rotation_axis = cad::normalized(m_session.camera().target - m_session.camera().position)
                .value_or(cad::Vector3{});
            break;
    }
    if (!m_session.rotate_selection(
            rotation_axis,
            angle->value() * std::numbers::pi / 180.0
        )) {
        QMessageBox::warning(this, "Rotate Selection", "The rotation could not be applied.");
    }
}

void MainWindow::scale_selected_numeric() {
    if (!m_session.selection_pivot()) {
        return;
    }
    QDialog dialog(this);
    dialog.setWindowTitle("Scale Selection");
    auto* constraint = new QComboBox(&dialog);
    constraint->addItems({"Uniform", "X Axis", "Y Axis", "Z Axis"});
    auto* factor = new QDoubleSpinBox(&dialog);
    factor->setRange(0.001, 1'000'000.0);
    factor->setDecimals(6);
    factor->setValue(1.0);
    factor->setSingleStep(0.1);
    auto* form = new QFormLayout;
    form->addRow("Constraint", constraint);
    form->addRow("Factor", factor);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    auto* layout = new QVBoxLayout(&dialog);
    layout->addLayout(form);
    layout->addWidget(buttons);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }
    const std::array constraints = {
        EditorSession::ScaleConstraint::uniform,
        EditorSession::ScaleConstraint::x,
        EditorSession::ScaleConstraint::y,
        EditorSession::ScaleConstraint::z
    };
    if (!m_session.scale_selection(
            constraints[static_cast<std::size_t>(constraint->currentIndex())],
            factor->value()
        )) {
        QMessageBox::warning(this, "Scale Selection", "The scale could not be applied.");
    }
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
    if (change.selection || change.entities || change.properties) {
        m_inspector->refresh();
    }
    if (change.selection || change.entities || change.geometry || change.hover ||
        change.interaction_mode || change.camera) {
        m_viewport->update();
    }
    if (change.selection || change.history || change.interaction_mode || change.camera) {
        refresh_ui_state();
    }
}

void MainWindow::refresh_ui_state() {
    const bool object_mode =
        m_session.selection_mode() == EditorSession::SelectionMode::object;
    const QSignalBlocker object_blocker(m_object_mode_action);
    const QSignalBlocker point_blocker(m_control_point_mode_action);
    m_object_mode_action->setChecked(object_mode);
    m_control_point_mode_action->setChecked(!object_mode);
    const char* transform_name = "Move";
    switch (m_session.transform_mode()) {
        case EditorSession::TransformMode::translate: transform_name = "Move"; break;
        case EditorSession::TransformMode::rotate: transform_name = "Rotate"; break;
        case EditorSession::TransformMode::scale: transform_name = "Scale"; break;
    }
    const char* orientation_name = m_session.transform_orientation() ==
        EditorSession::TransformOrientation::world ? "World" : "Local";
    m_mode_status->setText(QString("Mode: %1 / %2 / %3")
        .arg(object_mode ? "Object" : "Control Points", transform_name, orientation_name));
    const bool has_point = m_session.selection().control_point() != nullptr;
    m_select_all_points_action->setEnabled(!object_mode && selected_entity_id().has_value());
    m_select_point_row_action->setEnabled(!object_mode && has_point);
    m_select_point_column_action->setEnabled(!object_mode && has_point);
    m_grow_point_selection_action->setEnabled(!object_mode && has_point);
    m_shrink_point_selection_action->setEnabled(!object_mode && has_point);
    m_translate_action->setEnabled(m_session.selection_pivot().has_value());
    m_rotate_action->setEnabled(m_session.selection_pivot().has_value());
    m_scale_action->setEnabled(m_session.selection_pivot().has_value());
    const EditorSession::TransformMode transform_mode = m_session.transform_mode();
    m_translate_mode_action->setChecked(transform_mode == EditorSession::TransformMode::translate);
    m_rotate_mode_action->setChecked(transform_mode == EditorSession::TransformMode::rotate);
    m_scale_mode_action->setChecked(transform_mode == EditorSession::TransformMode::scale);
    const EditorSession::PivotMode pivot_mode = m_session.pivot_mode();
    m_center_pivot_action->setChecked(pivot_mode == EditorSession::PivotMode::selection_center);
    m_primary_pivot_action->setChecked(pivot_mode == EditorSession::PivotMode::primary_control_point);
    m_origin_pivot_action->setChecked(pivot_mode == EditorSession::PivotMode::world_origin);
    const bool world_orientation = m_session.transform_orientation() ==
        EditorSession::TransformOrientation::world;
    m_world_orientation_action->setChecked(world_orientation);
    m_local_orientation_action->setChecked(!world_orientation);

    m_undo_action->setEnabled(m_session.can_undo());
    m_redo_action->setEnabled(m_session.can_redo());
    const std::string undo_description = m_session.undo_description();
    const std::string redo_description = m_session.redo_description();
    m_undo_action->setText(undo_description.empty()
        ? "Undo"
        : QString("Undo %1").arg(QString::fromStdString(undo_description)));
    m_redo_action->setText(redo_description.empty()
        ? "Redo"
        : QString("Redo %1").arg(QString::fromStdString(redo_description)));
    refresh_window_title();

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
    m_frame_selection_action->setEnabled(!m_session.selection().empty());
    const bool perspective = m_session.camera().projection == ProjectionMode::perspective;
    m_perspective_action->setChecked(perspective);
    m_orthographic_action->setChecked(!perspective);

    const std::span selected_points = m_session.selection().control_points();
    if (!selected_points.empty()) {
        const ControlPointSelection& primary = selected_points.back();
        const SceneNode* node = m_session.scene().find_entity(primary.entity);
        const QString name = node == nullptr
            ? QString("Unknown entity")
            : QString::fromStdString(node->name);
        m_selection_status->setText(selected_points.size() == 1
            ? QString("%1 / Control vertex U%2 : V%3")
                .arg(name)
                .arg(static_cast<qulonglong>(primary.u))
                .arg(static_cast<qulonglong>(primary.v))
            : QString("%1 control points selected").arg(
                static_cast<qulonglong>(selected_points.size())
            ));
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
    return m_session.selected_entity_id();
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
