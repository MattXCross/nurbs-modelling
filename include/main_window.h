#pragma once

#include "editor_session.h"

#include <QMainWindow>

class MainWindow final : public QMainWindow {
public:
    explicit MainWindow(QWidget* parent = nullptr);

private:
    EditorSession m_session;
};
