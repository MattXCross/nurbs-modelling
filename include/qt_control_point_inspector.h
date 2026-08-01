#pragma once

#include "editor_session.h"

#include <QWidget>

#include <array>
#include <functional>

class QDoubleSpinBox;
class QLabel;

class ControlPointInspectorWidget final : public QWidget {
public:
    using ChangeHandler = std::function<void()>;

    explicit ControlPointInspectorWidget(
        EditorSession& session,
        QWidget* parent = nullptr
    );

    void set_change_handler(ChangeHandler handler);
    void refresh();

private:
    void value_changed(std::size_t field_index, double value);
    void editing_finished(std::size_t field_index);

    EditorSession& m_session;
    QLabel* m_selection_label{nullptr};
    QWidget* m_fields_widget{nullptr};
    std::array<QDoubleSpinBox*, 4> m_fields{};
    std::array<bool, 4> m_editing{};
    ChangeHandler m_change_handler;
};
