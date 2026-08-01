#include "qt_control_point_inspector.h"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <array>
#include <utility>

namespace {

constexpr std::array fields = {
    EditorSession::ControlPointField::position_x,
    EditorSession::ControlPointField::position_y,
    EditorSession::ControlPointField::position_z,
    EditorSession::ControlPointField::weight
};

constexpr std::array field_labels = {
    "Position X",
    "Position Y",
    "Position Z",
    "Weight W"
};

} // namespace

ControlPointInspectorWidget::ControlPointInspectorWidget(
    EditorSession& session,
    QWidget* parent
)
    : QWidget(parent),
      m_session(session) {
    m_selection_label = new QLabel(this);
    m_selection_label->setWordWrap(true);

    m_fields_widget = new QWidget(this);
    auto* form = new QFormLayout(m_fields_widget);
    form->setContentsMargins(0, 0, 0, 0);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    for (std::size_t index = 0; index < m_fields.size(); ++index) {
        auto* field = new QDoubleSpinBox(m_fields_widget);
        field->setDecimals(6);
        field->setSingleStep(0.1);
        field->setAccelerated(true);
        if (fields[index] == EditorSession::ControlPointField::weight) {
            field->setRange(0.000001, 1'000'000'000.0);
        } else {
            field->setRange(-1'000'000'000.0, 1'000'000'000.0);
        }

        connect(field, &QDoubleSpinBox::valueChanged, this, [this, index](double value) {
            value_changed(index, value);
        });
        connect(field, &QDoubleSpinBox::editingFinished, this, [this, index] {
            editing_finished(index);
        });

        m_fields[index] = field;
        form->addRow(field_labels[index], field);
    }

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(12, 12, 12, 12);
    layout->addWidget(m_selection_label);
    layout->addSpacing(8);
    layout->addWidget(m_fields_widget);
    layout->addStretch();

    refresh();
}

void ControlPointInspectorWidget::set_change_handler(ChangeHandler handler) {
    m_change_handler = std::move(handler);
}

void ControlPointInspectorWidget::refresh() {
    m_editing.fill(false);
    const ControlPointSelection* selection = m_session.selection().control_point();
    const ControlPoint* point = m_session.selected_control_point();
    const bool has_selection = selection != nullptr && point != nullptr;
    m_fields_widget->setEnabled(has_selection);

    if (!has_selection) {
        m_selection_label->setText("Select a control point to inspect its properties.");
        return;
    }

    m_selection_label->setText(QString("Control vertex U%1 : V%2")
        .arg(static_cast<qulonglong>(selection->u))
        .arg(static_cast<qulonglong>(selection->v)));

    const std::array values = {
        point->position.x,
        point->position.y,
        point->position.z,
        point->weight
    };
    for (std::size_t index = 0; index < m_fields.size(); ++index) {
        const QSignalBlocker blocker(m_fields[index]);
        m_fields[index]->setValue(values[index]);
    }
}

void ControlPointInspectorWidget::value_changed(std::size_t field_index, double value) {
    if (!m_editing[field_index]) {
        if (!m_session.begin_control_point_edit(fields[field_index])) {
            refresh();
            return;
        }
        m_editing[field_index] = true;
    }

    if (!m_session.preview_control_point_edit(fields[field_index], value)) {
        m_editing[field_index] = false;
        refresh();
    }
    if (m_change_handler) {
        m_change_handler();
    }
}

void ControlPointInspectorWidget::editing_finished(std::size_t field_index) {
    if (!m_editing[field_index]) {
        return;
    }

    m_session.finish_control_point_edit(fields[field_index]);
    m_editing[field_index] = false;
    if (m_change_handler) {
        m_change_handler();
    }
}
