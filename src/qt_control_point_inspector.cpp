#include "qt_control_point_inspector.h"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <array>

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

    m_entity_fields_widget = new QWidget(this);
    auto* entity_form = new QFormLayout(m_entity_fields_widget);
    entity_form->setContentsMargins(0, 0, 0, 0);
    entity_form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    m_entity_name = new QLineEdit(m_entity_fields_widget);
    connect(m_entity_name, &QLineEdit::editingFinished, this,
        [this] { entity_name_changed(); });
    entity_form->addRow("Name", m_entity_name);

    m_entity_visible = new QCheckBox(m_entity_fields_widget);
    connect(m_entity_visible, &QCheckBox::toggled, this,
        [this](bool visible) { entity_visibility_changed(visible); });
    entity_form->addRow("Visible", m_entity_visible);

    m_entity_id = new QLabel(m_entity_fields_widget);
    m_entity_type = new QLabel(m_entity_fields_widget);
    m_control_net_size = new QLabel(m_entity_fields_widget);
    m_surface_degree = new QLabel(m_entity_fields_widget);
    entity_form->addRow("Entity ID", m_entity_id);
    entity_form->addRow("Type", m_entity_type);
    entity_form->addRow("Control net", m_control_net_size);
    entity_form->addRow("Degree", m_surface_degree);

    m_control_point_fields_widget = new QWidget(this);
    auto* form = new QFormLayout(m_control_point_fields_widget);
    form->setContentsMargins(0, 0, 0, 0);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    for (std::size_t index = 0; index < m_fields.size(); ++index) {
        auto* field = new QDoubleSpinBox(m_control_point_fields_widget);
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
    layout->addWidget(m_entity_fields_widget);
    layout->addWidget(m_control_point_fields_widget);
    layout->addStretch();

    refresh();
}

void ControlPointInspectorWidget::refresh() {
    m_editing.fill(false);
    const EntitySelection* entity_selection = m_session.selection().entity();
    const SceneNode* entity = entity_selection == nullptr
        ? nullptr
        : m_session.scene().find_entity(entity_selection->entity);
    m_inspected_entity = entity == nullptr
        ? std::nullopt
        : std::optional{entity->id};
    m_entity_fields_widget->setVisible(entity != nullptr);

    const ControlPointSelection* point_selection = m_session.selection().control_point();
    const ControlPoint* point = m_session.selected_control_point();
    const bool has_control_point = point_selection != nullptr && point != nullptr;
    m_control_point_fields_widget->setVisible(has_control_point);

    if (entity != nullptr) {
        m_selection_label->setText("NURBS surface");
        const QSignalBlocker name_blocker(m_entity_name);
        const QSignalBlocker visibility_blocker(m_entity_visible);
        m_entity_name->setText(QString::fromStdString(entity->name));
        m_entity_visible->setChecked(entity->visible);
        m_entity_id->setText(QString::number(static_cast<qulonglong>(entity->id.value)));
        m_entity_type->setText("NURBS surface");
        m_control_net_size->setText(QString("%1 x %2")
            .arg(static_cast<qulonglong>(entity->surface->u_count()))
            .arg(static_cast<qulonglong>(entity->surface->v_count())));
        m_surface_degree->setText(QString("%1 x %2")
            .arg(static_cast<qulonglong>(entity->surface->u_degree()))
            .arg(static_cast<qulonglong>(entity->surface->v_degree())));
        return;
    }

    if (!has_control_point) {
        m_selection_label->setText("Select an entity or control point to inspect its properties.");
        return;
    }

    const SceneNode* owner = m_session.scene().find_entity(point_selection->entity);
    const QString owner_name = owner == nullptr
        ? QString("Unknown entity")
        : QString::fromStdString(owner->name);
    m_selection_label->setText(QString("%1 / Control vertex U%2 : V%3")
        .arg(owner_name)
        .arg(static_cast<qulonglong>(point_selection->u))
        .arg(static_cast<qulonglong>(point_selection->v)));

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

void ControlPointInspectorWidget::entity_name_changed() {
    if (!m_inspected_entity.has_value()) {
        refresh();
        return;
    }
    if (!m_session.rename_entity(*m_inspected_entity, m_entity_name->text().toStdString())
             .has_value()) {
        refresh();
    }
}

void ControlPointInspectorWidget::entity_visibility_changed(bool visible) {
    if (!m_inspected_entity.has_value()) {
        refresh();
        return;
    }
    if (!m_session.set_entity_visibility(*m_inspected_entity, visible).has_value()) {
        refresh();
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
}

void ControlPointInspectorWidget::editing_finished(std::size_t field_index) {
    if (!m_editing[field_index]) {
        return;
    }

    m_session.finish_control_point_edit(fields[field_index]);
    m_editing[field_index] = false;
}
