#pragma once

#include "editor_session.h"

#include <QWidget>

#include <array>
#include <optional>

class QDoubleSpinBox;
class QCheckBox;
class QLabel;
class QLineEdit;

class ControlPointInspectorWidget final : public QWidget {
public:
    explicit ControlPointInspectorWidget(
        EditorSession& session,
        QWidget* parent = nullptr
    );

    void refresh();

private:
    void entity_name_changed();
    void entity_visibility_changed(bool visible);
    void value_changed(std::size_t field_index, double value);
    void editing_finished(std::size_t field_index);

    EditorSession& m_session;
    QLabel* m_selection_label{nullptr};
    QWidget* m_entity_fields_widget{nullptr};
    QLineEdit* m_entity_name{nullptr};
    QCheckBox* m_entity_visible{nullptr};
    QLabel* m_entity_id{nullptr};
    QLabel* m_entity_type{nullptr};
    QLabel* m_control_net_size{nullptr};
    QLabel* m_surface_degree{nullptr};
    std::optional<EntityId> m_inspected_entity;
    QWidget* m_control_point_fields_widget{nullptr};
    std::array<QDoubleSpinBox*, 4> m_fields{};
    std::array<bool, 4> m_editing{};
};
