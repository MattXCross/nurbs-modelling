#include "qt_create_surface_dialog.h"

#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

#include <utility>

CreateSurfaceDialog::CreateSurfaceDialog(std::string suggested_name, QWidget* parent)
    : QDialog(parent) {
    setWindowTitle("Create NURBS Surface");

    m_name = new QLineEdit(QString::fromStdString(std::move(suggested_name)), this);
    m_name->selectAll();

    m_u_count = new QSpinBox(this);
    m_v_count = new QSpinBox(this);
    m_u_degree = new QSpinBox(this);
    m_v_degree = new QSpinBox(this);
    for (QSpinBox* count : {m_u_count, m_v_count}) {
        count->setRange(2, 64);
        count->setValue(4);
    }
    for (QSpinBox* degree : {m_u_degree, m_v_degree}) {
        degree->setMinimum(1);
        degree->setValue(3);
    }
    update_degree_ranges();

    connect(m_u_count, &QSpinBox::valueChanged, this, [this] { update_degree_ranges(); });
    connect(m_v_count, &QSpinBox::valueChanged, this, [this] { update_degree_ranges(); });

    auto* form = new QFormLayout;
    form->addRow("Name", m_name);
    form->addRow("U control points", m_u_count);
    form->addRow("V control points", m_v_count);
    form->addRow("U degree", m_u_degree);
    form->addRow("V degree", m_v_degree);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
        this
    );
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    QPushButton* create_button = buttons->button(QDialogButtonBox::Ok);
    connect(m_name, &QLineEdit::textChanged, this, [create_button](const QString& text) {
        create_button->setEnabled(!text.trimmed().isEmpty());
    });

    auto* layout = new QVBoxLayout(this);
    layout->addLayout(form);
    layout->addWidget(buttons);
}

SurfaceCreationParameters CreateSurfaceDialog::parameters() const {
    return {
        .name = m_name->text().trimmed().toStdString(),
        .u_count = static_cast<std::size_t>(m_u_count->value()),
        .v_count = static_cast<std::size_t>(m_v_count->value()),
        .u_degree = static_cast<std::size_t>(m_u_degree->value()),
        .v_degree = static_cast<std::size_t>(m_v_degree->value())
    };
}

void CreateSurfaceDialog::update_degree_ranges() {
    m_u_degree->setMaximum(m_u_count->value() - 1);
    m_v_degree->setMaximum(m_v_count->value() - 1);
}
