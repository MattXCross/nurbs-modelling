#pragma once

#include <QDialog>

#include <cstddef>
#include <string>

class QLineEdit;
class QSpinBox;

struct SurfaceCreationParameters {
    std::string name;
    std::size_t u_count{4};
    std::size_t v_count{4};
    std::size_t u_degree{3};
    std::size_t v_degree{3};
};

class CreateSurfaceDialog final : public QDialog {
public:
    explicit CreateSurfaceDialog(std::string suggested_name, QWidget* parent = nullptr);

    [[nodiscard]] SurfaceCreationParameters parameters() const;

private:
    void update_degree_ranges();

    QLineEdit* m_name{nullptr};
    QSpinBox* m_u_count{nullptr};
    QSpinBox* m_v_count{nullptr};
    QSpinBox* m_u_degree{nullptr};
    QSpinBox* m_v_degree{nullptr};
};
