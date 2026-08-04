#include "transform_controller.h"

#include "nurbs_surface.h"

#include <array>
#include <cmath>
#include <memory>
#include <numeric>
#include <ranges>
#include <string>
#include <utility>

namespace {

bool apply_points(
    Scene& scene,
    const std::vector<ControlPointSelection>& selections,
    const std::vector<ControlPoint>& points
) {
    if (selections.size() != points.size()) {
        return false;
    }
    std::vector<ControlPoint> originals;
    originals.reserve(selections.size());
    for (const ControlPointSelection selection : selections) {
        const ControlPoint* point = scene.resolve(selection);
        if (point == nullptr) {
            return false;
        }
        originals.push_back(*point);
    }
    for (std::size_t index = 0; index < selections.size(); ++index) {
        if (!scene.set_control_point(selections[index], points[index]).has_value()) {
            for (std::size_t rollback = 0; rollback < index; ++rollback) {
                (void)scene.set_control_point(selections[rollback], originals[rollback]);
            }
            return false;
        }
    }
    return true;
}

class TransformCommand final : public ICommand {
public:
    TransformCommand(
        std::string description,
        Scene& scene,
        std::vector<ControlPointSelection> selections,
        std::vector<ControlPoint> initial,
        std::vector<ControlPoint> final
    )
        : m_description(std::move(description)),
          m_scene(scene),
          m_selections(std::move(selections)),
          m_initial(std::move(initial)),
          m_final(std::move(final)) {}

    [[nodiscard]] std::string_view description() const override { return m_description; }
    bool undo() override { return apply_points(m_scene, m_selections, m_initial); }
    bool redo() override { return apply_points(m_scene, m_selections, m_final); }

private:
    std::string m_description;
    Scene& m_scene;
    std::vector<ControlPointSelection> m_selections;
    std::vector<ControlPoint> m_initial;
    std::vector<ControlPoint> m_final;
};

std::optional<std::array<cad::Vector3, 3>> make_frame(
    cad::Vector3 first,
    cad::Vector3 second
) {
    const auto x = cad::normalized(first);
    if (!x) {
        return std::nullopt;
    }
    const auto y = cad::normalized(second - *x * cad::dot(second, *x));
    if (!y) {
        return std::nullopt;
    }
    const auto z = cad::normalized(cad::cross(*x, *y));
    return z ? std::optional{std::array{*x, *y, *z}} : std::nullopt;
}

} // namespace

bool TransformController::set_mode(TransformMode mode) {
    if (m_mode == mode) {
        return false;
    }
    (void)cancel();
    m_mode = mode;
    return true;
}

bool TransformController::set_pivot_mode(PivotMode mode) {
    if (m_pivot_mode == mode) {
        return false;
    }
    (void)cancel();
    m_pivot_mode = mode;
    return true;
}

bool TransformController::set_orientation(TransformOrientation orientation) {
    if (m_orientation == orientation) {
        return false;
    }
    (void)cancel();
    m_orientation = orientation;
    return true;
}

std::vector<ControlPointSelection> TransformController::targets() const {
    const auto points = m_selection.control_points();
    if (!points.empty()) {
        return {points.begin(), points.end()};
    }
    const EntitySelection* entity = m_selection.entity();
    const SceneNode* node = entity == nullptr ? nullptr : m_scene.find_entity(entity->entity);
    if (node == nullptr || node->surface == nullptr) {
        return {};
    }
    std::vector<ControlPointSelection> result;
    result.reserve(node->surface->u_count() * node->surface->v_count());
    for (std::size_t u = 0; u < node->surface->u_count(); ++u) {
        for (std::size_t v = 0; v < node->surface->v_count(); ++v) {
            result.push_back({entity->entity, u, v});
        }
    }
    return result;
}

std::vector<ControlPoint> TransformController::capture(
    const std::vector<ControlPointSelection>& selections
) const {
    std::vector<ControlPoint> result;
    result.reserve(selections.size());
    for (const ControlPointSelection selection : selections) {
        const ControlPoint* point = m_scene.resolve(selection);
        if (point == nullptr) {
            return {};
        }
        result.push_back(*point);
    }
    return result;
}

std::optional<cad::Point3> TransformController::pivot() const {
    const std::vector<ControlPointSelection> selected = targets();
    if (selected.empty()) {
        return std::nullopt;
    }
    if (m_pivot_mode == PivotMode::world_origin) {
        return cad::Point3{};
    }
    if (m_pivot_mode == PivotMode::primary_control_point) {
        const ControlPointSelection* primary = m_selection.control_point();
        const ControlPoint* point = primary == nullptr ? nullptr : m_scene.resolve(*primary);
        if (point != nullptr) {
            return point->position;
        }
    }
    long double x = 0.0L;
    long double y = 0.0L;
    long double z = 0.0L;
    for (const ControlPointSelection selection : selected) {
        const ControlPoint* point = m_scene.resolve(selection);
        if (point == nullptr) {
            return std::nullopt;
        }
        x += point->position.x;
        y += point->position.y;
        z += point->position.z;
    }
    const long double count = static_cast<long double>(selected.size());
    const cad::Point3 result{
        static_cast<double>(x / count),
        static_cast<double>(y / count),
        static_cast<double>(z / count)
    };
    return cad::is_finite(result) ? std::optional{result} : std::nullopt;
}

std::optional<TransformFrame> TransformController::frame() const {
    const auto frame_pivot = pivot();
    if (!frame_pivot) {
        return std::nullopt;
    }
    TransformFrame result{.pivot = *frame_pivot};
    if (m_orientation == TransformOrientation::world) {
        return result;
    }
    std::optional<std::array<cad::Vector3, 3>> axes;
    if (const ControlPointSelection* primary = m_selection.control_point()) {
        const SceneNode* node = m_scene.find_entity(primary->entity);
        if (node != nullptr && node->surface != nullptr) {
            const auto net = node->surface->control_net_2d();
            const auto tangent = [&net](std::size_t u, std::size_t v, bool along_u) {
                const std::size_t index = along_u ? u : v;
                const std::size_t count = along_u ? net.extent(0) : net.extent(1);
                const auto point = [&net, along_u](std::size_t varying, std::size_t fixed) {
                    return along_u ? net[varying, fixed].position : net[fixed, varying].position;
                };
                const std::size_t fixed = along_u ? v : u;
                if (index > 0 && index + 1 < count) {
                    return point(index + 1, fixed) - point(index - 1, fixed);
                }
                if (index + 1 < count) {
                    return point(index + 1, fixed) - point(index, fixed);
                }
                return index > 0 ? point(index, fixed) - point(index - 1, fixed) :
                    cad::Vector3{};
            };
            axes = make_frame(
                tangent(primary->u, primary->v, true),
                tangent(primary->u, primary->v, false)
            );
        }
    } else if (const EntitySelection* entity = m_selection.entity()) {
        const SceneNode* node = m_scene.find_entity(entity->entity);
        if (node != nullptr && node->surface != nullptr) {
            const auto u_domain = node->surface->u_domain();
            const auto v_domain = node->surface->v_domain();
            if (u_domain && v_domain) {
                const auto derivatives = node->surface->evaluate_derivatives(
                    std::midpoint(u_domain->first, u_domain->second),
                    std::midpoint(v_domain->first, v_domain->second)
                );
                if (derivatives) {
                    axes = make_frame(derivatives->u, derivatives->v);
                }
            }
        }
    }
    if (axes) {
        result.x = (*axes)[0];
        result.y = (*axes)[1];
        result.z = (*axes)[2];
    }
    return result;
}

bool TransformController::begin_translation(TranslationConstraint constraint) {
    (void)cancel();
    auto selections = targets();
    auto points = capture(selections);
    if (selections.empty() || points.size() != selections.size()) {
        return false;
    }
    m_translation = TranslationState{std::move(selections), std::move(points), constraint, {}};
    return true;
}

bool TransformController::preview_translation(cad::Vector3 delta) {
    if (!m_translation || !cad::is_finite(delta)) {
        return false;
    }
    auto points = m_translation->initial_points;
    for (ControlPoint& point : points) {
        point.position = point.position + delta;
    }
    if (!apply_points(m_scene, m_translation->selections, points)) {
        return false;
    }
    m_translation->delta = delta;
    return true;
}

bool TransformController::finish_translation() {
    if (!m_translation) {
        return false;
    }
    TranslationState state = std::move(*m_translation);
    m_translation.reset();
    if (state.delta == cad::Vector3{}) {
        return false;
    }
    auto final = capture(state.selections);
    if (final.size() != state.selections.size()) {
        (void)apply_points(m_scene, state.selections, state.initial_points);
        return false;
    }
    m_history.record_applied(std::make_unique<TransformCommand>(
        "Translate Selection", m_scene, std::move(state.selections),
        std::move(state.initial_points), std::move(final)
    ));
    return true;
}

bool TransformController::cancel_translation() {
    if (!m_translation) {
        return false;
    }
    TranslationState state = std::move(*m_translation);
    m_translation.reset();
    return apply_points(m_scene, state.selections, state.initial_points);
}

bool TransformController::translate(cad::Vector3 delta) {
    if (!begin_translation(TranslationConstraint::screen)) {
        return false;
    }
    if (!preview_translation(delta)) {
        (void)cancel_translation();
        return false;
    }
    return finish_translation();
}

bool TransformController::begin_rotation(RotationConstraint constraint, cad::Vector3 axis) {
    (void)cancel();
    const auto unit_axis = cad::normalized(axis);
    const auto rotation_pivot = pivot();
    auto selections = targets();
    auto points = capture(selections);
    if (!unit_axis || !rotation_pivot || selections.empty() || points.size() != selections.size()) {
        return false;
    }
    m_rotation = RotationState{
        std::move(selections), std::move(points), constraint, *unit_axis, *rotation_pivot, 0.0
    };
    return true;
}

bool TransformController::preview_rotation(double angle) {
    if (!m_rotation || !std::isfinite(angle)) {
        return false;
    }
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    auto points = m_rotation->initial_points;
    for (ControlPoint& point : points) {
        const cad::Vector3 offset = point.position - m_rotation->pivot;
        point.position = m_rotation->pivot + offset * cosine +
            cad::cross(m_rotation->axis, offset) * sine +
            m_rotation->axis * (cad::dot(m_rotation->axis, offset) * (1.0 - cosine));
    }
    if (!apply_points(m_scene, m_rotation->selections, points)) {
        return false;
    }
    m_rotation->angle_radians = angle;
    return true;
}

bool TransformController::finish_rotation() {
    if (!m_rotation) {
        return false;
    }
    RotationState state = std::move(*m_rotation);
    m_rotation.reset();
    if (state.angle_radians == 0.0) {
        return false;
    }
    auto final = capture(state.selections);
    if (final.size() != state.selections.size()) {
        (void)apply_points(m_scene, state.selections, state.initial_points);
        return false;
    }
    m_history.record_applied(std::make_unique<TransformCommand>(
        "Rotate Selection", m_scene, std::move(state.selections),
        std::move(state.initial_points), std::move(final)
    ));
    return true;
}

bool TransformController::cancel_rotation() {
    if (!m_rotation) {
        return false;
    }
    RotationState state = std::move(*m_rotation);
    m_rotation.reset();
    return apply_points(m_scene, state.selections, state.initial_points);
}

bool TransformController::rotate(cad::Vector3 axis, double angle) {
    if (!begin_rotation(RotationConstraint::screen, axis)) {
        return false;
    }
    if (!preview_rotation(angle)) {
        (void)cancel_rotation();
        return false;
    }
    return finish_rotation();
}

bool TransformController::begin_scale(ScaleConstraint constraint) {
    (void)cancel();
    const auto transform_frame = frame();
    auto selections = targets();
    auto points = capture(selections);
    if (!transform_frame || selections.empty() || points.size() != selections.size()) {
        return false;
    }
    const cad::Vector3 axis = constraint == ScaleConstraint::x ? transform_frame->x :
        (constraint == ScaleConstraint::y ? transform_frame->y : transform_frame->z);
    m_scale = ScaleState{
        std::move(selections), std::move(points), constraint, axis, transform_frame->pivot, 1.0
    };
    return true;
}

bool TransformController::preview_scale(double factor) {
    if (!m_scale || !std::isfinite(factor) || factor <= 0.0) {
        return false;
    }
    auto points = m_scale->initial_points;
    for (ControlPoint& point : points) {
        cad::Vector3 offset = point.position - m_scale->pivot;
        offset = m_scale->constraint == ScaleConstraint::uniform
            ? offset * factor
            : offset + m_scale->axis * (cad::dot(offset, m_scale->axis) * (factor - 1.0));
        point.position = m_scale->pivot + offset;
    }
    if (!apply_points(m_scene, m_scale->selections, points)) {
        return false;
    }
    m_scale->factor = factor;
    return true;
}

bool TransformController::finish_scale() {
    if (!m_scale) {
        return false;
    }
    ScaleState state = std::move(*m_scale);
    m_scale.reset();
    if (state.factor == 1.0) {
        return false;
    }
    auto final = capture(state.selections);
    if (final.size() != state.selections.size()) {
        (void)apply_points(m_scene, state.selections, state.initial_points);
        return false;
    }
    m_history.record_applied(std::make_unique<TransformCommand>(
        "Scale Selection", m_scene, std::move(state.selections),
        std::move(state.initial_points), std::move(final)
    ));
    return true;
}

bool TransformController::cancel_scale() {
    if (!m_scale) {
        return false;
    }
    ScaleState state = std::move(*m_scale);
    m_scale.reset();
    return apply_points(m_scene, state.selections, state.initial_points);
}

bool TransformController::scale(ScaleConstraint constraint, double factor) {
    if (!begin_scale(constraint)) {
        return false;
    }
    if (!preview_scale(factor)) {
        (void)cancel_scale();
        return false;
    }
    return finish_scale();
}

bool TransformController::has_preview() const {
    return (m_translation && m_translation->delta != cad::Vector3{}) ||
        (m_rotation && m_rotation->angle_radians != 0.0) ||
        (m_scale && m_scale->factor != 1.0);
}

std::string_view TransformController::active_description() const {
    if (m_translation && m_translation->delta != cad::Vector3{}) return "Translate Selection";
    if (m_rotation && m_rotation->angle_radians != 0.0) return "Rotate Selection";
    if (m_scale && m_scale->factor != 1.0) return "Scale Selection";
    return {};
}

bool TransformController::cancel() {
    if (m_translation) return cancel_translation();
    if (m_rotation) return cancel_rotation();
    if (m_scale) return cancel_scale();
    return false;
}

bool TransformController::commit() {
    if (m_translation) return finish_translation();
    if (m_rotation) return finish_rotation();
    if (m_scale) return finish_scale();
    return false;
}

void TransformController::reset() {
    m_translation.reset();
    m_rotation.reset();
    m_scale.reset();
}

std::optional<TranslationConstraint> TransformController::translation_constraint() const {
    return m_translation ? std::optional{m_translation->constraint} : std::nullopt;
}

std::optional<RotationConstraint> TransformController::rotation_constraint() const {
    return m_rotation ? std::optional{m_rotation->constraint} : std::nullopt;
}

std::optional<ScaleConstraint> TransformController::scale_constraint() const {
    return m_scale ? std::optional{m_scale->constraint} : std::nullopt;
}
