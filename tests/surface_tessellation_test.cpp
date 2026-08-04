#include "nurbs_surface.h"
#include "surface_tessellation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <string_view>
#include <vector>

namespace {

int failures = 0;

void expect(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

std::unique_ptr<NurbsSurface> make_plane(
    std::vector<double> u_knots = {2.0, 2.0, 5.0, 5.0},
    std::vector<double> v_knots = {-3.0, -3.0, 7.0, 7.0}
) {
    auto surface = NurbsSurface::create(
        2, 2, 1, 1,
        std::vector<ControlPoint>{
            {{-2.0, -1.0, 4.0}, 1.0}, {{-2.0, 3.0, 4.0}, 1.0},
            {{6.0, -1.0, 4.0}, 1.0}, {{6.0, 3.0, 4.0}, 1.0}
        },
        std::move(u_knots), std::move(v_knots)
    );
    expect(surface.has_value(), "construct planar patch");
    return surface ? std::move(*surface) : nullptr;
}

std::unique_ptr<NurbsSurface> make_curved(bool rational = false) {
    std::vector<ControlPoint> points;
    for (std::size_t u = 0; u < 3; ++u) {
        for (std::size_t v = 0; v < 3; ++v) {
            const double x = static_cast<double>(u) - 1.0;
            const double y = static_cast<double>(v) - 1.0;
            points.push_back({{x, y, (u == 1 && v == 1) ? 2.0 : 0.0},
                              rational && u == 1 && v == 1 ? 3.0 : 1.0});
        }
    }
    auto surface = NurbsSurface::create(3, 3, 2, 2, std::move(points));
    expect(surface.has_value(), rational ? "construct rational patch" : "construct curved patch");
    return surface ? std::move(*surface) : nullptr;
}

bool finite_mesh(const cad::SurfaceMesh& mesh) {
    return mesh.positions.size() == mesh.normals.size() &&
        mesh.positions.size() == mesh.uvs.size() &&
        std::ranges::all_of(mesh.positions, [](cad::Point3 point) {
            return cad::is_finite(point);
        }) &&
        std::ranges::all_of(mesh.normals, [](cad::Vector3 normal) {
            return cad::is_finite(normal) && std::abs(cad::length(normal) - 1.0) < 1e-12;
        }) &&
        std::ranges::all_of(mesh.uvs, [](cad::Point2 uv) {
            return cad::is_finite(uv) && uv.x >= 0.0 && uv.x <= 1.0 &&
                uv.y >= 0.0 && uv.y <= 1.0;
        }) &&
        std::ranges::all_of(mesh.triangle_indices, [&mesh](std::uint32_t index) {
            return index < mesh.positions.size();
        });
}

double mesh_error(const NurbsSurface& surface, const cad::SurfaceMesh& mesh) {
    double result = 0.0;
    for (int u = 0; u <= 40; ++u) {
        for (int v = 0; v <= 40; ++v) {
            const double normalized_u = static_cast<double>(u) / 40.0;
            const double normalized_v = static_cast<double>(v) / 40.0;
            const auto u_domain = *surface.u_domain();
            const auto v_domain = *surface.v_domain();
            const auto exact = surface.evaluate(
                u_domain.first + (u_domain.second - u_domain.first) * normalized_u,
                v_domain.first + (v_domain.second - v_domain.first) * normalized_v
            );
            if (!exact) return std::numeric_limits<double>::infinity();

            double closest = std::numeric_limits<double>::infinity();
            for (std::size_t index = 0; index + 2 < mesh.triangle_indices.size(); index += 3) {
                const std::uint32_t ia = mesh.triangle_indices[index];
                const std::uint32_t ib = mesh.triangle_indices[index + 1];
                const std::uint32_t ic = mesh.triangle_indices[index + 2];
                const cad::Point2 a = mesh.uvs[ia];
                const cad::Point2 b = mesh.uvs[ib];
                const cad::Point2 c = mesh.uvs[ic];
                const double denominator = cad::cross(b - a, c - a);
                if (denominator == 0.0) continue;
                const cad::Point2 sample{normalized_u, normalized_v};
                const double wb = cad::cross(sample - a, c - a) / denominator;
                const double wc = cad::cross(b - a, sample - a) / denominator;
                const double wa = 1.0 - wb - wc;
                if (wa < -1e-12 || wb < -1e-12 || wc < -1e-12) continue;
                const cad::Point3 approximation{
                    mesh.positions[ia].x * wa + mesh.positions[ib].x * wb + mesh.positions[ic].x * wc,
                    mesh.positions[ia].y * wa + mesh.positions[ib].y * wb + mesh.positions[ic].y * wc,
                    mesh.positions[ia].z * wa + mesh.positions[ib].z * wb + mesh.positions[ic].z * wc
                };
                closest = std::min(closest, cad::distance(*exact, approximation));
            }
            result = std::max(result, closest);
        }
    }
    return result;
}

void test_exact_planar_mesh_and_domains() {
    const auto surface = make_plane();
    const auto mesh = cad::tessellate_surface(*surface);
    expect(mesh.has_value(), "tessellate non-default-domain plane");
    if (!mesh) return;
    expect(mesh->positions.size() == 4, "plane has one conforming quad");
    expect(mesh->triangle_indices == std::vector<std::uint32_t>{0, 2, 3, 0, 3, 1},
           "plane has deterministic indexed triangles");
    expect(mesh->uvs == std::vector<cad::Point2>{{0, 0}, {0, 1}, {1, 0}, {1, 1}},
           "UVs normalize the actual parameter domain");
    expect(std::ranges::all_of(mesh->normals, [](cad::Vector3 normal) {
        return normal == cad::Vector3{0.0, 0.0, 1.0};
    }), "plane derivative normals are exact and oriented");
    expect(finite_mesh(*mesh), "planar mesh data is finite and indexed safely");
}

void test_knot_spans_are_grid_boundaries() {
    std::vector<ControlPoint> points;
    for (std::size_t u = 0; u < 4; ++u) {
        for (std::size_t v = 0; v < 2; ++v) {
            points.push_back({{static_cast<double>(u), static_cast<double>(v), 0.0}, 1.0});
        }
    }
    auto surface = NurbsSurface::create(
        4, 2, 1, 1, std::move(points),
        {2.0, 2.0, 3.0, 4.0, 5.0, 5.0}, {-1.0, -1.0, 1.0, 1.0}
    );
    expect(surface.has_value(), "construct multi-span plane");
    if (!surface) return;
    const auto mesh = cad::tessellate_surface(**surface);
    expect(mesh.has_value() && mesh->positions.size() == 8,
           "every nonempty knot span contributes a grid boundary");
    if (mesh) {
        const auto has_uv = [&mesh](double u) {
            return std::ranges::any_of(mesh->uvs, [u](cad::Point2 uv) {
                return std::abs(uv.x - u) < 1e-15;
            });
        };
        expect(has_uv(1.0 / 3.0) && has_uv(2.0 / 3.0),
               "internal knots are preserved in normalized UVs");
    }
}

void test_curved_rational_and_monotonic_accuracy() {
    for (bool rational : {false, true}) {
        const auto surface = make_curved(rational);
        cad::SurfaceTessellationSettings loose{
            .chordal_tolerance = 0.2,
            .normal_angle_tolerance_radians = 0.8,
            .max_refinement_depth = 8
        };
        cad::SurfaceTessellationSettings tight = loose;
        tight.chordal_tolerance = 0.02;
        tight.normal_angle_tolerance_radians = 0.2;
        const auto loose_mesh = cad::tessellate_surface(*surface, loose);
        const auto tight_mesh = cad::tessellate_surface(*surface, tight);
        expect(loose_mesh.has_value() && tight_mesh.has_value(),
               rational ? "tessellate rational patch" : "tessellate curved patch");
        if (!loose_mesh || !tight_mesh) continue;
        expect(loose_mesh->positions.size() > 4,
               rational ? "rational interior samples trigger refinement"
                        : "curved interior samples trigger refinement");
        expect(tight_mesh->positions.size() >= loose_mesh->positions.size(),
               "tighter settings never reduce mesh detail");
        const double loose_error = mesh_error(*surface, *loose_mesh);
        const double tight_error = mesh_error(*surface, *tight_mesh);
        expect(std::isfinite(loose_error) && tight_error <= loose_error + 1e-12,
               "tighter settings never increase measured mesh error");
        expect(finite_mesh(*tight_mesh), "curved mesh data is finite");
    }
}

void test_degenerate_patch_fallbacks() {
    auto surface = NurbsSurface::create(
        2, 2, 1, 1,
        std::vector<ControlPoint>(4, ControlPoint{{3.0, -2.0, 1.0}, 1.0})
    );
    expect(surface.has_value(), "construct collapsed patch");
    if (!surface) return;
    const auto mesh = cad::tessellate_surface(**surface);
    expect(mesh.has_value(), "tessellate collapsed patch");
    if (!mesh) return;
    expect(mesh->triangle_indices.empty(), "omit geometrically degenerate triangles");
    expect(finite_mesh(*mesh), "collapsed patch has finite deterministic unit normals");
}

void test_errors_resources_and_cache() {
    const auto plane = make_plane();
    cad::SurfaceTessellationSettings invalid;
    invalid.chordal_tolerance = 0.0;
    auto result = cad::tessellate_surface(*plane, invalid);
    expect(!result && result.error().code ==
        cad::SurfaceTessellationErrorCode::invalid_chordal_tolerance,
        "reject invalid chordal tolerance with structured error");
    invalid = {};
    invalid.normal_angle_tolerance_radians = std::numbers::pi + 0.1;
    result = cad::tessellate_surface(*plane, invalid);
    expect(!result && result.error().code ==
        cad::SurfaceTessellationErrorCode::invalid_normal_angle_tolerance,
        "reject invalid normal tolerance");
    invalid = {};
    invalid.max_refinement_depth = 31;
    result = cad::tessellate_surface(*plane, invalid);
    expect(!result && result.error().code ==
        cad::SurfaceTessellationErrorCode::invalid_max_refinement_depth,
        "bound refinement depth");

    std::vector<ControlPoint> multi_span_points;
    for (std::size_t u = 0; u < 4; ++u) {
        for (std::size_t v = 0; v < 2; ++v) {
            multi_span_points.push_back({{
                static_cast<double>(u), static_cast<double>(v), 0.0
            }, 1.0});
        }
    }
    auto multi_span = NurbsSurface::create(
        4, 2, 1, 1, std::move(multi_span_points),
        {0.0, 0.0, 1.0, 2.0, 3.0, 3.0}, {0.0, 0.0, 1.0, 1.0}
    );
    cad::SurfaceTessellationSettings resource_limited;
    resource_limited.max_vertices = 4;
    result = cad::tessellate_surface(**multi_span, resource_limited);
    expect(!result && result.error().code ==
        cad::SurfaceTessellationErrorCode::resource_limit_exceeded,
        "enforce the configured vertex budget before allocation");

    const auto curved = make_curved();
    cad::SurfaceTessellationSettings bounded{
        .chordal_tolerance = 1e-8,
        .normal_angle_tolerance_radians = 1e-8,
        .max_refinement_depth = 0
    };
    result = cad::tessellate_surface(*curved, bounded);
    expect(!result && result.error().code ==
        cad::SurfaceTessellationErrorCode::refinement_limit_reached,
        "report an exhausted refinement safeguard");

    cad::SurfaceTessellationCache cache;
    const auto first = cache.get(*plane, 7);
    const auto same = cache.get(*plane, 7);
    const auto changed_revision = cache.get(*plane, 8);
    cad::SurfaceTessellationSettings changed_settings;
    changed_settings.chordal_tolerance = 0.02;
    const auto changed = cache.get(*plane, 7, changed_settings);
    expect(first && same && first->get() == same->get(), "cache reuses exact key mesh identity");
    expect(changed_revision && changed_revision->get() != first->get(),
           "geometry revision recomputes mesh");
    expect(changed && changed->get() != first->get(), "exact settings changes recompute mesh");
    expect(cache.size() == 3, "cache retains each distinct revision/settings key");
    cache.clear(*plane);
    expect(cache.size() == 0, "clear removes all stale entries for a surface identity");
}

} // namespace

int main() {
    test_exact_planar_mesh_and_domains();
    test_knot_spans_are_grid_boundaries();
    test_curved_rational_and_monotonic_accuracy();
    test_degenerate_patch_fallbacks();
    test_errors_resources_and_cache();

    if (failures != 0) {
        std::cerr << failures << " surface tessellation test(s) failed\n";
        return 1;
    }
    std::cout << "All surface tessellation tests passed\n";
    return 0;
}
