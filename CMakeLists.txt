cmake_minimum_required(VERSION 3.23)
project(nurbmodelling)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CTest)

include(FetchContent)

FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/6.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(raylib)

FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

find_package(Qt6 6.5 REQUIRED COMPONENTS
    OpenGL
    OpenGLWidgets
    Widgets
)

add_library(cad_geometry STATIC
    src/bspline_basis.cpp
    src/geometry_queries.cpp
    src/nurbs_surface.cpp
    src/surface_sampling.cpp
    src/surface_tessellation.cpp
)
target_include_directories(cad_geometry PUBLIC include)

add_library(cad_document STATIC
    src/command_history.cpp
    src/document_io.cpp
    src/scene.cpp
)
target_include_directories(cad_document PUBLIC include)
target_link_libraries(cad_document PUBLIC cad_geometry PRIVATE nlohmann_json::nlohmann_json)

add_library(cad_viewport STATIC
    src/gizmo_controller.cpp
    src/transform_gizmos.cpp
    src/viewport_display_settings.cpp
    src/viewport_math.cpp
)
target_include_directories(cad_viewport PUBLIC include)
target_link_libraries(cad_viewport PUBLIC cad_geometry)

add_library(cad_editor STATIC
    src/editor_session.cpp
    src/input_tools.cpp
    src/surface_picking.cpp
    src/transform_controller.cpp
)
target_include_directories(cad_editor PUBLIC include)
target_link_libraries(cad_editor PUBLIC cad_document cad_viewport)

add_library(cad_raylib_renderer STATIC
    src/raylib_viewport_renderer.cpp
)
target_include_directories(cad_raylib_renderer PUBLIC include)
target_link_libraries(cad_raylib_renderer PUBLIC cad_document cad_viewport PRIVATE raylib)

function(enable_project_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

enable_project_warnings(cad_geometry)
enable_project_warnings(cad_document)
enable_project_warnings(cad_viewport)
enable_project_warnings(cad_editor)
enable_project_warnings(cad_raylib_renderer)

add_library(cad_qt_ui STATIC
    src/main_window.cpp
    src/qt_create_surface_dialog.cpp
    src/qt_control_point_inspector.cpp
    src/qt_scene_outliner.cpp
    src/raylib_viewport_widget.cpp
)
target_include_directories(cad_qt_ui PUBLIC include)
target_link_libraries(cad_qt_ui PUBLIC
    cad_editor
    Qt6::OpenGLWidgets
    Qt6::Widgets
    PRIVATE
    cad_raylib_renderer
    raylib
)
enable_project_warnings(cad_qt_ui)

if(BUILD_TESTING)
    add_executable(kernel_math_test
        tests/kernel_math_test.cpp
    )
    target_link_libraries(kernel_math_test PRIVATE cad_geometry)
    enable_project_warnings(kernel_math_test)
    add_test(NAME kernel_math_test COMMAND kernel_math_test)

    add_executable(geometry_tolerance_test
        tests/geometry_tolerance_test.cpp
    )
    target_link_libraries(geometry_tolerance_test PRIVATE cad_geometry)
    enable_project_warnings(geometry_tolerance_test)
    add_test(NAME geometry_tolerance_test COMMAND geometry_tolerance_test)

    add_executable(bspline_basis_test
        tests/bspline_basis_test.cpp
    )
    target_link_libraries(bspline_basis_test PRIVATE cad_geometry)
    enable_project_warnings(bspline_basis_test)
    add_test(NAME bspline_basis_test COMMAND bspline_basis_test)

    add_executable(geometry_queries_test
        tests/geometry_queries_test.cpp
    )
    target_link_libraries(geometry_queries_test PRIVATE cad_geometry)
    enable_project_warnings(geometry_queries_test)
    add_test(NAME geometry_queries_test COMMAND geometry_queries_test)

    add_executable(orbit_camera_test
        tests/orbit_camera_test.cpp
    )
    target_link_libraries(orbit_camera_test PRIVATE cad_geometry)
    enable_project_warnings(orbit_camera_test)
    add_test(NAME orbit_camera_test COMMAND orbit_camera_test)

    add_executable(nurbs_surface_test
        tests/nurbs_surface_test.cpp
    )
    target_link_libraries(nurbs_surface_test PRIVATE cad_geometry)
    enable_project_warnings(nurbs_surface_test)
    add_test(NAME nurbs_surface_test COMMAND nurbs_surface_test)

    add_executable(document_io_test
        tests/document_io_test.cpp
    )
    target_link_libraries(document_io_test PRIVATE cad_document)
    enable_project_warnings(document_io_test)
    add_test(NAME document_io_test COMMAND document_io_test)

    add_executable(command_history_test
        tests/command_history_test.cpp
    )
    target_link_libraries(command_history_test PRIVATE cad_document)
    enable_project_warnings(command_history_test)
    add_test(NAME command_history_test COMMAND command_history_test)

    add_executable(editor_session_test
        tests/editor_session_test.cpp
    )
    target_link_libraries(editor_session_test PRIVATE cad_editor)
    enable_project_warnings(editor_session_test)
    add_test(NAME editor_session_test COMMAND editor_session_test)

    add_executable(surface_picking_test
        tests/surface_picking_test.cpp
    )
    target_link_libraries(surface_picking_test PRIVATE cad_editor)
    enable_project_warnings(surface_picking_test)
    add_test(NAME surface_picking_test COMMAND surface_picking_test)

    add_executable(surface_tessellation_test
        tests/surface_tessellation_test.cpp
    )
    target_link_libraries(surface_tessellation_test PRIVATE cad_geometry)
    enable_project_warnings(surface_tessellation_test)
    add_test(NAME surface_tessellation_test COMMAND surface_tessellation_test)

    add_executable(viewport_display_settings_test
        tests/viewport_display_settings_test.cpp
    )
    target_link_libraries(viewport_display_settings_test PRIVATE cad_viewport)
    enable_project_warnings(viewport_display_settings_test)
    add_test(NAME viewport_display_settings_test COMMAND viewport_display_settings_test)
endif()

add_executable(cad_app main.cpp)
target_link_libraries(cad_app PRIVATE
    cad_qt_ui
)

set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY VS_STARTUP_PROJECT cad_app)

include(GNUInstallDirs)
install(TARGETS cad_app
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT Nurbsman
)
