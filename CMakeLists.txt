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

find_package(Qt6 6.5 REQUIRED COMPONENTS
    OpenGL
    OpenGLWidgets
    Widgets
)

add_library(cad_geometry STATIC
    src/nurbs_surface.cpp
)
target_include_directories(cad_geometry PUBLIC include)

add_library(cad_document STATIC
    src/command_history.cpp
    src/scene.cpp
)
target_include_directories(cad_document PUBLIC include)
target_link_libraries(cad_document PUBLIC cad_geometry)

add_library(cad_editor STATIC
    src/editor_session.cpp
    src/input_tools.cpp
)
target_include_directories(cad_editor PUBLIC include)
target_link_libraries(cad_editor PUBLIC cad_document)

add_library(cad_raylib_renderer STATIC
    src/raylib_viewport_renderer.cpp
)
target_include_directories(cad_raylib_renderer PUBLIC include)
target_link_libraries(cad_raylib_renderer PUBLIC cad_document PRIVATE raylib)

function(enable_project_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

enable_project_warnings(cad_geometry)
enable_project_warnings(cad_document)
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
    add_executable(nurbs_surface_test
        tests/nurbs_surface_test.cpp
    )
    target_link_libraries(nurbs_surface_test PRIVATE cad_geometry)
    enable_project_warnings(nurbs_surface_test)
    add_test(NAME nurbs_surface_test COMMAND nurbs_surface_test)
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
