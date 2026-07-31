cmake_minimum_required(VERSION 3.23)
project(nurbmodelling)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/6.0.tar.gz
)
FetchContent_MakeAvailable(raylib)

add_library(cad_kernel STATIC
    src/nurbs_surface.cpp
    src/gpu_buffer.cpp
    src/input_tools.cpp
    src/scene.cpp
    src/ui/ui_widgets.cpp
    src/ui/ui_panel.cpp
    src/ui/ui_layer.cpp
    src/ui/control_point_inspector.cpp
)

target_include_directories(cad_kernel PUBLIC include)
target_link_libraries(cad_kernel PUBLIC raylib)

if(MSVC)
    target_compile_options(cad_kernel PRIVATE /W4)
else()
    target_compile_options(cad_kernel PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(cad_app main.cpp)
target_link_libraries(cad_app PRIVATE cad_kernel raylib)

set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY VS_STARTUP_PROJECT cad_app)