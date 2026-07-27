cmake_minimum_required(VERSION 3.23)
project(nurbmodelling)

set(CMAKE_CXX_STANDARD 23)

include(FetchContent)

FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/6.0.tar.gz
)
FetchContent_MakeAvailable(raylib)

add_library(cad_kernel STATIC
    src/nurbs_surface.cpp
    src/gpu_buffer.cpp
    src/scene.cpp
)

target_include_directories(cad_kernel PUBLIC include)

target_compile_options(cad_kernel PRIVATE -Wall -Wextra -Wpedantic)

add_executable(cad_app main.cpp)
target_link_libraries(cad_app PRIVATE cad_kernel raylib)