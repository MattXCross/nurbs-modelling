# Nurbsman

<img width="1279" height="718" alt="Nurbsman editor" src="https://github.com/user-attachments/assets/5dd0dde1-fe66-407c-b666-1b45d04ab25c" />

Nurbsman is an experimental C++23 NURBS modelling application.

## Requirements

- A C++23 compiler
- CMake 3.23 or newer
- Ninja
- Qt 6.5 or newer with Widgets and OpenGL support
- OpenGL 3.3 support

On Arch Linux, install the development dependencies with:

```bash
sudo pacman -S cmake ninja qt6-base
```

On Ubuntu or Debian, install them with:

```bash
sudo apt install cmake ninja-build qt6-base-dev libqt6opengl6-dev
```

Raylib 6.0 is downloaded by CMake during configuration.

## Build

Configure and build a debug version with the committed presets:

```bash
cmake --preset debug
cmake --build --preset debug
```

The executable is written to `build/debug/cad_app`. Use the `release` preset for an optimized build.

If Qt is installed outside the platform's standard search paths, create an ignored `CMakeUserPresets.json` that inherits a committed preset and sets `CMAKE_PREFIX_PATH` to the local Qt installation. Do not add machine-specific Qt paths to `CMakePresets.json`.