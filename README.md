# Nurbsman

<img width="1654" height="962" alt="image" src="https://github.com/user-attachments/assets/ef280734-bce6-4fa7-9eaa-58908a2acdd1" />

Nurbsman is an experimental C++23 NURBS modelling application.

## Requirements

- A compiler and standard library with support for `<mdspan>`, `<print>`, and explicit object parameters
- CMake 3.23 or newer
- Ninja
- Qt 6.5 or newer with Widgets and OpenGL support
- OpenGL 3.3 support

On Arch Linux, install the development dependencies with:

```bash
sudo pacman -S cmake ninja qt6-base
```

On other Linux distributions, install CMake, Ninja, a recent C++23 toolchain, and the Qt 6.5+ base development package. Ubuntu 24.04 ships Qt 6.4, so use a newer Qt installation from the official Qt installer or `aqtinstall` rather than its system Qt package.

Raylib 6.0 is downloaded by CMake during configuration.

## Build

Configure and build a debug version with the committed presets:

```bash
cmake --preset debug
cmake --build --preset debug
```

The executable is written to `build/debug/cad_app`. Use the `release` preset for an optimized build.

## Install

Install the release executable with:

```bash
cmake --preset release
cmake --build --preset release
cmake --install build/release --prefix dist/nurbsman --component Nurbsman
```

The installed executable is written to `dist/nurbsman/bin/cad_app`.
This install step does not bundle dependencies. The executable requires compatible Qt and OpenGL runtime libraries discoverable by the system dynamic linker. When using Qt from the official installer or `aqtinstall`, configure the runtime library path for that Qt installation before launching the installed executable.

If Qt is installed outside the platform's standard search paths, create an ignored `CMakeUserPresets.json` that inherits a committed preset and sets `CMAKE_PREFIX_PATH` to the local Qt installation. Do not add machine-specific Qt paths to `CMakePresets.json`.
