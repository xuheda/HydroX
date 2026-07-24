# HydroX

HydroX is a software-in-the-loop (SITL) flight-control runtime for marine and cross-domain autonomy. It provides navigation, guidance, control, allocation, and MAVLink HIL interfaces for vehicles running in simulation or on embedded platforms.

This repository contains the HydroX autopilot core only. It is designed to be built standalone and can be integrated with simulators such as OceanX via MAVLink HIL.

## Features

- Fossen-style 6-DOF marine dynamics and vehicle parameter models
- EKF-based state estimation
- Multi-domain controllers: AUV, ROV, surface vessel, multirotor, fixed-wing, VTOL
- MAVLink HIL sensor and actuator interfaces
- Micro XRCE-DDS client for ROS 2 / mission-system integration
- Extensive unit-test coverage

## Repository Layout

| Path | Role |
|---|---|
| `include/` | Public headers |
| `src/` | Core implementation |
| `src/sitl/` | SITL-specific runtime |
| `src/transport/` | MAVLink / serial / TCP transports |
| `tests/` | Unit tests |
| `cmake/` | CMake toolchain files |
| `third_party/` | External dependencies (Eigen) |

## Build

### Windows

```bat
build_sitl.bat
```

To also deploy the built executable to an external runtime directory:

```bat
build_sitl.bat hydrox_sitl C:\Path\To\Runtime\Dir
```

### Linux

```bash
bash build_sitl.sh
```

### Dependencies

- CMake 3.16+
- C++17 compiler (Visual Studio 2022 on Windows, GCC/Clang on Linux)
- Eigen3 (clone to `third_party/eigen`):
  ```bash
  git clone --depth 1 https://gitlab.com/libeigen/eigen.git third_party/eigen
  ```

Micro XRCE-DDS Client is downloaded automatically by CMake via FetchContent.

## Run Tests

After building:

```bat
cd build_sitl
ctest -C Release --output-on-failure
```

## Integration with OceanX

HydroX is developed as part of the OceanX simulator ecosystem. When built from within the OceanX workspace, the `build_sitl.bat` script can deploy `hydrox_sitl.exe` next to the packaged Unreal runtime. Standalone builds produce the executable under `build_sitl/Release/`.

## License

See [LICENSE](LICENSE).
