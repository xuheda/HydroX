# Third-party dependencies

HydroX uses the following third-party libraries:

| Library | Purpose | Management |
|---|---|---|
| [Eigen](https://eigen.tuxfamily.org/) | Header-only linear algebra | **Manual clone** into `third_party/eigen` |
| [Micro-CDR](https://github.com/eProsima/Micro-CDR) | CDR serialization for XRCE-DDS | CMake `FetchContent` |
| [Micro-XRCE-DDS-Client](https://github.com/eProsima/Micro-XRCE-DDS-Client) | DDS/XRCE client transport | CMake `FetchContent` |
| [Apache NuttX](https://nuttx.apache.org/) | Pixhawk 6C RTOS and MCU platform | Release and SHA-512 pinned in `nuttx.lock.json` |

## Eigen

Eigen must be cloned manually before building HydroX:

```bash
git clone --depth 1 --branch 3.4.0 \
    https://gitlab.com/libeigen/eigen.git \
    third_party/eigen
```

For users in China:

```bash
git clone --depth 1 --branch 3.4.0 \
    https://gitee.com/mirrors/eigen.git \
    third_party/eigen
```

The `third_party/eigen/` directory is ignored by Git so it is not committed to the HydroX repository.

## Micro XRCE-DDS

Micro-CDR and Micro-XRCE-DDS-Client are downloaded automatically by CMake on the first configure. If your build environment cannot reach GitHub, you may need to configure a proxy or clone the repositories manually and adjust the `FetchContent_Declare` calls in `CMakeLists.txt`.

## Apache NuttX

HydroX Stage 2 is pinned to the official Apache NuttX 13.0.0 OS and Apps
release. The source URLs and Apache-published SHA-512 values are recorded in
`nuttx.lock.json`. Fetch and verify both packages with:

```powershell
.\tools\fetch_nuttx.ps1
```

or:

```bash
./tools/fetch_nuttx.sh
```

The extracted `third_party/nuttx/` and `third_party/nuttx-apps/` directories
are local build dependencies and are ignored by Git. HydroX board code remains
in `boards/fmu_v6c`; it is not patched directly into the downloaded release.

## Arm GNU Toolchain

The Windows FMUv6C build is pinned to the official Arm GNU Toolchain
13.3.Rel1 archive and SHA-256 recorded in `arm_gnu_toolchain.lock.json`.
Install or verify the ignored local toolchain with:

```powershell
.\tools\fetch_arm_gnu_toolchain.ps1
```

The build also creates an ignored Python virtual environment under `build/`
and installs the exact packages listed in `requirements-fmuv6c-build.txt`.

## Build verification

```bash
cd HydroX
mkdir build_sitl && cd build_sitl
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
./hydrox_sitl --ue5-port 14600
```

## Cross-compilation (Pixhawk 6C HITL)

```powershell
.\build_pixhawk6c_hitl.bat -Clean
.\build_pixhawk6c_hitl.bat -Offline
```

The PowerShell entry fetches and verifies missing pinned dependencies unless
`-Offline` is specified. The compatibility batch file delegates to
`build_fmuv6c_hitl.ps1`. On Linux, use `./build_pixhawk6c_hitl.sh` with the
matching NuttX build tools and Arm compiler already available in `PATH`.

The build generates and validates these release artifacts under
`build/fmu_v6c_hitl/firmware/`:

- `hydrox_pixhawk6cmini_hitl.elf`
- `hydrox_pixhawk6cmini_hitl.bin`
- `hydrox_pixhawk6cmini_hitl.hex`
- `hydrox_pixhawk6cmini_hitl.map`
- `hydrox_pixhawk6cmini_hitl.px4`
- `hydrox_pixhawk6cmini_hitl.manifest.json`

Only artifacts whose manifest reports `FLASH_CANDIDATE_HITL_ONLY`, a clean
tracked tree, and `physical_actuator_drivers_enabled: false` are eligible for
the disconnected-actuator board qualification procedure. A successful build
does not by itself approve connection of motors, servos, PWM, or CAN loads.
