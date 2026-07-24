# Third-party dependencies

HydroX uses the following third-party libraries:

| Library | Purpose | Management |
|---|---|---|
| [Eigen](https://eigen.tuxfamily.org/) | Header-only linear algebra | **Manual clone** into `third_party/eigen` |
| [Micro-CDR](https://github.com/eProsima/Micro-CDR) | CDR serialization for XRCE-DDS | CMake `FetchContent` |
| [Micro-XRCE-DDS-Client](https://github.com/eProsima/Micro-XRCE-DDS-Client) | DDS/XRCE client transport | CMake `FetchContent` |

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

## Build verification

```bash
cd HydroX
mkdir build_sitl && cd build_sitl
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
./hydrox_sitl --ue5-port 14600
```

## Cross-compilation (STM32H7)

```bash
mkdir build_stm32 && cd build_stm32
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain_stm32h7.cmake
cmake --build . -j4
```

Requires `arm-none-eabi-gcc` (ARM GNU 12.3+ recommended).
