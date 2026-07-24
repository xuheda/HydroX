# 第三方依赖说明

本项目依赖以下纯 header-only 库，需在此目录下手动克隆：

## Eigen（线性代数）

```bash
# 在 HydroX/ 根目录执行
git clone --depth=1 --branch 3.4.0 \
    https://gitlab.com/libeigen/eigen.git \
    third_party/eigen
```

克隆后路径应为：`third_party/eigen/Eigen/Core`

## 构建验证

```bash
cd HydroX
mkdir build_sitl && cd build_sitl
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
./hydrox_sitl --ue5-port 14600
```

## 交叉编译（STM32H7）

```bash
mkdir build_stm32 && cd build_stm32
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain_stm32h7.cmake
cmake --build . -j4
```

需要安装 `arm-none-eabi-gcc`（推荐 ARM GNU 12.3+）。
