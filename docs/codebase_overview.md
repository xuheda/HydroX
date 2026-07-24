# HydroX 代码介绍文档

> 本文档面向希望阅读、修改或扩展 HydroX 的开发者。 HydroX 是一个用于海洋与跨域自主系统的软件在环（SITL）飞行控制运行时。

---

## 目录

1. [概述](#概述)
2. [顶层目录与构建系统](#顶层目录与构建系统)
3. [公共头文件](#公共头文件)
4. [核心源文件](#核心源文件)
5. [GNC 控制与分配](#gnc-控制与分配)
6. [SITL 运行时](#sitl-运行时)
7. [通信层](#通信层)
8. [XLog 二进制日志](#xlog-二进制日志)
9. [车辆参数系统](#车辆参数系统)
10. [测试覆盖](#测试覆盖)
11. [关键设计模式](#关键设计模式)
12. [数据流总结](#数据流总结)
13. [与 OceanX 的集成](#与-oceanx-的集成)

---

## 概述

HydroX 提供以下能力：

- **状态估计**：18 维混合辅助 EKF，融合 IMU、DVL、GPS、深度、磁罗盘等传感器。
- **导航制导控制（GNC）**：支持 slender-body AUV、ROV、USV、多旋翼、固定翼、VTOL 等多种载具。
- **控制分配**：将期望的 6 自由度广义力映射到具体执行机构（舵、螺旋桨、推进器阵列、旋翼等）。
- **MAVLink HIL 通信**：与仿真器（如 OceanX/Unreal Engine）通过 MAVLink HIL 协议交互。
- **DDS/XRCE 桥接**：通过 Micro XRCE-DDS 与 ROS 2 生态交换遥测和设定点。
- **XLog 日志**：自研二进制时序日志格式，用于回放与诊断。

HydroX 被设计为**可独立运行**的程序（`hydrox_sitl`），也可以作为**库**被其他项目链接。

---

## 顶层目录与构建系统

### 目录结构

```text
HydroX/
├── CMakeLists.txt          # 主构建定义
├── build_sitl.bat          # Windows 构建脚本
├── build_sitl.sh           # Linux 构建脚本
├── LICENSE                 # Apache-2.0
├── README.md
├── THIRD_PARTY_NOTICES.md
├── apps/
│   ├── sitl/main.cpp       # SITL 可执行程序入口
│   └── stm32/main.cpp      # STM32 嵌入式入口（桩）
├── cmake/
│   └── toolchain_stm32h7.cmake
├── include/                # 公共头文件
│   ├── gnc/                # GNC 头文件
│   └── ...
├── src/                    # 核心库实现
│   ├── gnc/                # GNC 实现
│   ├── sitl/               # SITL 运行时
│   ├── transport/          # 传输层
│   └── ...
├── tests/                  # 单元测试
└── third_party/            # 外部依赖说明
```

### CMake 构建目标

| 目标 | 类型 | 说明 |
|---|---|---|
| `hydrox` | 静态库 | 核心飞控库 |
| `hydrox_sitl` | 可执行文件 | PC 端 SITL，TCP 传输 |
| `hydrox_pixhawk6c` | 可执行文件 | STM32 交叉编译目标（需 `-DHYDROX_TARGET=STM32`） |

依赖：

- **Eigen 3.4+**：线性代数，需手动克隆到 `third_party/eigen`。
- **Micro-CDR / Micro-XRCE-DDS-Client**：CMake `FetchContent` 自动下载。
- **Threads / ws2_32 / bcrypt（Windows）或 OpenSSL（Linux）**。

---

## 公共头文件

### `include/types.h`

核心数据结构：

- `AUVState`：6 自由度位姿 `eta` 和体坐标系速度 `nu`，NED 坐标系。
- `FinCmd`：4 个舵角、RPM、推力分数，提供 clamp 和 normalize 方法。
- `IMUMeasurement`、`DVLMeasurement`、`DepthMeasurement`、`GPSMeasurement`：传感器测量。
- `GNCMode`：控制模式（DISABLED、DEPTH_HOLD、WAYPOINT_3D、DP、SURFACE）。
- `GNCSetpoint`：深度、航向、前进速度、航点等设定值。
- `VehicleClass`：载具类别（UUV、USV、多旋翼、固定翼、VTOL）。

### `include/ekf.h`

`class EKF`：18 维混合辅助扩展卡尔曼滤波器。

状态向量：

```text
[N, E, D, roll, pitch, yaw, u, v, w, b_ax, b_ay, b_az, b_gx, b_gy, b_gz, c_N, c_E, c_D]
```

- 位置 / 姿态 / 体坐标系速度
- 加速度计零偏、陀螺零偏
- NED 坐标系下水流速度

测量更新包括：

- 底跟踪 DVL 速度
- 水跟踪 DVL 速度
- 零速度伪测量
- 深度
- GPS 位置 / 速度
- 磁罗盘 / 真值航向
- 加速度计水平对准

### `include/sensor_adapter.h`

`class SensorAdapter`：将 MAVLink 帧解析为 `NavigationInput`。

- `ingest_frame()`：按 `msg_id` 分派。
- `build()`：计算时效性、协方差、GPS 地心地平坐标转 NED、判断加速度计是否可用。

### `include/mavlink_hil.h`

手写 MAVLink 2 编解码器，不依赖 pymavlink/mavlink C 库。

支持的消息：

- 输入：`HIL_SENSOR`、`HIL_GPS`、`HIL_DVL`、`HIL_TRUTH_STATE`、自定义 OceanX 传感器消息。
- 输出：`HIL_ACTUATOR_CONTROLS`、`HEARTBEAT`。
- QGC 遥测：`ATTITUDE`、`LOCAL_POSITION_NED`、`VFR_HUD`、`GLOBAL_POSITION_INT`、`SYS_STATUS`、`STATUSTEXT`。

### `include/transport.h` / `include/tcp_transport.h`

- `Transport`：抽象接口。
- `TcpTransport`：跨平台 TCP socket，支持客户端/服务器模式、非阻塞 I/O。

### `include/xlog_writer.h`

XLog 1.0 二进制日志格式。

- 主题：`HydroxState`、`HydroxSetpoint`、`HydroxControlError`、`HydroxControllerOutput`、`HydroxActuator`、`HydroxEstimatorHealth`、`HydroxTiming`、`SimulatorTruth`、`DeploymentAlignment`。
- `Writer`：分块、CRC32 校验、分段文件写入。

### DDS 相关头文件

| 头文件 | 作用 |
|---|---|
| `dds_publisher.h` | Micro XRCE-DDS 发布/订阅封装 |
| `dds_setpoint_codec.h` | ROS 2 设定点 CDR 解码 |
| `dds_topic_manifest.h` | 话题清单与命名规则 |
| `dds_cdr_writer.h` | 手写 CDR 序列化 |
| `dds_connection_state.h` | 连接状态与会话代次管理 |
| `latest_value_mailbox.h` | 线程安全最新值邮箱 |
| `nonblocking_frame_sender.h` | 非阻塞帧原子发送 |
| `fc_snapshot.h` | 飞控状态快照 |

### `include/fossen_vehicle_params.h`

- `VehicleArchetype`：载具控制构型。
- `FossenControlParams`：从 JSON 加载的车辆参数。
- `load_fossen_control_params()`：加载参数。
- `apply_inertia_normalized_gains()`：按车辆惯量归一化控制器增益。

---

## 核心源文件

### `src/ekf.cpp`

EKF 完整实现：

- `_predict()`：状态传播与协方差数值雅可比传播。
- `_update_*()`：各类传感器的测量更新。
- 创新卡方（NIS）门控，防止异常值污染。
- 水流状态通过水跟踪 DVL 与 GPS 地速联合估计。

### `src/sensor_adapter.cpp`

- 分派 MAVLink 帧。
- GPS 地心地平转局部 NED。
- 区分底跟踪与水跟踪 DVL。
- 根据 `fields_updated` 判断加速度计、磁力计是否有效。

### `src/mavlink_hil.cpp`

- CRC-16/MCRF4XX + CRC_EXTRA，与 PX4 兼容。
- 支持 MAVLink 2 签名验证与签名输出。
- 自定义 OceanX 传感器消息解析。

### `src/mavlink_signing.cpp`

- 从十六进制文件加载 32 字节密钥。
- 使用 Windows BCrypt 或 OpenSSL 计算 SHA-256 摘要。
- 常量时间签名比较。

### `src/fossen_vehicle_params.cpp`

- 手写轻量 JSON 解析器（不依赖外部 JSON 库）。
- 从 `engine/Content/Fossen` 或 `engine/Content/Aero` 加载 JSON。
- 内置 EcaA9、LAUV、DesistekSaga、RexROV2、Otter、WAMV、X500、RCCessna、StandardVTOL 等回退参数。
- 用长球体 Lamb 因子计算附加质量惯量。

### `src/xlog_writer.cpp`

- CRC32、FNV-1a64 schema 哈希、JSON 转义。
- 文件头、元数据、schema、块、页脚的写入。
- 按目标块大小和最大段大小自动分段。

---

## GNC 控制与分配

### 接口分离

```text
IController：状态 + 设定点 -> Wrench（6 自由度广义力）
IAllocator：Wrench -> ActuatorCmd（归一化执行器通道）
```

一个载具 = 一个控制器 + 一个分配器，由 `build_control_stack()` 组装。

### 控制器

| 控制器 | 文件 | 用途 |
|---|---|---|
| `SlenderBodyAUVController` | `gnc/gnc_controller.cpp` | 细长体鱼雷型 AUV，级联深度/俯仰/航向/前进控制 |
| `ThrusterVehicleController` | `gnc/thruster_controller.cpp` | 6 自由度推进器 ROV/AUV |
| `SurfaceVesselController` | `gnc/surface_controller.cpp` | 双桨 USV |
| `MultirotorController` | `gnc/multirotor_controller.cpp` | 四旋翼 |
| `FixedWingController` | `gnc/fixedwing_controller.cpp` | 固定翼 |
| `VtolController` | `gnc/vtol_controller.cpp` | 升力+巡航 VTOL |

### 分配器

| 分配器 | 文件 | 用途 |
|---|---|---|
| `FinAllocator` | `gnc/control_allocator.cpp` | 十字尾舵 + 螺旋桨 |
| `ThrusterMatrixAllocator` | `gnc/thruster_allocator.cpp` | 推进器阵列，阻尼最小二乘伪逆 |
| `SurfaceAllocator` | `gnc/surface_allocator.cpp` | 左右双桨 |
| `MultirotorAllocator` | `gnc/multirotor_allocator.cpp` | 四旋翼推力混合 |
| `FixedWingAllocator` | `gnc/fixedwing_allocator.cpp` | 升降舵/副翼/方向舵/油门 |
| `VtolAllocator` | `gnc/vtol_allocator.cpp` | 升力旋翼 + 舵面 + 推进桨 |

### 低层控制环

- `DepthPID`：深度控制输出期望俯仰角。
- `PitchPID`：俯仰角跟踪。
- `HeadingSMC`：航向滑模控制。
- `SurgeP`：前进速度比例控制。
- `RefModel`：三阶 Fossen 参考模型。

### 电机与能量模型

- `MotorModel`：一阶螺旋桨/电机动态，输出推力、扭矩、功率、电流。
- `EnergyModel`：电池 SOC、端电压、剩余运行时间估计。

---

## SITL 运行时

### `apps/sitl/main.cpp`

SITL 主程序，100 Hz 控制循环：

1. 解析命令行配置。
2. 初始化网络、父进程守护、MAVLink 签名、车辆参数、控制栈。
3. 外层循环：连接 UE5 TCP，断线重连。
4. 内层循环（`rate_hz`）：
   - 读取 MAVLink 字节，送入 `SensorAdapter`。
   - 处理仿真器暂停（`MAV_STATE_STANDBY`）。
   - 1 Hz 发送 `HEARTBEAT`。
   - 消费 DDS 连接状态与设定点，按会话代次过滤。
   - 设定点超时进入安全 DISABLED 状态。
   - EKF 更新。
   - 控制器更新 → 分配器 → 执行器命令。
   - 电机模型、能量模型。
   - 发送 `HIL_ACTUATOR_CONTUATOR_CONTROLS`。
   - 发布 DDS 遥测。
   - 发送 QGC 遥测（4 Hz）和 `SYS_STATUS`（1 Hz）。
   - 写入 XLog。

### `src/sitl/sitl_config.cpp`

命令行配置解析，包括 UE5/QGC/DDS 地址、载具类型、EKF 模式、XLog 路径、初始状态、任务半径/超时等。

### `src/sitl/sitl_platform.cpp`

- `NetworkRuntime`：Windows Winsock 初始化。
- `ParentProcessGuard`：父进程退出时终止 SITL。
- `UdpSender`：QGC 广播 UDP。

### `src/sitl/dds_worker.cpp`

后台线程拥有所有 XRCE-DDS 操作：

- 通过 `LatestValueMailbox` 与 GNC 线程交换遥测、设定点、连接状态。
- 断线重连，指数退避。
- 设定点加盖 XRCE 会话代次戳。

### `src/sitl/sitl_xlog.cpp`

`XLogRecorder`：每控制周期组装一次记录，包括状态、设定点、力矩、执行器、导航、EKF，写入 XLog。

---

## 通信层

### MAVLink HIL

HydroX 与仿真器通过 MAVLink HIL over TCP 通信，完全解耦。

输入：

- `HIL_SENSOR`：IMU、深度。
- `HIL_GPS`：GPS。
- `HIL_DVL`：DVL 体坐标系速度。
- `HIL_TRUTH_STATE`：仿真器真值。
- 自定义 OceanX 传感器消息。

输出：

- `HIL_ACTUATOR_CONTROLS`：归一化执行器输出。
- `HEARTBEAT`：1 Hz。

QGC 遥测：

- `ATTITUDE`、`LOCAL_POSITION_NED`、`VFR_HUD`、`GLOBAL_POSITION_INT` @ 4 Hz。
- `SYS_STATUS` @ 1 Hz。

### Micro XRCE-DDS

```text
GNC 线程 -> LatestValueMailbox -> DDS worker 线程 -> DdsPublisher -> UDP -> MicroXRCEAgent -> ROS 2
ROS 2 设定点 -> MicroXRCEAgent -> UDP -> DdsPublisher -> callback -> LatestValueMailbox -> GNC 线程
```

话题清单见 `include/dds_topic_manifest.h`，包括：

- `vehicle_local_position`、`sensor_combined`、`actuator_outputs`
- `vehicle_status`、`auv_state`、`truth_state`、`odom`、`tf`
- `setpoint`、`passive_sonar_bearing`、`acoustic_neighbors`、`rangefinder_scan`

---

## XLog 二进制日志

### 文件布局

```text
FileHeader (XLOG)
metadata JSON
schema JSON
{ BlockHeader (XBLK) + payload records }
...
FileFooter (XEND)
```

### 记录格式

每条记录：

```text
RecordHeader (topic_id, timestamp_ns, sequence, payload_size) + payload
```

### 可靠性

- CRC32 校验文件头、块头、块载荷。
- 默认 512 MB 分段。
- 所有记录结构体有 `static_assert` 大小校验。
- 不完整尾部块自动忽略。

---

## 车辆参数系统

### 控制构型

```cpp
enum class VehicleArchetype {
    SlenderBodyFin,   // 鱼雷型 AUV（舵+桨）
    Thruster,         // 推进器 ROV/AUV
    Surface,          // USV
    Multirotor,       // 多旋翼
    FixedWing,        // 固定翼
    VTOL              // 垂直起降
};
```

### 参数来源

- `engine/Content/Fossen/<vehicle>_params.json`：海洋载具。
- `engine/Content/Aero/<vehicle>_params.json`：空中载具。
- 内置回退参数。

### 惯量归一化增益

控制器增益按车辆惯量缩放：

- 俯仰/偏航力矩环按 `M44_pitch` 缩放。
- 前进力环按 `mass_total` 缩放。

这样不同载具可获得相似的闭环动态响应。

---

## 测试覆盖

`tests/` 下共有 13 个测试：

| 测试 | 验证内容 |
|---|---|
| `test_dds_cdr` | CDR 序列化、GNCSetpoint 解码、异常输入处理 |
| `test_dds_cdr_writer` | `dds_cdr::Writer` 往返与溢出检测 |
| `test_latest_value_mailbox` | 单槽覆盖、序列号跟踪 |
| `test_nonblocking_frame_sender` | 非阻塞帧发送、背压丢弃 |
| `test_dds_connection_state` | 连接代次、过期设定点拒绝 |
| `test_ue_control_session` | UE 控制会话阶段、设定点门控、超时 |
| `test_fossen_vehicle_params` | 车辆参数加载、构型选择、惯量归一化 |
| `test_air_allocators` | 多旋翼、VTOL 分配 |
| `test_ekf` | 静态零漂、无加速度计回退、零偏收敛、倾斜观测、航向辅助、水流估计 |
| `test_sitl_config` | 命令行解析与验证 |
| `test_sitl_xlog` | XLog 写入有效性 |
| `test_sensor_adapter` | 传感器适配、DVL/GPS/真值时效 |
| `test_mavlink_signing` | MAVLink 签名、重放、篡改检测 |
| `test_xlog_writer` | CRC、分段、序列单调性、损坏检测 |

---

## 关键设计模式

### 策略模式

`IController` 和 `IAllocator` 把控制律与执行器布局解耦。

### 工厂模式

`build_control_stack()` 根据载具构型组装控制器和分配器。

### 适配器模式

`SensorAdapter` 把原始 MAVLink 帧转换为类型化的 `NavigationInput`。

### 非阻塞线程边界

- GNC 实时循环永不等待 DDS 或 TCP。
- `LatestValueMailbox` 在线程间传递最新值。
- DDS worker 在后台线程执行所有 XRCE-DDS I/O。

### 帧原子传输

`NonblockingFrameSender` 保证部分发送的帧尾在新帧之前完成，背压下仅丢弃完整未发送的帧。

### 会话门控

UE TCP 和 DDS 连接均使用会话代次。重连后旧会话的设定点被拒绝，防止陈旧命令。

---

## 数据流总结

```text
UE5/OceanX
   │ HIL_SENSOR/HIL_GPS/HIL_DVL (TCP)
   ▼
TcpTransport ──► MavlinkHIL ──► SensorAdapter ──► NavigationInput
                                                    │
                                                    ▼
                                                  EKF.update()
                                                    │
                                                    ▼
                                                 AUVState
                                                    │
                              ┌─────────────────────┼─────────────────────┐
                              ▼                     ▼                     ▼
                        IController           IAllocator            DDS publisher
                              │                     │                     │
                              ▼                     ▼                     ▼
                           Wrench              ActuatorCmd        FcSnapshot -> ROS 2
                              │                     │
                              └──────────┬──────────┘
                                         ▼
                              HIL_ACTUATOR_CONTROLS (TCP)
                                         │
                                         ▼
                                      UE5/OceanX
```

---

## 与 OceanX 的集成

OceanX 通过 Git Submodule 引用 HydroX：

```text
OceanX/
├── HydroX/          # submodule -> https://github.com/xuheda/HydroX
├── engine/          # Unreal Engine 项目
├── ros/             # ROS 2 包
└── ...
```

构建 OceanX 前：

```bash
git submodule update --init --recursive
cd HydroX
git clone --depth 1 --branch 3.4.0 https://gitee.com/mirrors/eigen.git third_party/eigen
```

然后运行 OceanX 的构建脚本，会调用 `HydroX/build_sitl.bat` 编译 `hydrox_sitl.exe` 并部署到 `engine/Binaries/ThirdParty/HydroX/`。

HydroX 与 OceanX 之间仅通过 **MAVLink HIL 协议** 通信，因此两者可以独立开发、独立版本发布。

---

> 最后更新：2026-07-24
