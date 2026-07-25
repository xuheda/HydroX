<a id="hydrox"></a>

<div align="center">
<img src="docs/assets/hydrox-hero-v2.png" alt="HydroX coordinated underwater, surface, and aerial vehicle concept" width="100%">

<h1>HydroX</h1>

<p><strong>Ocean-First, Cross-Domain Autopilot for Underwater, Surface, and Aerial Vehicles</strong></p>

<p>
A unified control and allocation architecture · Six vehicle archetypes · Designed for SIL, HIL, and real-world deployment
</p>

<p>
<a href="https://isocpp.org/"><img alt="C++ 17" src="https://img.shields.io/badge/C%2B%2B-17-00599C?style=flat-square&logo=cplusplus&logoColor=white"></a>
<a href="https://cmake.org/"><img alt="CMake 3.16 or newer" src="https://img.shields.io/badge/CMake-%E2%89%A53.16-064F8C?style=flat-square&logo=cmake&logoColor=white"></a>
<img alt="MAVLink HIL" src="https://img.shields.io/badge/Interface-MAVLink%20HIL-4B8BBE?style=flat-square">
<img alt="ROS 2 integration" src="https://img.shields.io/badge/Integration-ROS%202-22314E?style=flat-square&logo=ros&logoColor=white">
<a href="LICENSE"><img alt="Apache License 2.0" src="https://img.shields.io/badge/License-Apache--2.0-D22128?style=flat-square"></a>
</p>

<p>
<a href="#overview">Overview</a> ·
<a href="#architecture">Architecture</a> ·
<a href="#vehicle-coverage">Vehicles</a> ·
<a href="#integration-contract">Integration</a> ·
<a href="#quick-start">Quick Start</a>
</p>
<sub>Concept visualization of a coordinated fleet spanning underwater, surface, and aerial vehicles.</sub>

</div>

---

## Overview

HydroX is a standalone, open-source C++17 autopilot implementation for underwater, surface, and aerial vehicles. It implements the complete GNC path from sensor processing and state estimation to guidance, control, allocation, actuator output, telemetry, and logging.

HydroX uses the same GNC core for software-in-the-loop (SIL), hardware-in-the-loop (HIL), and onboard deployment, with platform-specific interfaces handling sensors, actuators, and communications.

> [!NOTE]
> A shared state, control, and actuator contract keeps the estimator, controller, and allocator chain consistent across development, validation, and deployment.

### At a glance

| | Engineering focus | What HydroX provides |
|---|---|---|
| 🧭 | **Navigation** | An 18-state aided EKF for IMU, DVL, GPS, depth, and heading observations |
| 🎛️ | **Control** | Vehicle-specific control laws behind a common <code>IController</code> interface |
| ⚙️ | **Allocation** | A clean six-degree-of-freedom wrench boundary between control and actuators |
| 🚀 | **Vehicle coverage** | Controllers and allocators for underwater vehicles, surface vessels, multirotors, fixed-wing aircraft, and VTOL aircraft |
| 🌊 | **Marine vehicle modeling** | Fossen-based vehicle control parameters for underwater vehicles and surface vessels |
| 🔌 | **Integration** | MAVLink HIL over TCP, Micro XRCE-DDS over UDP, and MAVLink telemetry for QGroundControl |
| 📈 | **Observability** | XLog time-series recording plus focused tests for estimation, allocation, transport, and serialization |

## Architecture

### One control chain, multiple domains

<p align="center">
<img src="docs/assets/gnc-pipeline.svg" alt="HydroX GNC pipeline from sensors through navigation, control, allocation, and actuators" width="100%">
</p>

HydroX separates **what forces and moments the vehicle needs** from **how its actuators produce them**. Every controller emits a six-degree-of-freedom wrench; a vehicle-specific allocator then maps that demand to fins, propellers, thrusters, rotors, or control surfaces.

That boundary keeps the estimator and GNC orchestration stable while vehicle geometry and actuator layouts change.

~~~text
Sensor frames
    → SensorAdapter
    → 18-state aided EKF
    → vehicle state
    → IController
    → 6-DOF force / moment demand
    → IAllocator
    → normalized actuator commands
    → actuator / transport interface
~~~

### Integration topology

~~~mermaid
flowchart LR
    SIL["SIL environment"]
    HIL["HIL bench"]
    REAL["Onboard computer"]
    HX["HydroX<br/>Autopilot"]
    AGENT["Micro XRCE-DDS Agent"]
    ROS["ROS 2 / mission application"]
    QGC["QGroundControl"]
    LOG["XLog recorder"]

    SIL <-->|"simulation I/O"| HX
    HIL <-->|"MAVLink HIL / hardware I/O"| HX
    REAL <-->|"platform I/O"| HX
    HX <-->|"telemetry + setpoint · UDP"| AGENT
    AGENT <-->|"DDS"| ROS
    HX -->|"MAVLink telemetry · UDP"| QGC
    HX -->|"binary time series"| LOG

    classDef core fill:#0a3048,stroke:#34d6ff,color:#ffffff,stroke-width:2px;
    classDef edge fill:#102334,stroke:#54768d,color:#ffffff;
    class HX core;
    class SIL,HIL,REAL,AGENT,ROS,QGC,LOG edge;
~~~

HydroX runs the same estimation, control, and allocation chain across SIL, HIL, and onboard environments. Each environment connects through the transport and platform interface appropriate to that deployment.

ROS 2 applications exchange telemetry, actuator outputs, and GNC setpoints through an external Micro XRCE-DDS Agent. QGroundControl receives a separate, one-way MAVLink/UDP telemetry stream.

### Navigation core

The aided EKF estimates NED position, Euler attitude, body-frame velocity, IMU biases, and NED current velocity. It accepts IMU propagation together with available DVL bottom/water track, GPS, depth, and heading observations.

<details>
<summary><strong>18-state vector</strong></summary>

~~~text
[N, E, D,
 roll, pitch, yaw,
 u, v, w,
 b_ax, b_ay, b_az,
 b_gx, b_gy, b_gz,
 c_N, c_E, c_D]
~~~

</details>

## Vehicle coverage

HydroX provides controller and allocator implementations for six vehicle archetypes. This describes software coverage, not flight certification or completed hardware validation.

| Domain | Vehicle archetype | Actuation / control allocation |
|---|---|---|
| Underwater | Slender-body AUV | Cross-tail fins and propeller |
| Underwater | Thruster ROV / hovering AUV | Multi-thruster six-DOF allocation |
| Surface | USV / WAM-V | Differential twin-propeller control |
| Aerial | Multirotor | Quad-rotor thrust mixing |
| Aerial | Fixed-wing | Elevator, aileron, rudder, and throttle |
| Aerial | VTOL | Lift rotors, control surfaces, and pusher |

## Execution modes

| Mode | How HydroX runs | Primary interfaces |
|---|---|---|
| **SIL** | The PC executable closes the control loop with a simulated vehicle | TCP MAVLink HIL and ROS 2 integration |
| **HIL** | The same control loop exchanges sensor and actuator data with a hardware bench | MAVLink HIL and the transport abstraction |
| **Real-world deployment** | The GNC core runs on an onboard computer with platform I/O | Static library, embedded target, UART, and platform adapters |

The default SIL control loop is configured for **100 Hz**. This is a configurable rate, not a hard real-time guarantee.

## Interfaces

| Peer | Transport | Direction | Default | Purpose |
|---|---|---|---|---|
| Simulator or adapter | MAVLink HIL over TCP | Bidirectional | <code>127.0.0.1:14600</code> | Sensor frames in; actuator controls out |
| Micro XRCE-DDS Agent | XRCE-DDS over UDP | Bidirectional | <code>127.0.0.1:8888</code> | ROS 2 telemetry and GNC setpoints |
| QGroundControl | MAVLink over UDP | HydroX → QGC | <code>255.255.255.255:14550</code> | Attitude, position, HUD, and system telemetry |
| XLog | Local binary file | HydroX → log | Configurable | Estimator, setpoint, control, allocation, and diagnostic records |

The ROS 2 topic namespace is vehicle-aware. See [<code>include/dds_topic_manifest.h</code>](include/dds_topic_manifest.h) for the authoritative topic names, directions, rates, and message types.

> [!IMPORTANT]
> HydroX publishes state and actuator data through DDS and subscribes to high-level GNC setpoints. Simulation and hardware integrations can bridge their native topics or I/O to the HydroX transport and message contracts.

## Integration contract

HydroX keeps the autopilot loop separate from the simulation or hardware endpoint. An integration only needs to exchange sensor data and actuator commands through the contracts below.

### MAVLink HIL endpoint

The SIL process opens a TCP connection to the configured MAVLink HIL endpoint. By default, it connects to <code>127.0.0.1:14600</code> and runs the control loop at 100 Hz.

| Direction | Required message | Purpose |
|---|---|---|
| Endpoint → HydroX | <code>HIL_SENSOR</code> | Required IMU and pressure data for estimator propagation |
| Endpoint → HydroX | <code>HIL_GPS</code> | Optional position and velocity aid |
| Endpoint → HydroX | <code>HIL_DVL</code> | Optional DVL body velocity aid; HydroX extension, message ID 11060 |
| HydroX → endpoint | <code>HIL_ACTUATOR_CONTROLS</code> | Normalized actuator commands for the simulated or physical plant |

The endpoint should apply each actuator command to its vehicle model or I/O layer, then return the next sensor sample. <code>HIL_SENSOR</code> is the minimum input required to advance the control loop.

### ROS 2 and DDS bridge

An optional Micro XRCE-DDS Agent exposes the autopilot to ROS 2. HydroX publishes vehicle state, sensor summaries, actuator outputs, and status under <code>/hydrox/&lt;vehicle&gt;/out/...</code>, and receives high-level GNC setpoints from <code>/hydrox/&lt;vehicle&gt;/in/setpoint</code>.

For a ROS 2 based simulator or hardware integration, a bridge node can map native sensor topics to the MAVLink HIL input contract and map HydroX actuator outputs to the native actuator interface. The complete topic names, message types, directions, and rates are defined in [<code>include/dds_topic_manifest.h</code>](include/dds_topic_manifest.h).

## Quick start

### Prerequisites

- Git
- CMake 3.16 or newer
- A C++17 compiler
  - Windows: Visual Studio 2022 recommended
  - Linux: GCC or Clang, plus OpenSSL development files
- Eigen 3.4
- Network access during the first configure so CMake can fetch Micro-CDR and Micro XRCE-DDS Client

### 1. Clone HydroX and prepare Eigen

~~~bash
git clone https://github.com/xuheda/HydroX.git
cd HydroX

git clone --depth 1 --branch 3.4.0 \
  https://gitlab.com/libeigen/eigen.git \
  third_party/eigen
~~~

If GitLab is slow or unavailable in your region:

~~~bash
git clone --depth 1 --branch 3.4.0 \
  https://gitee.com/mirrors/eigen.git \
  third_party/eigen
~~~

### 2. Build the SIL executable

#### Windows

~~~powershell
.\build_sil.bat
~~~

Output:

~~~text
build_sil\Release\hydrox_sil.exe
~~~

#### Linux

~~~bash
bash build_sil.sh
~~~

Output:

~~~text
build_sil_linux/hydrox_sil
~~~

### 3. Run

Start a simulation or HIL endpoint first, then launch HydroX:

~~~powershell
.\build_sil\Release\hydrox_sil.exe
~~~

~~~bash
./build_sil_linux/hydrox_sil
~~~

The default endpoints are the MAVLink HIL connection at <code>127.0.0.1:14600</code>, the Micro XRCE-DDS Agent at <code>127.0.0.1:8888</code>, and the QGroundControl broadcast port at <code>14550</code>. These endpoints are configurable at launch.

To deploy the Windows SIL executable beside an external integration runtime:

~~~powershell
.\build_sil.bat hydrox_sil C:\Path\To\Runtime\Dir
~~~

### 4. Build and run tests

Windows:

~~~powershell
cmake --build build_sil --config Release
ctest --test-dir build_sil -C Release --output-on-failure
~~~

Linux:

~~~bash
cmake --build build_sil_linux --parallel
ctest --test-dir build_sil_linux --output-on-failure
~~~

The test suite covers EKF behavior, control allocation, DDS/CDR serialization, connection state, sensor adaptation, MAVLink signing, SIL configuration, logging, and transport helpers.

## Build targets

| Target | Role |
|---|---|
| <code>hydrox</code> | Reusable static GNC library |
| <code>hydrox_sil</code> | Windows/Linux SIL reference executable |
| <code>hydrox_pixhawk6c</code> | STM32H7 / Pixhawk 6C embedded deployment target |

> [!NOTE]
> Embedded deployments reuse the same GNC core through the platform adapter layer. Board-specific HAL, RTOS, watchdog, UART, and startup integration are supplied by the target platform.

## Project map

| Path | Responsibility |
|---|---|
| [<code>include/</code>](include/) | Public interfaces, state types, estimator, and transport contracts |
| [<code>src/</code>](src/) | Core library implementation |
| [<code>src/gnc/</code>](src/gnc/) | Controllers, control allocators, reference models, and factory |
| [<code>src/sil/</code>](src/sil/) | SIL configuration, platform support, DDS worker, and XLog integration |
| [<code>src/transport/</code>](src/transport/) | TCP and UART transport implementations |
| [<code>apps/sil/</code>](apps/sil/) | <code>hydrox_sil</code> entry point and 100 Hz control orchestration |
| [<code>apps/stm32/</code>](apps/stm32/) | Embedded deployment entry point |
| [<code>tests/</code>](tests/) | Focused unit tests |
| [<code>docs/</code>](docs/) | Architecture notes and project assets |

## Documentation

- [Codebase overview](docs/codebase_overview.md): module boundaries and control flow
- [DDS topic manifest](include/dds_topic_manifest.h): authoritative ROS 2 / DDS interface contract
- [Third-party notices](THIRD_PARTY_NOTICES.md): dependency licenses and attribution

## Contributing

Issues and pull requests are welcome. When adding a new vehicle archetype, keep control-law logic behind <code>IController</code>, actuator mapping behind <code>IAllocator</code>, and extend the relevant tests with the implementation.

## License

HydroX is released under the [Apache License 2.0](LICENSE). Third-party components are documented in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
