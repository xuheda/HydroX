# Pixhawk 6C board support

This is the only Pixhawk 6C-specific boundary in HydroX. The board uses an
STM32H743, but application and runtime code must not depend directly on STM32
or Pixhawk headers.

Stage 2 uses Apache NuttX as the RTOS and MCU platform. HydroX owns the board
port, logical device mapping, platform adapters, runtime, and flight stack; it
does not use PX4 modules, uORB, Commander, PX4 parameters, or PX4 logging.

Before `hydrox_pixhawk6c_hitl.bin` can be built, this directory must provide:

- a version-pinned NuttX custom board port for Pixhawk 6C;
- a bootloader-compatible linker script and verified flash/RAM layout;
- STM32H743 clock, cache, MPU, DMA-memory, and device initialization;
- non-blocking OceanX HIL and RK3588 companion devices;
- monotonic time, watchdog, reset reason, console, and status LED;
- a `hydrox_hitl` configuration that keeps physical actuator outputs disabled.
- a `hydrox_hitl_board()` implementation of `runtime::HitlBoard` that exposes
  those devices, loads a validated vehicle profile, and reports the matching
  VehicleBundle profile ID and FNV-1a fingerprint;

Expected layout:

```text
boards/pixhawk6c/
├── CMakeLists.txt
├── Kconfig
├── configs/
│   └── hydrox_hitl/
│       └── defconfig
├── include/
│   └── board.h
├── scripts/
│   └── memory.ld
└── src/
    ├── Makefile
    ├── board_bringup.c
    ├── board_devices.c
    ├── board_memory.c
    └── hydrox_hitl_board.cpp
```

NuttX board layout depends on the pinned NuttX revision, so the concrete files
above are added together after the upstream revision is selected. The
repository intentionally does not provide a fake BSP target or copied memory
map. The build stops until real hardware bring-up files exist.

Everything above this boundary is implemented: the NuttX app entry validates
the profile and actuator inhibit, the supervisor services watchdog and links,
and the shared runtime executes sensor parsing, EKF, GNC, allocation,
arming/failsafe and explicit zero/unarmed output. The BSP must not reproduce
any of those policies.
