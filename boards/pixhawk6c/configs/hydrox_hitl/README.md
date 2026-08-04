# `hydrox_hitl` NuttX configuration

This directory will contain the version-pinned NuttX `defconfig` for the
`hydrox_pixhawk6c_hitl` firmware.

The first bootable configuration must enable only the facilities required for
board bring-up and HIL:

- STM32H743 core, FPU, clocks, cache, and MPU;
- NSH/debug console during bring-up;
- monotonic timer;
- one non-blocking OceanX HIL link;
- one non-blocking RK3588 companion link;
- watchdog, reset reason, status LED, and basic persistent storage;
- stack coloring/watermarks and runtime diagnostics.

Physical PWM, DShot, CAN actuator output, RC input, and real sensor drivers stay
disabled in this HITL configuration. A concrete `defconfig` will be committed
only together with the pinned NuttX revision and a board port that boots on
real Pixhawk 6C hardware.
