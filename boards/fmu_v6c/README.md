# FMUv6C / Pixhawk 6C Mini HITL board integration

This directory is the current HydroX NuttX board integration for the default
Pixhawk 6C Mini HITL target.

Implemented boundaries:

- NuttX 13.0.0 custom-board CMake, Kconfig, and `hydrox_hitl` defconfig;
- STM32H743 boot, linker, serial and USB diagnostic configuration;
- TELEM1/UART7 MAVLink HIL link at 921600 baud with RX/TX DMA over a
  three-wire TX/RX/GND TTL connection;
- TELEM2/UART5 companion command link at 921600 baud with DMA and flow control;
- bootloader-safe early MPU reset before NuttX clears RAM or starts devices;
- STM32 IWDG registration and the HydroX `NuttxWatchdog` adapter;
- reset-reason capture and early inhibition of all PWM outputs;
- PX4-compatible firmware metadata and packaging definition;
- one generated immutable registry containing all ten qualified profiles:
  `generic-auv-fin`, `EcaA9`, `LAUV`, `DesistekSaga`, `RexROV2`, `VRX_WAMV`,
  `X500`, `RCCessna`, `StandardVTOL`, and `R1Rover`, each tied to its exact
  source bytes by a 64-bit FNV-1a fingerprint and SHA-256 release evidence.

The NuttX external application is
`nuttx_apps/external/hydrox_hitl`. It consumes the embedded control source
manifest `cmake/hitl_core_sources.cmake`.

The release build is pinned to Apache NuttX 13.0.0, Arm GNU Toolchain
13.3.Rel1, and the Python dependencies in `requirements-fmuv6c-build.txt`.
On Windows, build and verify the target with:

```powershell
.\build_pixhawk6c_hitl.bat -Clean
.\build_pixhawk6c_hitl.bat -Offline
```

The first command may fetch missing pinned dependencies. The second command
proves that the installed dependency set is sufficient without network access.
Both routes produce `.elf`, `.bin`, `.hex`, `.map`, `.px4`, and a release
manifest under `build/fmu_v6c_hitl/firmware/`. Verification rejects a mismatched
board ID, oversized or corrupt package, invalid Intel HEX, invalid Cortex-M
vectors/link address, missing HITL-only configuration, enabled physical output
drivers, stale generated registry, changed source JSON, or any missing compiled
profile ID, fingerprint, or exact JSON byte sequence.

This is a verified `FLASH_CANDIDATE_HITL_ONLY`, not completed hardware
qualification. On the exact Pixhawk 6C Mini hardware revision, still verify the
bootloader transfer, USB console, UART/DMA throughput, watchdog reset,
safe-output behavior, reset reasons, brownout/reconnect behavior, and
long-duration HITL operation. The Windows host bench has a strict preflight and
lifecycle tool at `../../../tools/hitl_bench.ps1`; its operator runbook is
`../../../doc/internal/hitl_bench_setup.html`. The preflight validates
repository and host artifacts but cannot prove which firmware is flashed.

The board boots into the safe `generic-auv-fin` default but does not silently
substitute it for another vehicle. Before the Router connects Unreal Engine it
must send an exact profile ID, fingerprint, and nonzero nonce. A different
valid profile revokes the previous command epoch, emits a neutral command,
destroys the old estimator/controller/allocator, rebuilds the selected stack,
and only then acknowledges `READY` with the same nonce. Unknown identities are
rejected. See `../../docs/sitl_hitl_parity.html` for the complete parity
contract. Keep TELEM1 at 921600 baud with RTS/CTS disabled on both endpoints
and all physical PWM/CAN actuator loads disconnected for the documented bench.
TELEM2 is a separate companion link and retains crossed RTS/CTS hardware flow
control.

## Application-to-bootloader maintenance

Firmware containing this maintenance path can be upgraded by the official PX4
uploader through TELEM1; GPS1 is not needed for subsequent updates. Keep both
TELEM1 and the board USB cable connected, then run the uploader against the
TELEM1 FTDI port at the HydroX flight-stack baud rate:

```powershell
python <px4_uploader.py> `
  --port COM11 `
  --baud-flightstack 921600 `
  build/fmu_v6c_hitl/firmware/hydrox_pixhawk6cmini_hitl.px4
```

Replace `COM11` with the current TELEM1 port. The application accepts only the
official MAVLink 1 broadcast-then-system-1 reboot pair within one second, and
only while physical actuator outputs are inhibited, no Session is active, and
the independent TELEM2 command link has no live peer. It neutralizes the HITL
actuator epoch, writes PX4's `0xB007B007` signature to `STM32_RTC_BK0R`, closes
the links, and resets. The resident bootloader then appears on USB and the same
uploader process continues the transfer.

This path cannot bootstrap an older HydroX image that does not contain it. Such
a board still needs one-time bootloader entry through GPS1 serial recovery,
SWD, or BOOT0; after this image is installed, normal TELEM1-to-USB maintenance
does not require moving the FTDI adapter to GPS1.
