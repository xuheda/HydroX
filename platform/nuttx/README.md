# HydroX NuttX platform adapter

This directory contains operating-system adapters shared by all NuttX boards.
It must not contain Pixhawk 6C pin assignments, buses, DMA channels, memory
layout, or board bring-up logic; those belong in `boards/fmu_v6c`.

The adapters include the monotonic clock/sleeper, non-blocking serial stream,
critical section, task, storage and watchdog boundaries used by the shared
runtime. The following platform services remain incomplete for a real-vehicle
release:

- task creation and stack-watermark reporting;
- parameter storage and atomic update;
- auditable runtime health/profile reporting and production parameter policy.

These sources are intentionally not part of the PC CMake build. They are
compiled by the pinned NuttX firmware build with the FMUv6C board
configuration.
