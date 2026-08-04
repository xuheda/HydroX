# HydroX NuttX platform adapter

This directory contains operating-system adapters shared by all NuttX boards.
It must not contain Pixhawk 6C pin assignments, buses, DMA channels, memory
layout, or board bring-up logic; those belong in `boards/pixhawk6c`.

The first adapters are the monotonic clock and monotonic sleeper used by the
shared periodic scheduler. The following adapters remain to be
implemented as the Pixhawk 6C board port becomes available:

- task creation and stack-watermark reporting;
- UART/USB byte streams backed by non-blocking NuttX drivers;
- parameter storage and atomic update;
- hardware watchdog service;
- reset-reason and health reporting.

These sources are intentionally not part of the PC CMake build. They are
compiled by the pinned NuttX firmware build after the board configuration has
been created.
