# Pixhawk 6C board sources

Only board-specific code belongs here:

- clock tree and boot-time pin setup;
- bootloader-compatible flash and RAM layout;
- DMA-safe memory regions and cache maintenance;
- device registration and logical-port-to-device mapping;
- board status LED, watchdog, and reset-reason integration.

Protocol parsing, HIL policy, scheduling, estimation, GNC, and allocation must
not be implemented in this directory.
