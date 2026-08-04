# HydroX runtime

This directory owns flight-runtime policy that is independent of the host OS,
RTOS, and controller board.

Public runtime contracts live under `include/hydrox/runtime`. Implementations
belong here. Runtime code may depend on `platform/api`, but it must not include
Windows, POSIX, NuttX, STM32, or Pixhawk headers directly.

Implemented runtime components:

- `periodic_scheduler.h`: drift-free fixed-period releases and missed-period
  accounting;
- `control_session.h`: connection epochs, sensor-ready command gating,
  command freshness, and authority revocation;
- `hil_runtime.{h,cpp}`: the common EKF → GNC → allocation → arming/failsafe
  pipeline used by both SITL and HITL;
- `hitl_supervisor.{h,cpp}` and `hitl_board.h`: non-blocking board loop,
  watchdog, companion-command generations, HIL reconnect and physical-output
  inhibit gate;
- `fixed_vector.h`, `fixed_frame_sender.h`, and `mavlink_deframer.h`:
  fixed-capacity embedded/wire primitives;
- `latest_value_topic.h` and `spsc_queue.h`: fixed-memory bus primitives.
