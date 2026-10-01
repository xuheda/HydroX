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
- `hil_session_driver.{h,cpp}`: the transport-neutral HIL connection, sensor
  timestamp, pause/resume, maintenance, authority-revoke, and immediate-safe-
  actuator envelope shared by the host SITL loop and embedded supervisor;
- `hil_session_config.{h,cpp}` and `hil_session_mapping.h`: the versioned,
  fixed-point per-run configuration, canonical SHA-256 identity, validation,
  and the one mapping used by both SITL and HITL;
- `mavlink_hil.{h,cpp}` `HIL_CONTROL_TRACE` (message 11070): the wire-level
  per-control-tick snapshot used by HITL capture to preserve the same replay
  boundary as SITL XLog;
- `hitl_supervisor.{h,cpp}` and `hitl_board.h`: non-blocking board loop,
  watchdog, companion-command generations, HIL reconnect and physical-output
  inhibit gate;
- `fixed_vector.h`, `fixed_frame_sender.h`, and `mavlink_deframer.h`:
  fixed-capacity embedded/wire primitives;
- `latest_value_topic.h` and `spsc_queue.h`: fixed-memory bus primitives.

## Multi-domain protection (2026-09-19)

SITL and compiled FMUv6C profiles select `safety_profile_for(control)` from the
actual control bundle and always use the same supervisor and arbiter. There is no
shadow/active migration switch or alternate direct-control path. HITL physical
output inhibition and hardware qualification requirements remain independent.

`accept_setpoint()` buffers an external candidate; it
does not directly mutate the running controller. `VehicleSupervisor`,
`VehicleFailsafeNavigator` and `CommandArbiter` own the effective reference:
UUVs surface, USVs stop/station-keep, ground vehicles brake, hover aircraft hold,
and wing-borne aircraft loiter. Air vehicles never automatically return or land.
VTOL authority handoff preserves physical flight phase.

Accepted EKF position/vertical observations gate geographic and vertical
protection. Position loss degrades to supported non-geographic control;
unobservable vertical state, nonfinite state or sensor-stream/time failure
inhibits output. Output inhibition is not a guarantee of physical recovery.

`resume_external_control()` explicitly requests takeover from a controllable
latched fallback, with fresh commands/sensors and available navigation; it does
not disarm. Recovered packets alone cannot clear the latch. Hard faults require
disarm, `acknowledge_safety_fault()`, rearm and fresh input. Simulator
pause/resume creates a new session epoch without clearing fault latches. Fresh
sensor and command input are required; paused time is not integrated. Sensor-loss
watchdogs continue after command revocation and always outrank task-link loss.

See [the safety policy and validation boundaries](../safety/README.md) and
[the SITL GCS command mapping](../../docs/gcs_mavlink_architecture.md).

Health status has one construction path: `assess_health()` submits component
facts to `HealthManager`, then supplies its snapshot to the supervisor. Runtime
code no longer builds a parallel health summary. The aggregator does not latch
faults; the supervisor does. Only wired External/Failsafe/EmergencyAbort
candidate sources remain.
