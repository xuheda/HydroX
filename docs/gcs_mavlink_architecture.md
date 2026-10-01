# HydroX GCS/MAVLink architecture

HydroX exposes two independent external interfaces, following the same boundary used by PX4:

```
XGC/GCS <-> MAVLink 2 <-> HydroX runtime
ROS 2   <-> XRCE-DDS  <-> HydroX runtime
```

The GCS link never writes actuator outputs. Incoming commands are decoded into typed requests and are accepted or rejected by the HydroX runtime. The runtime owns arming, control-session freshness, mode transitions, failsafe behavior, allocation, and final actuator authorization.

## Layers

- `gcs::MavlinkProtocol`: generated MAVLink common-message encoding/decoding, target filtering, command acknowledgements, parameter discovery, and telemetry scheduling.
- `sitl::UdpSender`: non-blocking UDP transport. It initially transmits to the configured QGC address and learns the last valid peer for replies.
- `runtime::HilRuntime`: authoritative arming and actuator-safety state. Armed state is distinct from actuator authorization.
- XGC custom firmware plugin: maps `MAV_AUTOPILOT_GENERIC` and HydroX custom modes to operator-facing names and controls.

## Safety invariants

1. A GCS command cannot directly produce actuator output.
2. Disarming revokes the active setpoint and publishes a zero-output safe state.
3. Re-arming does not replay a pre-disarm setpoint; a fresh setpoint or mode request is required.
4. `HEARTBEAT.base_mode.SAFETY_ARMED` reports vehicle arming, while actuator authorization remains a separate HydroX status.
5. Commands are target-filtered by system/component and return `COMMAND_ACK` for accepted and rejected command-protocol operations.
6. ROS 2/DDS remains independent of the GCS link; loss of the UI does not silently rewrite the active DDS control source.
7. With active supervision, recovered setpoints are candidates, not permission to clear a latched failsafe. Air/ground vehicles reject `SURFACE` at input and final authorization.

## Explicit takeover from failsafe (SITL)

`COMMAND_LONG` with `MAV_CMD_DO_PAUSE_CONTINUE`, `param1=1`, is decoded as
`CommandKind::RESUME_CONTROL`. `GcsRuntimeBridge` calls
`HilRuntime::resume_external_control()` and returns `COMMAND_ACK`:

- `ACCEPTED` only if the runtime authorizes takeover; it does not disarm or stop rotors.
- `TEMPORARILY_REJECTED` if prerequisites fail, with a status-text explanation.

Supply a fresh external reference **before** requesting continue. Continue does
not carry a waypoint, revive an expired command or start a new mission itself.
The runtime requires active supervision, armed state, a valid current session,
an unexpired candidate, recent sensor input and non-degraded navigation.
Allowed recovery states are `FailsafeStabilize`, `FailsafeHold` and
`FailsafeSurface`; a VTOL transition fault can be acknowledged only in Hover.
`EmergencyAbort`, `OutputDisabled` and `FaultLocked` cannot be cleared this way.

Only `param1=1` is implemented here; `param1=0` does not provide a pause command.
The disarmed `acknowledge_safety_fault()` API has no new GCS mapping in this
change. Neither this protocol support nor the runtime API implies an added
OceanX button, ROS service, or embedded-board command bridge.

SITL protection is active; FMUv6C still uses shadow decisions until its release
gates pass. Air policies are hold/loiter, with no automatic return or landing.
See [the policy matrix](../src/safety/README.md). The protocol mapping is covered
by `test_gcs_mavlink`; runtime latching/takeover by `test_multidomain_failsafe`.

## Network convention

All local HydroX vehicles should transmit to the same XGC UDP listener, normally `127.0.0.1:14550`. Each vehicle must use a unique `--mavlink-system-id` in `[1, 255]`; UDP source ports may remain ephemeral. This is the PX4-style multi-vehicle topology and avoids configuring a separate XGC listening port per simulated vehicle.

For a single vehicle, the default system ID is `1`. `MAV_COMP_ID_AUTOPILOT1` remains component ID `1`, which XGC requires when creating the vehicle object.

## Initial standard MAVLink surface

- Discovery/status: `HEARTBEAT`, `SYS_STATUS`, `AUTOPILOT_VERSION`, `STATUSTEXT`
- Navigation: `ATTITUDE`, `LOCAL_POSITION_NED`, `GLOBAL_POSITION_INT`, `VFR_HUD`
- Control: `COMMAND_LONG`, `COMMAND_ACK`, `SET_MODE`
- Link support: `PING`, `TIMESYNC`, `MAV_CMD_REQUEST_MESSAGE`, `MAV_CMD_SET_MESSAGE_INTERVAL`
- Parameters: `PARAM_REQUEST_LIST`, `PARAM_REQUEST_READ`, `PARAM_SET`, `PARAM_VALUE`

Mission Protocol and HydroX-specific sonar/control-status messages are added on top of this boundary without changing actuator ownership.
