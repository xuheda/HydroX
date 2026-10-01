# HydroX HITL Command Simulator

`hydrox_hitl_command_simulator` has two roles: OceanX uses it as the managed
HITL ROS bridge, while developers can also run it as a standalone TELEM2
diagnostic sender. The managed bridge republishes flight-controller estimates
to the same ROS 2 API used by SITL and converts the final ROS setpoint to the
HXSP v2 frames decoded by the FMUv6C firmware.

## Transport

- TELEM2: 921600 baud, 8-N-1, 3.3 V TTL.
- RTS/CTS hardware flow control is enabled by default.
- The desktop serial port is exclusive. Do not run the simulator and the real
  RK3588 command service on the same adapter at the same time.
- TELEM1 remains owned by `hydrox_hitl_router`; this tool never opens TELEM1.

## Safety

Standalone diagnostic mode defaults to `disabled`; every non-disabled static or
mission command requires the explicit `--arm-command` option. Managed ROS bridge
mode accepts commands only from the final DDS setpoint endpoint and fails closed
to DISABLED if DDS, the flight-controller estimate, or the setpoint lease is
stale. On Ctrl+C, normal duration expiry, or a clean stop, the tool transmits
five DISABLED frames before closing the port. The FMUv6C runtime independently
revokes a stale command after 500 ms.

A successful serial write is not an acknowledgement from the flight
controller. Until the TELEM2 acknowledgement protocol is added, acceptance
must be verified from vehicle response and TELEM1 runtime behavior. The JSON
status therefore reports `board_acknowledged: false`.

## Managed ROS bridge

OceanX launches one bridge per vehicle and owns its lifetime. The upper API is
identical in SITL and HITL:

- state: `/hydrox/<vehicle>/out/state_estimate`
- setpoint: `/hydrox/<vehicle>/in/setpoint`

The lower HITL path is `TELEM1 -> HIL Router -> state UDP -> DDS` for state and
`DDS -> HXSP v2 -> TELEM2` for commands. Before simulation begins, serial and DDS
endpoints can be ready while `fc_state_fresh` and `setpoint_fresh` remain false.
That is a valid preflight state; real estimator data is published only after the
flight-controller runtime starts receiving simulated sensors.

Example managed bridge invocation:

```powershell
./build/sitl/Release/hydrox_hitl_command_simulator.exe `
  --serial COM12 --mode disabled --vehicle ecaa9_test `
  --dds-host 127.0.0.1 --dds-port 8888 --ros-domain-id 0 `
  --dds-client-key 14600 --state-udp-bind 127.0.0.1 `
  --state-udp-port 14700
```

## Examples

Validate arguments without opening a serial port:

```powershell
.\build\sitl\Release\hydrox_hitl_command_simulator.exe `
  --serial COM12 --mode disabled --validate-only
```

Run a two-second neutral link test:

```powershell
.\build\sitl\Release\hydrox_hitl_command_simulator.exe `
  --serial COM12 --mode disabled --duration-s 2
```

Command an AUV waypoint in NED coordinates:

```powershell
.\build\sitl\Release\hydrox_hitl_command_simulator.exe `
  --serial COM12 --mode waypoint-3d `
  --north-m 30 --east-m 10 --down-m 5 `
  --surge-mps 1.2 --lookahead-m 8 --arrival-radius-m 2 `
  --arm-command
```

Use `--help` for all setpoint and transport options.
