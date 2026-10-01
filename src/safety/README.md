# HydroX vehicle safety core

The supervisor owns arming/latching, the arbiter alone selects the effective
reference, and `VehicleFailsafeNavigator` generates domain-specific references
through the normal controller/allocator. `safety_profile_for(control)` is selected
alongside the actual vehicle control bundle, in both SITL and compiled HITL
profiles. A mismatched class/archetype is invalid. The former AUV-only navigator
and return-to-safe-point policy have been removed, with no compatibility aliases.

## Sustained external-control loss

Runtime component reports now flow through `HealthManager` before every safety
evaluation, including initialization and explicit recovery. Reports are rebuilt
from current facts; the supervisor alone owns fault latches. Platform validity,
IMU stream faults, position/vertical estimate availability, phase-transition
faults and invalid control output use this single aggregation path.

Only External, Failsafe and EmergencyAbort candidates exist. Unwired diagnostic,
manual and onboard-mission slots, the onboard-mission mode, and the unused
HardwareKill source have been removed without aliases. Physical output
inhibition remains independent of control candidates.

| Physical family | Action |
| --- | --- |
| Fin AUV | Short command hold; retain fin-control speed, then rate-limited ascent |
| Thruster ROV / hovering UUV | Reduce surge to zero; ascend using direct heave, including near-surface depth control |
| Twin-screw USV | Controlled stop; bounded-radius station keeping if position is available |
| Multirotor | Capture current position and altitude; hold |
| Fixed wing | Tangent-entry clockwise loiter at captured altitude and model-derived cruise speed/radius |
| Lift-plus-cruise VTOL | Hover: hold; cruise: loiter; either transition: recover to hover without resetting the physical phase |
| Differential-drive ground vehicle | Slew speed to zero and regulate zero yaw rate; keep braking control active |

Air vehicles do **not** return or automatically land. No protection action sets
their altitude reference to the water surface. SURFACE commands are rejected at
both the external-input and final authorization gates for air/ground vehicles.
Position hold means feedback control, not freezing the simulated rigid body.
ROV protection conservatively uses surge/heave/yaw, not an unproven lateral DP
capability inferred from controller gains.

VTOL reference/source changes preserve flight phase. A front transition exceeding
15 seconds or losing more than 3 metres aborts to back transition. A back transition
exceeding 20 seconds latches an alarm and continues lift/braking; it does not
declare hover until measured ground speed permits it. These are simulation
protection bounds, not certification of every aircraft's transition envelope.

## Navigation and recovery

A geographic hold/loiter requires a recently **accepted** EKF horizontal position
aid (3 seconds); controlled vertical motion requires an accepted vertical aid
(1 second). Mere finite estimated coordinates do not prove observability.
These acceptance windows use the runtime's platform monotonic clock; measurement
sample-age/fusion timing remains governed by the sensor timestamp contract.
Without position, air vehicles retain captured altitude/course (hover vehicles
level attitude) and report navigation degraded. This is not geographic station
keeping. USVs retain zero-speed/heading control. Loss of all vertical feedback,
nonfinite state, IMU stream timeout or sensor-time discontinuity inhibits outputs
and latches a hard fault; there is no implemented blind-flight controller.
This last-resort software inhibition cannot guarantee physical recovery.

Recovered setpoints are buffered without touching the running controller.
An airborne takeover requires a fresh command, current sensors, healthy navigation
and an explicit `resume_external_control()`. SITL exposes it as MAVLink
`MAV_CMD_DO_PAUSE_CONTINUE` with param1=1. It never disarms the vehicle; transition
faults can be acknowledged only after hover recovery. Hard output-inhibition
faults still require disarm, `acknowledge_safety_fault()`, rearm and fresh input.
The regular stream alone cannot clear any latch.
Continue does not supply a new target: a fresh candidate must already exist.
No OceanX UI button or ROS service is added by this mapping, and the disarmed
fault-acknowledgement API has no newly implemented GCS command.

## Validation and remaining release gates

SITL and FMUv6C use mandatory supervision through the same runtime. No shadow-only
mode, active-enable switch, or direct external-to-controller bypass remains.
Safety status is exposed as `safety_status`. Session configuration maps the command
loss timeout into SafetyProfile once; there is no second runtime timeout policy.
HITL still requires physical actuator outputs to be inhibited. Source unification
is not hardware qualification: protected-actuator timing and recovery testing
remains mandatory before deployment.

Sensor watchdogs remain active during onboard fallback and take priority over
simultaneous command loss. Invalid controller/allocator output latches inhibition.
A late packet cannot hide an elapsed loss deadline. A residual augmentor cannot
override a failsafe-owned wrench.

Simulator pause/disconnect suspends output without clearing the supervisor latch.
Reconnect/resume invalidates all candidate references and requires fresh sensor
and command input. Healthy sessions can resume; existing safety latches still
require explicit recovery. No controller integration runs while paused, and
paused wall time is not integrated into the first resumed control step.

`test_multidomain_failsafe` covers all seven families, cross-domain rejection,
VTOL handoff/timeout/altitude-loss recovery, real multirotor controller/allocation
through the runtime, GPS loss and explicit live takeover. Other control, sensor,
runtime, protocol and HITL tests remain part of the regression suite.

On 2026-09-19, HydroX code revision `df7d033` (OceanX integration `a297313f3`)
built successfully and passed all 52 CTest tests registered by that historical
build. The current HydroX PC build registers 54 CTest tests; the 52-test result
must not be read as the current test total. The multirotor
runtime case uses estimated feedback; the VTOL runtime handoff case deliberately
uses truth-debug feedback to isolate the phase/authority contract. Later
documentation-only commits do not expand this validation evidence.

These are policy/controller tests, not a full closed-loop world qualification.
Terrain/collision avoidance, water-entry damage, energy-aware return/landing,
leak/battery/actuator fault telemetry and navigation integrity beyond existing EKF
innovation gates are not established by these tests. HealthManager consumes the
existing runtime facts; this does not invent battery/leak/actuator reports that
do not exist.
