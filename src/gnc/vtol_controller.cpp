// Copyright (c) 2026 OceanX
#include "gnc/vtol_controller.h"

#include <algorithm>
#include <cmath>

namespace hydrox
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
double clamp_abs(double v, double limit) { return std::max(-limit, std::min(limit, v)); }
double wrap_pi(double a)
{
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}
}

VtolController::VtolController(const Params& p) : _p(p) {}

Wrench VtolController::update(const NavigationState& state, double /*dt*/)
{
    Wrench tau;
    tau.setZero();
    if (_mode == GNCMode::DISABLED)
        return tau;

    double depth_ref = _sp.depth_ref;
    double heading_ref = _sp.heading_ref;
    double roll_ref = 0.0;
    double pitch_ref = 0.0;
    double speed_ref = std::max(0.0, _sp.surge_ref);
    if (_mode == GNCMode::WAYPOINT_3D)
    {
        depth_ref = _sp.wp_d;
        const double dn = _sp.wp_n - state.eta[0];
        const double de = _sp.wp_e - state.eta[1];
        const double horizontal_distance = std::hypot(dn, de);
        if (horizontal_distance > 20.0)
            speed_ref = (_sp.surge_ref > 0.1) ? _sp.surge_ref : _p.cruise_speed_mps;
        // In lift-rotor flight the aircraft can translate laterally while
        // maintaining heading.  Forcing its nose toward every short waypoint
        // couples the yaw and horizontal loops and caused the StandardVTOL to
        // reverse course before its yaw loop settled.  Hold the entry heading
        // until a dedicated wing-borne transition controller takes over.
        heading_ref = _heading_hold_valid ? _heading_hold : state.eta[5];
        const double yaw = state.eta[5];
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);
        // Position feedback alone drives a VTOL to its tilt limit, then keeps
        // accelerating through short waypoint legs.  Work in NED first so the
        // existing derivative gain damps ground speed independently of yaw.
        const double vel_n = cy * state.nu[0] - sy * state.nu[1];
        const double vel_e = sy * state.nu[0] + cy * state.nu[1];
        double accel_n = _p.xy_kp * dn - _p.xy_kd * vel_n;
        double accel_e = _p.xy_kp * de - _p.xy_kd * vel_e;
        // FollowWaypoints.speed is a speed command, not merely a pusher
        // setting.  Once the lift-rotor aircraft reaches that ground speed,
        // suppress further position acceleration and brake in the velocity
        // loop.  This prevents short legs from building transition-level
        // speed before the controller can arrest them.
        const double horizontal_speed = std::hypot(vel_n, vel_e);
        const double max_horizontal_speed = std::max(0.2, speed_ref);
        if (horizontal_speed > max_horizontal_speed)
        {
            accel_n = -_p.xy_kd * vel_n;
            accel_e = -_p.xy_kd * vel_e;
        }
        const double accel_forward = cy * accel_n + sy * accel_e;
        const double accel_right = -sy * accel_n + cy * accel_e;
        pitch_ref = clamp_abs(-accel_forward / _p.g, _p.max_tilt_rad);
        roll_ref = clamp_abs(accel_right / _p.g, _p.max_tilt_rad);
    }

    const double z_error = depth_ref - state.eta[2];
    const double down_accel = clamp_abs(_p.z_kp * z_error - _p.z_kd * state.nu[2], 4.0);
    // A FollowWaypoints goal is currently executed in lift-rotor flight.  Its
    // speed field limits the horizontal guidance loop above, but it must not
    // also drive the pusher: with heading held at waypoint entry, a positive
    // pusher command propels the airframe along that stale heading even when
    // the waypoint lies behind it.  Reserve tau[0] for explicit surge /
    // wing-borne modes until a transition controller owns the handover.
    tau[0] = (_mode == GNCMode::WAYPOINT_3D) ? 0.0 : speed_ref;
    tau[2] = _p.mass * (_p.g - down_accel);
    tau[3] = clamp_abs(_p.roll_kp * (roll_ref - state.eta[3]) - _p.roll_kd * state.nu[3], 1.0);
    tau[4] = clamp_abs(_p.pitch_kp * (pitch_ref - state.eta[4]) - _p.pitch_kd * state.nu[4], 1.0);
    tau[5] = _sp.use_yaw_rate_ref
        ? clamp_abs(_p.yaw_kd * (_sp.yaw_rate_ref - state.nu[5]), 1.0)
        : clamp_abs(_p.yaw_kp * wrap_pi(heading_ref - state.eta[5]) - _p.yaw_kd * state.nu[5], 1.0);
    return tau;
}
}
