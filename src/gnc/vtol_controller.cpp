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

Wrench VtolController::update(const AUVState& state, double /*dt*/)
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
        heading_ref = std::atan2(de, dn);
        const double yaw = state.eta[5];
        const double forward_error = std::cos(yaw) * dn + std::sin(yaw) * de;
        const double right_error = -std::sin(yaw) * dn + std::cos(yaw) * de;
        pitch_ref = clamp_abs(-_p.xy_kp * forward_error / _p.g, _p.max_tilt_rad);
        roll_ref = clamp_abs(_p.xy_kp * right_error / _p.g, _p.max_tilt_rad);
        if (std::hypot(dn, de) > 20.0)
            speed_ref = (_sp.surge_ref > 0.1) ? _sp.surge_ref : _p.cruise_speed_mps;
    }

    const double z_error = depth_ref - state.eta[2];
    const double down_accel = clamp_abs(_p.z_kp * z_error - _p.z_kd * state.nu[2], 4.0);
    tau[0] = speed_ref;
    tau[2] = _p.mass * (_p.g - down_accel);
    tau[3] = clamp_abs(_p.roll_kp * (roll_ref - state.eta[3]) - _p.roll_kd * state.nu[3], 1.0);
    tau[4] = clamp_abs(_p.pitch_kp * (pitch_ref - state.eta[4]) - _p.pitch_kd * state.nu[4], 1.0);
    tau[5] = _sp.use_yaw_rate_ref
        ? clamp_abs(_p.yaw_kd * (_sp.yaw_rate_ref - state.nu[5]), 1.0)
        : clamp_abs(_p.yaw_kp * wrap_pi(heading_ref - state.eta[5]) - _p.yaw_kd * state.nu[5], 1.0);
    return tau;
}
}
