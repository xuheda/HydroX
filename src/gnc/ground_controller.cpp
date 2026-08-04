// Copyright (c) 2026 OceanX. Author: xuheda
#include "gnc/ground_controller.h"

#include <algorithm>
#include <cmath>

namespace hydrox
{
namespace
{
    constexpr double kPi = 3.14159265358979323846;
    double clamp_abs(double value, double limit)
    {
        return std::max(-limit, std::min(limit, value));
    }
    double wrap_pi(double angle)
    {
        while (angle > kPi) angle -= 2.0 * kPi;
        while (angle < -kPi) angle += 2.0 * kPi;
        return angle;
    }
}

DifferentialDriveController::DifferentialDriveController(const Params &p) : _p(p) {}

void DifferentialDriveController::reset(const NavigationState &)
{
    _surge_integral = 0.0;
}

void DifferentialDriveController::set_mode(GNCMode mode)
{
    if (mode != _mode)
        _surge_integral = 0.0;
    _mode = mode;
}

Wrench DifferentialDriveController::update(const NavigationState &state, double dt)
{
    Wrench tau;
    tau.setZero();
    if (_mode == GNCMode::DISABLED)
        return tau;

    double surge_ref = _sp.surge_ref;
    double yaw_rate_ref = _sp.yaw_rate_ref;

    if (_mode == GNCMode::WAYPOINT_3D)
    {
        const double dn = _sp.wp_n - state.eta[0];
        const double de = _sp.wp_e - state.eta[1];
        const double distance = std::hypot(dn, de);
        const double bearing = distance > 1.0e-6 ? std::atan2(de, dn) : state.eta[5];
        const double heading_error = wrap_pi(bearing - state.eta[5]);
        yaw_rate_ref = clamp_abs(_p.yaw_heading_kp * heading_error,
                                 std::max(0.0, _p.max_yaw_rate_radps));

        if (distance <= _p.waypoint_stop_radius_m)
            surge_ref = 0.0;
        else
        {
            const double slowdown = std::min(1.0,
                (distance - _p.waypoint_stop_radius_m) /
                std::max(_p.waypoint_slowdown_m, 1.0e-6));
            surge_ref = std::min(std::max(0.0, _sp.surge_ref),
                                 _p.waypoint_surge_mps) * slowdown;
            // Turn in place when the waypoint lies behind the rover.
            surge_ref *= std::max(0.0, std::cos(heading_error));
        }
    }
    else if (!_sp.use_yaw_rate_ref)
    {
        const double heading_error = wrap_pi(_sp.heading_ref - state.eta[5]);
        yaw_rate_ref = clamp_abs(_p.yaw_heading_kp * heading_error,
                                 std::max(0.0, _p.max_yaw_rate_radps));
    }

    const double surge_error = surge_ref - state.nu[0];
    const double control_dt = std::max(0.0, std::min(dt, 0.1));
    const double candidate_integral = clamp_abs(
        _surge_integral + surge_error * control_dt,
        std::max(0.0, _p.surge_integral_limit));
    const double unsaturated_force =
        _p.surge_kp * surge_error + _p.surge_ki * candidate_integral;
    if (std::abs(unsaturated_force) < _p.max_force_N ||
        unsaturated_force * surge_error < 0.0)
        _surge_integral = candidate_integral;

    tau[0] = clamp_abs(
        _p.surge_kp * surge_error + _p.surge_ki * _surge_integral,
        std::max(0.0, _p.max_force_N));
    tau[5] = clamp_abs(
        _p.yaw_rate_kp * (yaw_rate_ref - state.nu[5]),
        std::max(0.0, _p.max_moment_Nm));
    return tau;
}
} // namespace hydrox
