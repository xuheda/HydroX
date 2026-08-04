// Copyright (c) 2026 OceanX. Author: xuheda
#include "gnc/surface_controller.h"

#include <algorithm>
#include <cmath>

namespace hydrox
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;

        double clamp_abs(double v, double limit)
        {
            return std::max(-limit, std::min(limit, v));
        }

        double wrap_pi(double a)
        {
            while (a > kPi)
                a -= 2.0 * kPi;
            while (a < -kPi)
                a += 2.0 * kPi;
            return a;
        }
    } // namespace

    SurfaceVesselController::SurfaceVesselController(const Params &p) : _p(p) {}

    Wrench SurfaceVesselController::update(const NavigationState &state, double dt)
    {
        Wrench tau;
        tau.setZero();
        if (_mode == GNCMode::DISABLED)
            return tau;

        double heading_ref = _sp.heading_ref;
        double surge_ref = _sp.surge_ref;

        if (_mode == GNCMode::WAYPOINT_3D)
        {
            const double dn = _sp.wp_n - state.eta[0];
            const double de = _sp.wp_e - state.eta[1];
            const double dist = std::sqrt(dn * dn + de * de);
            if (dist > 0.2)
                heading_ref = std::atan2(de, dn);

            // Body sway is a directly observable sideslip disturbance.  Crab
            // into it so the *ground track*, rather than merely the bow, is
            // aligned with the waypoint line.  The cap prevents wave-induced
            // sway spikes from demanding an abrupt differential-thrust turn.
            const double effective_surge = std::max(0.15, std::abs(state.nu[0]));
            const double sideslip = std::atan2(state.nu[1], effective_surge);
            const double crab = clamp_abs(
                _p.sideslip_compensation_gain * sideslip,
                std::max(0.0, _p.max_crab_angle_rad));
            heading_ref = wrap_pi(heading_ref - crab);

            // The planner owns the requested approach speed.  Treat the
            // vehicle parameter as a safety ceiling rather than replacing
            // the mission command: otherwise a 0.5--1.0 m/s terminal hold
            // becomes a 3 m/s WAM-V pass-through.
            surge_ref = std::min({std::max(0.0, _sp.surge_ref),
                                  _p.waypoint_surge_mps,
                                  std::max(0.0, dist)});

            // The WAM-V can yaw with a differential (including opposed)
            // thrust pair.  Reduce propulsion while its bow is away from the
            // waypoint, so it turns into the line of sight instead of making
            // a large forward-only orbit around the terminal point.
            const double line_of_sight_error = wrap_pi(heading_ref - state.eta[5]);
            surge_ref *= std::max(0.0, std::cos(line_of_sight_error));
        }

        const double u = state.nu[0];
        const double r = state.nu[5];
        const double yaw_err = wrap_pi(heading_ref - state.eta[5]);
        const double control_dt = std::max(0.0, std::min(dt, 0.1));

        if (_mode == GNCMode::WAYPOINT_3D)
        {
            // A surface vessel has substantial passive hull drag.  Do not
            // reverse its waterjets merely because it has entered a waypoint
            // hold radius: that turns a gentle coast into an astern pass and
            // makes the terminal reacquisition oscillate.  Positive throttle
            // is used only to regain the planner-requested approach speed.
            const double surge_error = surge_ref - u;
            if (surge_ref <= 0.01)
            {
                _surge_error_integral = 0.0;
                tau[0] = 0.0;
            }
            else
            {
                // Do not wind up a positive integral at actuator saturation;
                // permit negative error to unwind it after a wave-assisted
                // overspeed.  The hull's passive drag still handles coasting.
                const double unsaturated = _p.surge_kp * surge_error +
                                           _p.surge_ki * _surge_error_integral;
                if (unsaturated < _p.max_force_N || surge_error < 0.0)
                {
                    _surge_error_integral += surge_error * control_dt;
                    _surge_error_integral = clamp_abs(
                        _surge_error_integral,
                        std::max(0.0, _p.surge_integral_limit));
                }
                tau[0] = std::max(0.0, std::min(
                    _p.max_force_N,
                    _p.surge_kp * surge_error +
                        _p.surge_ki * _surge_error_integral));
            }
        }
        else
        {
            _surge_error_integral = 0.0;
            tau[0] = clamp_abs(_p.surge_kp * (surge_ref - u) - _p.surge_kd * u,
                               _p.max_force_N);
        }
        if (_sp.use_yaw_rate_ref)
            tau[5] = clamp_abs(_p.yaw_kd * (_sp.yaw_rate_ref - r),
                               _p.max_moment_Nm);
        else
            tau[5] = clamp_abs(_p.yaw_kp * yaw_err - _p.yaw_kd * r,
                               _p.max_moment_Nm);

        return tau;
    }
} // namespace hydrox
