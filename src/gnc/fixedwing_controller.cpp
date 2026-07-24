// Copyright (c) 2026 OceanX. Author: xuheda
#include "gnc/fixedwing_controller.h"

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

    FixedWingController::FixedWingController(const Params &p) : _p(p) {}

    Wrench FixedWingController::update(const AUVState &state, double /*dt*/)
    {
        Wrench tau;
        tau.setZero();
        if (_mode == GNCMode::DISABLED)
            return tau;

        double d_ref = _sp.depth_ref;
        double heading_ref = _sp.heading_ref;
        double speed_ref = (_sp.surge_ref > 0.1) ? _sp.surge_ref : _p.cruise_speed_mps;

        if (_mode == GNCMode::WAYPOINT_3D)
        {
            d_ref = _sp.wp_d;
            heading_ref = std::atan2(_sp.wp_e - state.eta[1],
                                     _sp.wp_n - state.eta[0]);
        }

        const double d_err = d_ref - state.eta[2];
        const double pitch_ref =
            clamp_abs(-_p.altitude_kp * d_err + _p.altitude_kd * state.nu[2],
                      _p.pitch_limit_rad);
        const double roll_ref =
            clamp_abs(_p.course_kp * wrap_pi(heading_ref - state.eta[5]),
                      _p.roll_limit_rad);

        // FixedWingAllocator interprets tau as actuator-normalized demands:
        // X=speed target, K=roll target, M=pitch target, N=yaw-rate target.
        tau[0] = speed_ref;
        tau[3] = roll_ref;
        tau[4] = pitch_ref;
        tau[5] = _sp.use_yaw_rate_ref ? _sp.yaw_rate_ref : 0.0;
        return tau;
    }
} // namespace hydrox
