// Copyright (c) 2026 OceanX. Author: xuheda
#include "gnc/multirotor_controller.h"

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

    MultirotorController::MultirotorController(const Params &p) : _p(p) {}

    Wrench MultirotorController::update(const NavigationState &state, double /*dt*/)
    {
        Wrench tau;
        tau.setZero();
        if (_mode == GNCMode::DISABLED)
            return tau;

        double d_ref = _sp.depth_ref;
        double heading_ref = _sp.heading_ref;
        double roll_ref = 0.0;
        double pitch_ref = 0.0;
        if (_mode == GNCMode::WAYPOINT_3D)
        {
            d_ref = _sp.wp_d;
            heading_ref = std::atan2(_sp.wp_e - state.eta[1],
                                     _sp.wp_n - state.eta[0]);

            const double yaw = state.eta[5];
            const double cy = std::cos(yaw);
            const double sy = std::sin(yaw);
            const double vel_n = cy * state.nu[0] - sy * state.nu[1];
            const double vel_e = sy * state.nu[0] + cy * state.nu[1];

            double acc_n =
                _p.xy_kp * (_sp.wp_n - state.eta[0]) - _p.xy_kd * vel_n;
            double acc_e =
                _p.xy_kp * (_sp.wp_e - state.eta[1]) - _p.xy_kd * vel_e;
            const double acc_norm = std::hypot(acc_n, acc_e);
            if (acc_norm > _p.max_xy_accel && acc_norm > 1.0e-9)
            {
                const double scale = _p.max_xy_accel / acc_norm;
                acc_n *= scale;
                acc_e *= scale;
            }

            const double acc_forward = cy * acc_n + sy * acc_e;
            const double acc_right = -sy * acc_n + cy * acc_e;
            pitch_ref = clamp_abs(-acc_forward / _p.g, _p.max_tilt_rad);
            roll_ref = clamp_abs(acc_right / _p.g, _p.max_tilt_rad);
        }

        const double z_err = d_ref - state.eta[2];
        const double z_accel_down = clamp_abs(_p.z_kp * z_err - _p.z_kd * state.nu[2],
                                              _p.max_z_accel);

        // Positive tau[2] is interpreted by the multirotor allocator as upward
        // collective thrust. In NED, negative z_accel_down means climb.
        tau[2] = _p.mass * (_p.g - z_accel_down);

        const double yaw = state.eta[5];
        const double yaw_err = wrap_pi(heading_ref - yaw);
        tau[3] = _p.mass * clamp_abs(_p.roll_kp * (roll_ref - state.eta[3]) - _p.roll_kd * state.nu[3],
                                     _p.max_tilt_rad);
        tau[4] = _p.mass * clamp_abs(_p.pitch_kp * (pitch_ref - state.eta[4]) - _p.pitch_kd * state.nu[4],
                                     _p.max_tilt_rad);
        if (_sp.use_yaw_rate_ref)
            tau[5] = _p.mass * (_p.yaw_kd * (_sp.yaw_rate_ref - state.nu[5]));
        else
            tau[5] = _p.mass * (_p.yaw_kp * yaw_err - _p.yaw_kd * state.nu[5]);

        return tau;
    }
} // namespace hydrox
