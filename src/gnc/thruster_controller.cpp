// Copyright (c) 2026 OceanX. Author: xuheda
/** thruster_controller.cpp — 6-DOF wrench controller for ROV / hovering AUV. */
#include "gnc/thruster_controller.h"
#include <algorithm>

namespace hydrox
{
    namespace
    {
        constexpr double kPi = 3.14159265358979;

        double clamp_abs(double v, double m)
        {
            return std::max(-m, std::min(m, v));
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

    ThrusterVehicleController::ThrusterVehicleController(const Params &p) : _p(p) {}

    Wrench ThrusterVehicleController::update(const AUVState &state, double /*dt*/)
    {
        Wrench tau;
        tau.setZero();
        if (_mode == GNCMode::DISABLED)
            return tau;

        const double mass = (_p.mass > 1e-6) ? _p.mass : 11.0;

        // Acceleration-level PD, then scale to a force/torque by the vehicle mass.
        const auto demand = [mass](const ThrusterVehicleController::Axis &g,
                                   double err, double rate) -> double
        {
            const double accel = clamp_abs(g.kp * err - g.kd * rate, g.accel_max);
            return mass * accel;
        };

        const double u = state.nu[0], v = state.nu[1], w = state.nu[2];
        const double p_rate = state.nu[3], q_rate = state.nu[4], r_rate = state.nu[5];
        const double roll = state.eta[3], pitch = state.eta[4], psi = state.eta[5];
        const double depth = state.depth_m;

        if (_mode == GNCMode::DP)
        {
            const double err_n = _sp.wp_n - state.eta[0];
            const double err_e = _sp.wp_e - state.eta[1];
            const double c = std::cos(psi);
            const double s = std::sin(psi);
            const double err_forward = c * err_n + s * err_e;
            const double err_right = -s * err_n + c * err_e;

            tau[0] = demand(_p.surge, err_forward, u);
            tau[1] = demand(_p.sway, err_right, v);
            tau[2] = demand(_p.heave, _sp.wp_d - depth, w);
        }
        else
        {
            // Surge: track reference forward speed.
            tau[0] = demand(_p.surge, _sp.surge_ref - u, u);
            // Sway: damp lateral velocity toward zero.
            tau[1] = demand(_p.sway, 0.0, v);
            // Heave: depth hold via direct vertical thrust (NED z is +down -> too
            // shallow (depth<ref) -> +err -> +Z (down) -> descend).
            if (_mode != GNCMode::SURFACE)
                tau[2] = demand(_p.heave, _sp.depth_ref - depth, w);
        }
        // Roll / pitch: passive by default (gains 0). Active only if configured.
        tau[3] = demand(_p.roll, -roll, p_rate);
        tau[4] = demand(_p.pitch, -pitch, q_rate);
        // Yaw: heading hold, or track a yaw-rate reference.
        if (_sp.use_yaw_rate_ref)
            tau[5] = mass * clamp_abs(_p.yaw.kd * (_sp.yaw_rate_ref - r_rate), _p.yaw.accel_max);
        else
            tau[5] = demand(_p.yaw, wrap_pi(_sp.heading_ref - psi), r_rate);

        return tau;
    }

} // namespace hydrox
