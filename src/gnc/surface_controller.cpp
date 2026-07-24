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

    Wrench SurfaceVesselController::update(const AUVState &state, double /*dt*/)
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
            surge_ref = std::min(_p.waypoint_surge_mps, std::max(0.0, dist));
        }

        const double u = state.nu[0];
        const double r = state.nu[5];
        const double yaw_err = wrap_pi(heading_ref - state.eta[5]);

        tau[0] = clamp_abs(_p.surge_kp * (surge_ref - u) - _p.surge_kd * u,
                           _p.max_force_N);
        if (_sp.use_yaw_rate_ref)
            tau[5] = clamp_abs(_p.yaw_kd * (_sp.yaw_rate_ref - r),
                               _p.max_moment_Nm);
        else
            tau[5] = clamp_abs(_p.yaw_kp * yaw_err - _p.yaw_kd * r,
                               _p.max_moment_Nm);

        return tau;
    }
} // namespace hydrox
