// Copyright (c) 2026 OceanX. Author: xuheda
#include "gnc/fixedwing_allocator.h"

#include <algorithm>
#include <cmath>

namespace hydrox
{
    namespace
    {
        double clamp(double v, double lo, double hi)
        {
            return std::max(lo, std::min(hi, v));
        }
    } // namespace

    FixedWingAllocator::FixedWingAllocator(const Params &p) : _p(p) {}

    ActuatorCmd FixedWingAllocator::allocate(const Wrench &tau, double surge) const
    {
        const double reference_speed = std::max(
            std::abs(_p.reference_airspeed_mps), 1.0e-3);
        const double effective_speed = std::max(
            std::abs(surge), std::max(std::abs(_p.minimum_control_airspeed_mps), 1.0e-3));
        const double dynamic_pressure_scale =
            (reference_speed * reference_speed) /
            (effective_speed * effective_speed);

        const double elevator = clamp(
            _p.elevator_per_moment_inv_Nm * tau[4] * dynamic_pressure_scale,
            -1.0, 1.0);
        const double aileron = clamp(
            _p.aileron_per_moment_inv_Nm * tau[3] * dynamic_pressure_scale,
            -1.0, 1.0);
        const double rudder = clamp(
            _p.rudder_per_moment_inv_Nm * tau[5] * dynamic_pressure_scale,
            -1.0, 1.0);

        ActuatorCmd cmd;
        cmd.layout = ActuatorLayout::ConventionalFixedWing;
        cmd.set_position(0, elevator * _p.elevator_limit_rad,
                         _p.elevator_limit_rad, _p.elevator_limit_rad);
        cmd.set_position(1, aileron * _p.aileron_limit_rad,
                         _p.aileron_limit_rad, _p.aileron_limit_rad);
        cmd.set_position(2, rudder * _p.rudder_limit_rad,
                         _p.rudder_limit_rad, _p.rudder_limit_rad);
        cmd.set_thrust(3, tau[0], _p.max_forward_thrust_N);
        return cmd;
    }
} // namespace hydrox
