// Copyright (c) 2026 OceanX
#include "gnc/vtol_allocator.h"

#include <algorithm>
#include <cmath>

namespace hydrox
{
namespace
{
double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
}

VtolAllocator::VtolAllocator(const Params& p) : _p(p) {}

ActuatorCmd VtolAllocator::allocate(const Wrench& tau, double surge) const
{
    (void)surge;
    const double max_total = std::max(_p.max_total_lift_N, 1.0e-6);
    const double max_per_rotor = max_total * 0.25;
    const double collective = clamp(tau[2] / max_total, 0.0, 1.0);
    const double roll_authority = max_total *
        std::max(std::abs(_p.roll_pitch_moment_arm_m), 1.0e-6);
    const double yaw_authority = max_total *
        std::max(std::abs(_p.yaw_moment_per_thrust_m), 1.0e-6);
    const double roll = clamp(tau[3] / roll_authority, -1.0, 1.0);
    const double pitch = clamp(tau[4] / roll_authority, -1.0, 1.0);
    const double yaw = clamp(tau[5] / yaw_authority, -1.0, 1.0);
    ActuatorCmd cmd;
    cmd.layout = ActuatorLayout::LiftCruiseVtol;
    cmd.set_thrust(0, max_per_rotor * (collective + roll + pitch + yaw), max_per_rotor);
    cmd.set_thrust(1, max_per_rotor * (collective - roll + pitch - yaw), max_per_rotor);
    cmd.set_thrust(2, max_per_rotor * (collective - roll - pitch + yaw), max_per_rotor);
    cmd.set_thrust(3, max_per_rotor * (collective + roll - pitch - yaw), max_per_rotor);
    // StandardVTOL uses imported LiftDrag surfaces instead of the compact
    // derivative table above it in the JSON file.  With those joint mixes a
    // positive elevator channel creates positive body-FRD pitch, while a
    // positive aileron channel creates negative body-FRD roll.  Keep these
    // signs locked to the engine's physical surface tests.
    const double surface_limit = std::max(std::abs(_p.surface_limit_rad), 1.0e-6);
    cmd.set_position(4, clamp(_p.elevator_gain_inv_Nm * tau[4], -1.0, 1.0) * surface_limit,
                     surface_limit, surface_limit);
    cmd.set_position(5, clamp(-_p.aileron_gain_inv_Nm * tau[3], -1.0, 1.0) * surface_limit,
                     surface_limit, surface_limit);
    cmd.set_position(6, clamp(-_p.rudder_gain_inv_Nm * tau[5], -1.0, 1.0) * surface_limit,
                     surface_limit, surface_limit);
    cmd.set_thrust(7, tau[0], _p.max_pusher_thrust_N);
    return cmd;
}
}
