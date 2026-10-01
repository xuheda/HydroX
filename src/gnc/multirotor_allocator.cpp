// Copyright (c) 2026 OceanX. Author: xuheda
#include "gnc/multirotor_allocator.h"

#include <algorithm>
#include <cmath>

namespace hydrox
{
    QuadrotorAllocator::QuadrotorAllocator(const Params &p) : _p(p) {}

    ActuatorCmd QuadrotorAllocator::allocate(const Wrench &tau, double /*surge*/) const
    {
        const double max_total = std::max(_p.max_total_thrust_N, 1e-6);
        const double collective =
            std::max(0.0, std::min(1.0, tau[2] / max_total));
        const double roll_pitch_authority =
            max_total * std::max(_p.roll_pitch_moment_arm_m, 1.0e-6);
        const double yaw_authority =
            max_total * std::max(_p.yaw_moment_per_thrust_m, 1.0e-6);
        const double roll = std::max(
            -1.0, std::min(1.0, tau[3] / roll_pitch_authority));
        const double pitch = std::max(
            -1.0, std::min(1.0, tau[4] / roll_pitch_authority));
        const double yaw = std::max(
            -1.0, std::min(1.0, tau[5] / yaw_authority));

        ActuatorCmd cmd;
        cmd.layout = ActuatorLayout::QuadX;
        const double max_per_rotor_N = max_total * 0.25;
        // Canonical body-FRD channel order is FL, FR, RR, RL. Positive body
        // roll/pitch moments therefore add thrust on the left/front pair.
        // The wire contract is normalized thrust. Rotor-speed inversion belongs
        // to the actuator model, not to control allocation.
        cmd.set_thrust(0, max_per_rotor_N * (collective + roll + pitch + yaw), max_per_rotor_N);
        cmd.set_thrust(1, max_per_rotor_N * (collective - roll + pitch - yaw), max_per_rotor_N);
        cmd.set_thrust(2, max_per_rotor_N * (collective - roll - pitch + yaw), max_per_rotor_N);
        cmd.set_thrust(3, max_per_rotor_N * (collective + roll - pitch - yaw), max_per_rotor_N);
        return cmd;
    }
} // namespace hydrox
