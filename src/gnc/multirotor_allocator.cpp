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
        const double roll = std::max(-1.0, std::min(1.0, tau[3])) * _p.roll_scale;
        const double pitch = std::max(-1.0, std::min(1.0, tau[4])) * _p.pitch_scale;
        const double yaw = std::max(-1.0, std::min(1.0, tau[5])) * _p.yaw_scale;

        ActuatorCmd cmd;
        // Canonical body-FRD channel order is FL, FR, RR, RL. Positive body
        // roll/pitch moments therefore add thrust on the left/front pair.
        // Aero body commands normalized rotor angular speed, while mixing is
        // performed in thrust space (T proportional to omega squared).
        cmd.ch[0] = static_cast<float>(std::sqrt(std::max(0.0, std::min(1.0, collective + roll + pitch + yaw))));
        cmd.ch[1] = static_cast<float>(std::sqrt(std::max(0.0, std::min(1.0, collective - roll + pitch - yaw))));
        cmd.ch[2] = static_cast<float>(std::sqrt(std::max(0.0, std::min(1.0, collective - roll - pitch + yaw))));
        cmd.ch[3] = static_cast<float>(std::sqrt(std::max(0.0, std::min(1.0, collective + roll - pitch - yaw))));
        return cmd;
    }
} // namespace hydrox
