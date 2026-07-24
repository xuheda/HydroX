// Copyright (c) 2026 OceanX
#include "gnc/vtol_allocator.h"

#include <algorithm>
#include <cmath>

namespace hydrox
{
namespace
{
double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
float rotor_command(double normalized_thrust)
{
    return static_cast<float>(std::sqrt(clamp(normalized_thrust, 0.0, 1.0)));
}
}

VtolAllocator::VtolAllocator(const Params& p) : _p(p) {}

ActuatorCmd VtolAllocator::allocate(const Wrench& tau, double surge) const
{
    const double collective = clamp(tau[2] / std::max(_p.max_total_lift_N, 1.0e-6), 0.0, 1.0);
    const double roll = clamp(tau[3], -1.0, 1.0) * _p.roll_scale;
    const double pitch = clamp(tau[4], -1.0, 1.0) * _p.pitch_scale;
    const double yaw = clamp(tau[5], -1.0, 1.0) * _p.yaw_scale;
    ActuatorCmd cmd;
    cmd.ch[0] = rotor_command(collective + roll + pitch + yaw);
    cmd.ch[1] = rotor_command(collective - roll + pitch - yaw);
    cmd.ch[2] = rotor_command(collective - roll - pitch + yaw);
    cmd.ch[3] = rotor_command(collective + roll - pitch - yaw);
    cmd.ch[4] = static_cast<float>(clamp(_p.surface_gain * tau[4], -1.0, 1.0));
    cmd.ch[5] = static_cast<float>(clamp(_p.surface_gain * tau[3], -1.0, 1.0));
    cmd.ch[6] = static_cast<float>(clamp(_p.surface_gain * tau[5], -1.0, 1.0));
    cmd.ch[7] = static_cast<float>(clamp(_p.pusher_trim + _p.pusher_kp * (tau[0] - surge), 0.0, 1.0));
    return cmd;
}
}
