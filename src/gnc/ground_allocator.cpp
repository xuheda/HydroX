// Copyright (c) 2026 OceanX. Author: xuheda
#include "gnc/ground_allocator.h"

#include <algorithm>
#include <cmath>

namespace hydrox
{
DifferentialDriveAllocator::DifferentialDriveAllocator(const Params &p) : _p(p) {}

ActuatorCmd DifferentialDriveAllocator::allocate(const Wrench &tau, double surge) const
{
    const double track = std::max(std::abs(_p.track_width_m), 1.0e-6);
    const double radius = std::max(std::abs(_p.wheel_radius_m), 1.0e-6);
    const double max_omega = std::max(std::abs(_p.max_wheel_angular_speed_radps), 1.0e-6);
    const double gain = std::max(std::abs(_p.longitudinal_speed_gain_N_per_mps), 1.0e-6);

    // FRD/NED positive yaw is produced by more left-side than right-side force.
    const double right_force = 0.5 * tau[0] - tau[5] / track;
    const double left_force = 0.5 * tau[0] + tau[5] / track;
    const double right_omega = (surge + right_force / gain) / radius;
    const double left_omega = (surge + left_force / gain) / radius;

    ActuatorCmd cmd;
    cmd.ch[0] = static_cast<float>(std::clamp(right_omega / max_omega, -1.0, 1.0));
    cmd.ch[1] = static_cast<float>(std::clamp(left_omega / max_omega, -1.0, 1.0));
    return cmd;
}
} // namespace hydrox
