// Copyright (c) 2026 OceanX. Author: xuheda
#include "gnc/fixedwing_allocator.h"

#include <algorithm>

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
        const double speed_ref = tau[0];
        const double roll_ref = tau[3];
        const double pitch_ref = tau[4];
        const double yaw_rate_ref = tau[5];

        ActuatorCmd cmd;
        cmd.ch[0] = static_cast<float>(clamp(_p.elevator_kp * pitch_ref, -1.0, 1.0));
        cmd.ch[1] = static_cast<float>(clamp(_p.aileron_kp * roll_ref, -1.0, 1.0));
        cmd.ch[2] = static_cast<float>(clamp(_p.rudder_kp * yaw_rate_ref, -1.0, 1.0));
        cmd.ch[3] = static_cast<float>(
            clamp(_p.throttle_trim + _p.throttle_kp * (speed_ref - surge), 0.0, 1.0));
        return cmd;
    }
} // namespace hydrox
