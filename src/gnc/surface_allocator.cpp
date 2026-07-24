// Copyright (c) 2026 OceanX. Author: xuheda
#include "gnc/surface_allocator.h"

#include <algorithm>
#include <cmath>

namespace hydrox
{
    TwinScrewAllocator::TwinScrewAllocator(const Params &p) : _p(p) {}

    ActuatorCmd TwinScrewAllocator::allocate(const Wrench &tau, double /*surge*/) const
    {
        const double max_t = std::max(_p.max_thrust_N, 1e-6);
        const double lever = std::max(std::abs(_p.lever_arm_m), 1e-6);

        const double X = tau[0];
        const double N = tau[5];

        // SurfaceVessel thruster locations are stored in UE coordinates:
        // ch[0] is port (UE Y < 0), ch[1] is starboard (UE Y > 0). A larger
        // starboard thrust produces a negative UE-Z moment, which maps to a
        // positive NED yaw moment through the project torque conversion.
        const double port = 0.5 * X - 0.5 * N / lever;
        const double starboard = 0.5 * X + 0.5 * N / lever;

        ActuatorCmd cmd;
        cmd.ch[0] = static_cast<float>(std::max(-1.0, std::min(1.0, port / max_t)));
        cmd.ch[1] = static_cast<float>(std::max(-1.0, std::min(1.0, starboard / max_t)));
        return cmd;
    }
} // namespace hydrox
