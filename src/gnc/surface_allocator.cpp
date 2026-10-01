// Copyright (c) 2026 OceanX. Author: xuheda
#include "gnc/surface_allocator.h"
#include "gnc/surface_control_demand.h"

#include <algorithm>
#include <cmath>

namespace hydrox
{
    TwinScrewAllocator::TwinScrewAllocator(const Params &p) : _p(p) {}

    ActuatorCmd TwinScrewAllocator::allocate(const Wrench &tau, double /*surge*/) const
    {
        const double max_t = std::max(_p.max_thrust_N, 1e-6);
        const double lever = std::max(std::abs(_p.lever_arm_m), 1e-6);

        const SurfaceControlDemand demand = SurfaceControlDemand::from_wrench(tau);
        const double X = demand.surge_force_N;
        const double N = demand.yaw_moment_Nm;

        // SurfaceVessel thruster locations are stored in UE coordinates:
        // ch[0] is port (UE Y < 0), ch[1] is starboard (UE Y > 0). A larger
        // starboard thrust produces a negative UE-Z moment, which maps to a
        // positive NED yaw moment through the project torque conversion.
        const double port = 0.5 * X - 0.5 * N / lever;
        const double starboard = 0.5 * X + 0.5 * N / lever;
        // Joint desaturation preserves the requested X:N wrench ratio. Two
        // independent clamps distort the turn whenever only one side hits a
        // limit, which is a common cause of hooked and oscillatory tracks.
        const double peak = std::max(std::abs(port), std::abs(starboard));
        const double scale = peak > max_t ? max_t / peak : 1.0;

        ActuatorCmd cmd;
        cmd.layout = ActuatorLayout::TwinScrew;
        cmd.set_thrust(0, port * scale, max_t, max_t);
        cmd.set_thrust(1, starboard * scale, max_t, max_t);
        return cmd;
    }
} // namespace hydrox
