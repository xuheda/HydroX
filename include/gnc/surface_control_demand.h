// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once

#include "gnc/control_interfaces.h"

namespace hydrox
{
    /** Achievable generalized demand for the twin-screw surface layout. */
    struct SurfaceControlDemand
    {
        double surge_force_N = 0.0;
        double yaw_moment_Nm = 0.0;

        Wrench to_wrench() const
        {
            Wrench wrench = Wrench::Zero();
            wrench[0] = surge_force_N;
            wrench[5] = yaw_moment_Nm;
            return wrench;
        }

        static SurfaceControlDemand from_wrench(const Wrench &wrench)
        {
            return {wrench[0], wrench[5]};
        }
    };
} // namespace hydrox
