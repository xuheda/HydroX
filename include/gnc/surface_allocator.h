// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once

#include "gnc/control_interfaces.h"

namespace hydrox
{
    class TwinScrewAllocator : public IAllocator
    {
    public:
        struct Params
        {
            double max_thrust_N = 119.7;
            double lever_arm_m = 0.395;
        };

        explicit TwinScrewAllocator(const Params &p = {});
        ActuatorCmd allocate(const Wrench &tau, double surge) const override;

    private:
        Params _p;
    };
} // namespace hydrox
