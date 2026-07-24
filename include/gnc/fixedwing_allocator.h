// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once

#include "gnc/control_interfaces.h"

namespace hydrox
{
    class FixedWingAllocator : public IAllocator
    {
    public:
        struct Params
        {
            double elevator_kp = -2.0;
            double elevator_kd = -0.35;
            double aileron_kp = 1.8;
            double aileron_kd = 0.25;
            double rudder_kp = 0.35;
            double throttle_trim = 0.55;
            double throttle_kp = 0.04;
        };

        explicit FixedWingAllocator(const Params &p = {});
        ActuatorCmd allocate(const Wrench &tau, double surge) const override;

    private:
        Params _p;
    };
} // namespace hydrox
