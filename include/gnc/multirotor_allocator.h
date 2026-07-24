// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once

#include "gnc/control_interfaces.h"

namespace hydrox
{
    class QuadrotorAllocator : public IAllocator
    {
    public:
        struct Params
        {
            double hover_throttle = 0.55;
            double max_total_thrust_N = 26.0;
            double roll_scale = 0.12;
            double pitch_scale = 0.12;
            double yaw_scale = 0.08;
        };

        explicit QuadrotorAllocator(const Params &p = {});
        ActuatorCmd allocate(const Wrench &tau, double surge) const override;

    private:
        Params _p;
    };
} // namespace hydrox
