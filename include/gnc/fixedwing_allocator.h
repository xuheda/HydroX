// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once
#include "hydrox/default_parameter.h"

#include "gnc/control_interfaces.h"

namespace hydrox
{
    class FixedWingAllocator : public IAllocator
    {
    public:
        struct Params
        {
            // Normalized surface position per requested physical body moment
            // at reference_airspeed_mps. Signs match the imported joints.
            double elevator_per_moment_inv_Nm = 2.0 / 0.149361;
            double aileron_per_moment_inv_Nm = -1.8 / 0.201497;
            double rudder_per_moment_inv_Nm = 0.35 / 0.150727;
            double reference_airspeed_mps = 12.0;
            double minimum_control_airspeed_mps = 6.0;
            double elevator_limit_rad = 0.78;
            double aileron_limit_rad = 0.78;
            double rudder_limit_rad = 0.78;
            double max_forward_thrust_N = 12.0;
        };

        explicit FixedWingAllocator(const Params &p = detail::default_parameter<Params>());
        ActuatorCmd allocate(const Wrench &tau, double surge) const override;

    private:
        Params _p;
    };
} // namespace hydrox
