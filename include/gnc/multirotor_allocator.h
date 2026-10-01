// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once
#include "hydrox/default_parameter.h"

#include "gnc/control_interfaces.h"

namespace hydrox
{
    class QuadrotorAllocator : public IAllocator
    {
    public:
        struct Params
        {
            double max_total_thrust_N = 26.0;
            // X-configuration effective moment arm (arm_len / sqrt(2)) and
            // rotor reaction-torque/thrust ratio (kQ / kT).
            double roll_pitch_moment_arm_m = 0.175;
            double yaw_moment_per_thrust_m = 0.02;
        };

        explicit QuadrotorAllocator(const Params &p = detail::default_parameter<Params>());
        ActuatorCmd allocate(const Wrench &tau, double surge) const override;

    private:
        Params _p;
    };
} // namespace hydrox
