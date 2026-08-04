// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once

#include "gnc/control_interfaces.h"

namespace hydrox
{
    /**
     * Converts body force/yaw moment into normalized wheel speeds.
     * Channel contract: ch[0] right side, ch[1] left side.
     */
    class DifferentialDriveAllocator final : public IAllocator
    {
    public:
        struct Params
        {
            double wheel_radius_m = 0.0686;
            double track_width_m = 0.32634;
            double max_wheel_angular_speed_radps = 40.0;
            double longitudinal_speed_gain_N_per_mps = 180.0;
        };

        explicit DifferentialDriveAllocator(const Params &p = {});
        ActuatorCmd allocate(const Wrench &tau, double surge) const override;

    private:
        Params _p;
    };
} // namespace hydrox
