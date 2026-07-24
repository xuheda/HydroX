// Copyright (c) 2026 OceanX
#pragma once

#include "gnc/control_interfaces.h"

namespace hydrox
{
    /**
     * Channels: 0..3 lift rotors, 4 elevator, 5 aileron, 6 rudder, 7 pusher.
     */
    class VtolAllocator : public IAllocator
    {
    public:
        struct Params
        {
            double max_total_lift_N = 180.0;
            double roll_scale = 0.08;
            double pitch_scale = 0.08;
            double yaw_scale = 0.05;
            double surface_gain = 1.0;
            double pusher_trim = 0.15;
            double pusher_kp = 0.08;
        };

        explicit VtolAllocator(const Params& p = {});
        ActuatorCmd allocate(const Wrench& tau, double surge) const override;

    private:
        Params _p;
    };
}
