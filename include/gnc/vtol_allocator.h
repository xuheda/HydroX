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
            // StandardVTOL: arm = 0.495 / sqrt(2) m and total lift = 180 N.
            // The thrust-space differential needed for one N*m is therefore
            // 1 / (arm * total_lift) ~= 0.016.  The former 0.08 values made
            // short waypoint commands apply about five times the requested
            // roll/pitch moment and destabilised hover flight.
            double roll_scale = 0.016;
            double pitch_scale = 0.016;
            double yaw_scale = 0.05;
            double surface_gain = 1.0;
            // A fixed pusher trim can force a short hover translation through
            // the airspeed transition before the lift-rotor loop has settled.
            // Keep the pusher off at zero speed error; it is driven by pusher_kp.
            double pusher_trim = 0.0;
            double pusher_kp = 0.08;
        };

        explicit VtolAllocator(const Params& p = {});
        ActuatorCmd allocate(const Wrench& tau, double surge) const override;

    private:
        Params _p;
    };
}
