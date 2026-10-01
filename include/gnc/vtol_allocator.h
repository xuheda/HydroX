// Copyright (c) 2026 OceanX
#pragma once
#include "hydrox/default_parameter.h"

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
            double roll_pitch_moment_arm_m = 0.4949747468 * 0.7071067811865476;
            double yaw_moment_per_thrust_m = 1.2e-6 / 2.0e-5;
            // Rotor moments own low-speed attitude control. Aerodynamic
            // surfaces are authority-limited so they cannot apply a full
            // deflection as dynamic pressure rises through transition.
            // Each axis has explicit authority; zero disables that surface.
            double elevator_gain_inv_Nm = 0.18;
            double aileron_gain_inv_Nm = 0.18;
            double rudder_gain_inv_Nm = 0.18;
            double surface_limit_rad = 0.78;
            double max_pusher_thrust_N = 104.72;
        };

        explicit VtolAllocator(const Params& p = detail::default_parameter<Params>());
        ActuatorCmd allocate(const Wrench& tau, double surge) const override;

    private:
        Params _p;
    };
}
