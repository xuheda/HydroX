// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once

#include "gnc/control_interfaces.h"

namespace hydrox
{
    class FixedWingController : public IController
    {
    public:
        struct Params
        {
            double cruise_speed_mps = 14.0;
            double altitude_kp = 0.08;
            double altitude_kd = 0.05;
            // A small, bounded integrator removes the altitude offset created
            // by the airframe's fixed elevator trim and lift-model mismatch.
            double altitude_ki = 0.0;
            double altitude_integral_limit = 10.0;
            double pitch_limit_rad = 0.35;
            double course_kp = 0.9;
            double roll_limit_rad = 0.55;

            // Inner attitude loops convert the outer-loop references into
            // bounded actuator demands.  Without these rate-damped loops a
            // waypoint reference is effectively held as a full control
            // surface deflection and the small fixed wing departs rapidly.
            double roll_attitude_kp = 0.65;
            double roll_rate_kd = 0.28;
            double max_roll_command = 0.16;
            double pitch_attitude_kp = 0.55;
            double pitch_rate_kd = 0.22;
            // rc_cessna_params: at 8 m/s, CL_0 alone is below the 1.67 kg
            // weight.  This trim maps through the -2 allocator gain to about
            // -0.25 rad elevator, supplying the missing lift before altitude
            // feedback is required.
            double pitch_trim = 0.285;
            double max_pitch_command = 0.35;
        };

        explicit FixedWingController(const Params &p = {});

        void reset(const NavigationState &) override { _altitude_error_integral = 0.0; }
        void set_mode(GNCMode mode) override
        {
            if (mode != _mode)
                _altitude_error_integral = 0.0;
            _mode = mode;
        }
        void set_setpoint(const GNCSetpoint &sp) override { _sp = sp; }
        Wrench update(const NavigationState &state, double dt) override;

    private:
        Params _p;
        GNCMode _mode = GNCMode::DISABLED;
        GNCSetpoint _sp;
        double _altitude_error_integral = 0.0;
    };
} // namespace hydrox
