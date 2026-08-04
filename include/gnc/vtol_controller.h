// Copyright (c) 2026 OceanX
#pragma once

#include "gnc/control_interfaces.h"

namespace hydrox
{
    /** Position/attitude controller for lift-plus-cruise VTOL aircraft. */
    class VtolController : public IController
    {
    public:
        struct Params
        {
            double mass = 5.0;
            double g = 9.80665;
            double z_kp = 1.8;
            double z_kd = 1.5;
            double roll_kp = 3.2;
            double roll_kd = 1.3;
            double pitch_kp = 3.2;
            double pitch_kd = 1.3;
            double yaw_kp = 1.5;
            double yaw_kd = 0.7;
            double xy_kp = 0.45;
            double xy_kd = 1.1;
            double max_tilt_rad = 0.35;
            double cruise_speed_mps = 12.0;
        };

        explicit VtolController(const Params& p = {});
        void reset(const NavigationState& state) override
        {
            _heading_hold = state.eta[5];
            _heading_hold_valid = true;
        }
        void set_mode(GNCMode mode) override { _mode = mode; }
        void set_setpoint(const GNCSetpoint& sp) override { _sp = sp; }
        Wrench update(const NavigationState& state, double dt) override;

    private:
        Params _p;
        GNCMode _mode = GNCMode::DISABLED;
        GNCSetpoint _sp;
        double _heading_hold = 0.0;
        bool _heading_hold_valid = false;
    };
}
