// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once

#include "gnc/control_interfaces.h"

namespace hydrox
{
    class MultirotorController : public IController
    {
    public:
        struct Params
        {
            double mass = 1.375;
            double g = 9.80665;
            double z_kp = 2.0;
            double z_kd = 1.6;
            double roll_kp = 4.0;
            double roll_kd = 1.6;
            double pitch_kp = 4.0;
            double pitch_kd = 1.6;
            double yaw_kp = 1.8;
            double yaw_kd = 0.8;
            double xy_kp = 0.6;
            double xy_kd = 1.4;
            double max_tilt_rad = 0.45;
            double max_xy_accel = 3.0;
            double max_z_accel = 5.0;
        };

        explicit MultirotorController(const Params &p = {});

        void reset(const NavigationState &) override {}
        void set_mode(GNCMode mode) override { _mode = mode; }
        void set_setpoint(const GNCSetpoint &sp) override { _sp = sp; }
        Wrench update(const NavigationState &state, double dt) override;

    private:
        Params _p;
        GNCMode _mode = GNCMode::DISABLED;
        GNCSetpoint _sp;
    };
} // namespace hydrox
