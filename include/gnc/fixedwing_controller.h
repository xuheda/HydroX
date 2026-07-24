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
            double pitch_limit_rad = 0.35;
            double course_kp = 0.9;
            double roll_limit_rad = 0.55;
        };

        explicit FixedWingController(const Params &p = {});

        void reset(const AUVState &) override {}
        void set_mode(GNCMode mode) override { _mode = mode; }
        void set_setpoint(const GNCSetpoint &sp) override { _sp = sp; }
        Wrench update(const AUVState &state, double dt) override;

    private:
        Params _p;
        GNCMode _mode = GNCMode::DISABLED;
        GNCSetpoint _sp;
    };
} // namespace hydrox
