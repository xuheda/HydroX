// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once

#include "gnc/control_interfaces.h"

namespace hydrox
{
    class SurfaceVesselController : public IController
    {
    public:
        struct Params
        {
            double surge_kp = 220.0;
            double surge_kd = 80.0;
            double yaw_kp = 180.0;
            double yaw_kd = 80.0;
            double max_force_N = 220.0;
            double max_moment_Nm = 160.0;
            double waypoint_surge_mps = 1.2;
        };

        explicit SurfaceVesselController(const Params &p = {});

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
