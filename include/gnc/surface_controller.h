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
            // Integral compensation removes the steady speed deficit caused
            // by hull drag and a continuously moving waterplane.
            double surge_ki = 0.0;
            double surge_integral_limit = 0.0;
            double yaw_kp = 180.0;
            double yaw_kd = 80.0;
            double max_force_N = 220.0;
            double max_moment_Nm = 160.0;
            double waypoint_surge_mps = 1.2;
            // Use measured sideslip to crab into a cross-flow instead of
            // repeatedly steering toward the instantaneous waypoint bearing.
            double sideslip_compensation_gain = 0.0;
            double max_crab_angle_rad = 0.35;
        };

        explicit SurfaceVesselController(const Params &p = {});

        void reset(const NavigationState &) override { _surge_error_integral = 0.0; }
        void set_mode(GNCMode mode) override
        {
            if (mode != _mode)
                _surge_error_integral = 0.0;
            _mode = mode;
        }
        void set_setpoint(const GNCSetpoint &sp) override { _sp = sp; }
        Wrench update(const NavigationState &state, double dt) override;

    private:
        Params _p;
        GNCMode _mode = GNCMode::DISABLED;
        GNCSetpoint _sp;
        double _surge_error_integral = 0.0;
    };
} // namespace hydrox
