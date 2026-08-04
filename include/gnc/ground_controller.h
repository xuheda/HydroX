// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once

#include "gnc/control_interfaces.h"

namespace hydrox
{
    /** Planar speed/heading controller for skid-steer and differential-drive UGVs. */
    class DifferentialDriveController final : public IController
    {
    public:
        struct Params
        {
            double surge_kp = 180.0;
            double surge_ki = 30.0;
            double surge_integral_limit = 1.0;
            double yaw_heading_kp = 2.5;
            double yaw_rate_kp = 12.0;
            double max_force_N = 220.0;
            double max_moment_Nm = 24.0;
            double waypoint_surge_mps = 1.5;
            double waypoint_stop_radius_m = 0.20;
            double waypoint_slowdown_m = 1.5;
            double max_yaw_rate_radps = 2.0;
        };

        explicit DifferentialDriveController(const Params &p = {});
        void reset(const NavigationState &) override;
        void set_mode(GNCMode mode) override;
        void set_setpoint(const GNCSetpoint &sp) override { _sp = sp; }
        Wrench update(const NavigationState &state, double dt) override;

    private:
        Params _p;
        GNCMode _mode = GNCMode::DISABLED;
        GNCSetpoint _sp;
        double _surge_integral = 0.0;
    };
} // namespace hydrox
