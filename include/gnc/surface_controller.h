// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once
#include "hydrox/default_parameter.h"

#include "gnc/control_interfaces.h"
#include "gnc/surface_guidance.h"

namespace hydrox
{
    class SurfaceVesselController : public IController
    {
    public:
        struct Params
        {
            double surge_kp = 220.0;
            // Derivative-on-measurement damps measured surge acceleration
            // without kicking when guidance changes its speed reference.
            double surge_kd = 80.0;
            double surge_accel_filter_tau_s = 0.20;
            // Integral compensation removes the steady speed deficit caused
            // by hull drag and a continuously moving waterplane.
            double surge_ki = 0.0;
            double surge_integral_limit = 0.0;
            double surge_drag_linear_N_per_mps = 0.0;
            double surge_drag_quadratic_N_per_mps2 = 0.0;
            double max_brake_force_N = 0.0;
            // Limit closed-loop inertial correction independently of the
            // plant's full bollard-pull authority. A zero value preserves
            // legacy behavior. Drag feed-forward is not included in this cap.
            double surge_feedback_force_limit_N = 0.0;
            double yaw_kp = 180.0;
            double yaw_kd = 80.0;
            double max_yaw_rate_radps = 0.45;
            double max_force_N = 220.0;
            double max_moment_Nm = 160.0;
            double waypoint_surge_mps = 1.2;
            double station_keep_surge_mps = 0.5;
            double station_keep_hysteresis_m = 1.0;
            double los_lookahead_m = 12.0;
            double ilos_integral_gain = 0.015;
            double ilos_integral_limit = 75.0;
            // Use measured sideslip to crab into a cross-flow instead of
            // repeatedly steering toward the instantaneous waypoint bearing.
            double sideslip_compensation_gain = 0.0;
            double sideslip_filter_time_constant_s = 1.0;
            double max_crab_angle_rad = 0.35;
            double max_accel_mps2 = 0.45;
            double max_decel_mps2 = 0.70;
        };

        explicit SurfaceVesselController(const Params &p = detail::default_parameter<Params>());

        void reset(const NavigationState &state) override;
        void set_mode(GNCMode mode) override
        {
            if (mode != _mode)
            {
                _surge_error_integral = 0.0;
                _surge_derivative_initialized = false;
                _filtered_surge_accel_mps2 = 0.0;
                _guidance.clear();
            }
            _mode = mode;
        }
        void set_setpoint(const GNCSetpoint &sp) override { _sp = sp; }
        Wrench update(const NavigationState &state, double dt) override;

    private:
        Params _p;
        SurfaceVesselGuidance _guidance;
        GNCMode _mode = GNCMode::DISABLED;
        GNCSetpoint _sp;
        double _surge_error_integral = 0.0;
        double _last_surge_mps = 0.0;
        double _filtered_surge_accel_mps2 = 0.0;
        bool _surge_derivative_initialized = false;
    };
} // namespace hydrox
