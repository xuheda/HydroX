// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once
#include "hydrox/default_parameter.h"

#include "gnc/control_interfaces.h"

namespace hydrox
{
    /** Horizontal command consumed by the underactuated USV controller. */
    struct SurfaceGuidanceCommand
    {
        double heading_ref_rad = 0.0;
        double surge_ref_mps = 0.0;
        double course_ref_rad = 0.0;
        double cross_track_error_m = 0.0;
        double unshaped_surge_ref_mps = 0.0;
        double waypoint_distance_m = 0.0;
    };

    /**
     * Converts a horizontal navigation objective into heading and surge
     * references. It never generates heave, roll or pitch objectives.
     */
    class SurfaceVesselGuidance
    {
    public:
        struct Params
        {
            double waypoint_surge_mps = 1.2;
            double station_keep_surge_mps = 0.5;
            double station_keep_hysteresis_m = 1.0;
            double los_lookahead_m = 12.0;
            double ilos_integral_gain = 0.015;
            double ilos_integral_limit = 75.0;
            double sideslip_compensation_gain = 0.0;
            double sideslip_filter_time_constant_s = 1.0;
            double max_crab_angle_rad = 0.35;
            double max_accel_mps2 = 0.45;
            double max_decel_mps2 = 0.70;
        };

        explicit SurfaceVesselGuidance(const Params &p = detail::default_parameter<Params>());

        void reset(const NavigationState &state);
        void clear();
        SurfaceGuidanceCommand update(
            const NavigationState &state,
            const GNCSetpoint &setpoint,
            GNCMode mode,
            double dt);

    private:
        Params _p;
        bool _initialized = false;
        bool _station_reacquiring = false;
        bool _segment_active = false;
        double _filtered_sideslip_rad = 0.0;
        double _cross_track_integral = 0.0;
        double _last_surge_ref_mps = 0.0;
        double _segment_start_n = 0.0;
        double _segment_start_e = 0.0;
        double _segment_goal_n = 0.0;
        double _segment_goal_e = 0.0;
    };
} // namespace hydrox
