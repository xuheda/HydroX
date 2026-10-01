#pragma once

#include "hydrox/platform/clock.h"
#include "types.h"

#include <cmath>

namespace hydrox::runtime
{
    /** Defaults shared by host SITL and embedded HITL profiles. */
    constexpr double kDefaultHilControlPeriodS = 0.01;
    constexpr double kDefaultHilMaxSensorDtS = 0.25;
    constexpr platform::MonotonicTimeUs kDefaultHilSetpointTimeoutUs = 500'000;
    constexpr platform::MonotonicTimeUs kDefaultHilSensorTimeoutUs = 500'000;

    /**
     * Transport-independent GNC command bounds.
     *
     * Both DDS/SITL and companion/HITL inputs must pass these limits. The
     * shared runtime checks them again so a future transport cannot bypass the
     * command contract.
     */
    struct GncSetpointLimits
    {
        static constexpr double max_abs_depth_m = 12'000.0;
        static constexpr double max_abs_heading_rad = 1'000.0;
        static constexpr double max_abs_surge_mps = 100.0;
        static constexpr double max_abs_yaw_rate_radps = 100.0;
        static constexpr double max_abs_waypoint_horizontal_m = 10'000'000.0;
        static constexpr double max_abs_waypoint_down_m = 12'000.0;
        static constexpr double max_lookahead_m = 10'000.0;
        static constexpr double max_arrival_radius_m = 10'000.0;
    };

    inline bool valid_gnc_mode(GNCMode mode) noexcept
    {
        switch (mode)
        {
        case GNCMode::DISABLED:
        case GNCMode::DEPTH_HOLD:
        case GNCMode::WAYPOINT_3D:
        case GNCMode::DP:
        case GNCMode::SURFACE:
            return true;
        }
        return false;
    }

    inline bool valid_gnc_setpoint(
        const GNCSetpoint &setpoint,
        GNCMode mode) noexcept
    {
        const auto bounded = [](double value, double maximum) noexcept
        {
            return std::isfinite(value) && std::abs(value) <= maximum;
        };

        return valid_gnc_mode(mode) &&
               bounded(setpoint.depth_ref,
                       GncSetpointLimits::max_abs_depth_m) &&
               bounded(setpoint.heading_ref,
                       GncSetpointLimits::max_abs_heading_rad) &&
               bounded(setpoint.surge_ref,
                       GncSetpointLimits::max_abs_surge_mps) &&
               bounded(setpoint.yaw_rate_ref,
                       GncSetpointLimits::max_abs_yaw_rate_radps) &&
               bounded(setpoint.wp_n,
                       GncSetpointLimits::max_abs_waypoint_horizontal_m) &&
               bounded(setpoint.wp_e,
                       GncSetpointLimits::max_abs_waypoint_horizontal_m) &&
               bounded(setpoint.wp_d,
                       GncSetpointLimits::max_abs_waypoint_down_m) &&
               bounded(setpoint.path_start_n,
                       GncSetpointLimits::max_abs_waypoint_horizontal_m) &&
               bounded(setpoint.path_start_e,
                       GncSetpointLimits::max_abs_waypoint_horizontal_m) &&
               std::isfinite(setpoint.lookahead_m) &&
               setpoint.lookahead_m >= 0.0 &&
               setpoint.lookahead_m <= GncSetpointLimits::max_lookahead_m &&
               std::isfinite(setpoint.arrival_radius_m) &&
               setpoint.arrival_radius_m >= 0.0 &&
               setpoint.arrival_radius_m <=
                   GncSetpointLimits::max_arrival_radius_m;
    }
} // namespace hydrox::runtime
