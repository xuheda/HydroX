#pragma once

#include "types.h"

#include <array>
#include <cstddef>

namespace hydrox::runtime
{
    struct MissionGuidanceState
    {
        double north_m = 0.0;
        double east_m = 0.0;
        double down_m = 0.0;
        double yaw_rad = 0.0;
    };

    struct MissionWaypoint
    {
        double north_m = 0.0;
        double east_m = 0.0;
        double down_m = 0.0;
    };

    // Host-side HIL counterpart of the accepted slender_auv SITL mission.
    // It produces the same dynamic setpoints while the board retains all
    // estimator, controller, allocator, and actuator authority.
    class EcaA9DiagnosticMission
    {
    public:
        static constexpr double kCruiseSpeedMps = 2.0;
        static constexpr double kArrivalRadiusM = 10.0;
        static constexpr double kLookaheadM = 15.0;
        static constexpr std::size_t kWaypointCount = 6;

        void reset() noexcept;
        GNCSetpoint update(const MissionGuidanceState &state) noexcept;

        std::size_t waypoint_index() const noexcept { return waypoint_index_; }
        const MissionWaypoint &active_waypoint() const noexcept;

        static const std::array<MissionWaypoint, kWaypointCount> &waypoints()
            noexcept;

    private:
        bool initialized_ = false;
        std::size_t waypoint_index_ = 0;
        MissionWaypoint segment_start_{};
    };
}
