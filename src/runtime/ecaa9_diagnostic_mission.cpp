#include "hydrox/runtime/ecaa9_diagnostic_mission.h"

#include <algorithm>
#include <cmath>

namespace hydrox::runtime
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double wrap_pi(double angle) noexcept
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

double distance_3d(const MissionGuidanceState &state,
                   const MissionWaypoint &goal) noexcept
{
    const double north = goal.north_m - state.north_m;
    const double east = goal.east_m - state.east_m;
    const double down = goal.down_m - state.down_m;
    return std::sqrt(north * north + east * east + down * down);
}
}

const std::array<MissionWaypoint, EcaA9DiagnosticMission::kWaypointCount> &
EcaA9DiagnosticMission::waypoints() noexcept
{
    static constexpr std::array<MissionWaypoint, kWaypointCount> route{{
        {0.0, -30.0, 100.0},
        {40.0, 10.0, 105.0},
        {0.0, 50.0, 110.0},
        {-120.0, 50.0, 110.0},
        {-160.0, 10.0, 105.0},
        {-120.0, -30.0, 100.0},
    }};
    return route;
}

void EcaA9DiagnosticMission::reset() noexcept
{
    initialized_ = false;
    waypoint_index_ = 0;
    segment_start_ = {};
}

const MissionWaypoint &EcaA9DiagnosticMission::active_waypoint() const noexcept
{
    return waypoints()[waypoint_index_];
}

GNCSetpoint EcaA9DiagnosticMission::update(const MissionGuidanceState &state) noexcept
{
    if (!initialized_)
    {
        segment_start_ = {state.north_m, state.east_m, state.down_m};
        initialized_ = true;
    }

    if (distance_3d(state, active_waypoint()) <= kArrivalRadiusM)
    {
        segment_start_ = active_waypoint();
        waypoint_index_ = (waypoint_index_ + 1) % kWaypointCount;
    }

    const MissionWaypoint &goal = active_waypoint();
    const double segment_n = goal.north_m - segment_start_.north_m;
    const double segment_e = goal.east_m - segment_start_.east_m;
    const double segment_length = std::hypot(segment_n, segment_e);

    double desired_yaw = state.yaw_rad;
    double commanded_depth = goal.down_m;
    if (segment_length > 1.0e-6)
    {
        const double unit_n = segment_n / segment_length;
        const double unit_e = segment_e / segment_length;
        const double relative_n = state.north_m - segment_start_.north_m;
        const double relative_e = state.east_m - segment_start_.east_m;
        const double along_track = relative_n * unit_n + relative_e * unit_e;
        const double target_n = segment_start_.north_m +
                                (along_track + kLookaheadM) * unit_n;
        const double target_e = segment_start_.east_m +
                                (along_track + kLookaheadM) * unit_e;
        const double aim_n = target_n - state.north_m;
        const double aim_e = target_e - state.east_m;
        if (std::hypot(aim_n, aim_e) > 1.0e-6)
            desired_yaw = std::atan2(aim_e, aim_n);

        const double progress = std::clamp(
            (along_track + kLookaheadM) / segment_length, 0.0, 1.0);
        commanded_depth = segment_start_.down_m +
                            progress * (goal.down_m - segment_start_.down_m);
    }
    else
    {
        const double direct_n = goal.north_m - state.north_m;
        const double direct_e = goal.east_m - state.east_m;
        if (std::hypot(direct_n, direct_e) > 1.0e-6)
            desired_yaw = std::atan2(direct_e, direct_n);
    }

    const double course_error = wrap_pi(desired_yaw - state.yaw_rad);
    const double yaw_rate = std::clamp(0.45 * course_error, -0.35, 0.35);
    const double reduction_start = 20.0 * kPi / 180.0;
    const double full_reduction = 90.0 * kPi / 180.0;
    const double severity = std::clamp(
        (std::abs(course_error) - reduction_start) /
            (full_reduction - reduction_start),
        0.0, 1.0);
    const double commanded_speed =
        kCruiseSpeedMps * (1.0 - 0.4 * severity);

    GNCSetpoint setpoint{};
    setpoint.depth_ref = commanded_depth;
    setpoint.heading_ref = desired_yaw;
    setpoint.surge_ref = commanded_speed;
    setpoint.use_yaw_rate_ref = true;
    setpoint.yaw_rate_ref = yaw_rate;
    setpoint.wp_n = goal.north_m;
    setpoint.wp_e = goal.east_m;
    setpoint.wp_d = commanded_depth;
    setpoint.use_path_segment = true;
    setpoint.path_start_n = segment_start_.north_m;
    setpoint.path_start_e = segment_start_.east_m;
    setpoint.lookahead_m = kLookaheadM;
    setpoint.arrival_radius_m = kArrivalRadiusM;
    setpoint.hold_heading = false;
    return setpoint;
}
}
