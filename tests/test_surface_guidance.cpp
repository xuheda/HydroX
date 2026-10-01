#include "gnc/surface_guidance.h"

#include <cmath>
#include <iostream>

namespace
{
int expect(bool condition, const char *message)
{
    if (condition)
        return 0;
    std::cerr << "FAIL: " << message << '\n';
    return 1;
}
}

int main()
{
    int failures = 0;

    hydrox::SurfaceVesselGuidance::Params params;
    params.waypoint_surge_mps = 3.0;
    params.station_keep_surge_mps = 0.3;
    params.station_keep_hysteresis_m = 2.0;
    params.los_lookahead_m = 10.0;
    params.ilos_integral_gain = 0.02;
    params.sideslip_compensation_gain = 1.0;
    params.sideslip_filter_time_constant_s = 0.1;
    params.max_crab_angle_rad = 0.3;
    params.max_accel_mps2 = 1.0;
    params.max_decel_mps2 = 1.0;
    hydrox::SurfaceVesselGuidance guidance(params);

    hydrox::GNCSetpoint waypoint;
    waypoint.wp_n = 50.0;
    waypoint.wp_e = 0.0;
    waypoint.surge_ref = 0.5;
    waypoint.use_path_segment = true;
    waypoint.path_start_n = 0.0;
    waypoint.path_start_e = 0.0;
    waypoint.lookahead_m = 10.0;
    waypoint.arrival_radius_m = 2.0;

    auto state = hydrox::NavigationState::zeros();
    auto ahead = guidance.update(
        state, waypoint, hydrox::GNCMode::WAYPOINT_3D, 0.1);
    failures += expect(std::abs(ahead.heading_ref_rad) < 1.0e-12,
                       "north waypoint produces north heading");
    failures += expect(std::abs(ahead.surge_ref_mps - 0.1) < 1.0e-12,
                       "guidance applies the configured acceleration limit");
    for (int i = 0; i < 4; ++i)
        ahead = guidance.update(
            state, waypoint, hydrox::GNCMode::WAYPOINT_3D, 0.1);
    failures += expect(std::abs(ahead.surge_ref_mps - 0.5) < 1.0e-12,
                       "guidance reaches the planner speed without exceeding it");

    hydrox::SurfaceVesselGuidance cross_track_guidance(params);
    auto cross_track_state = hydrox::NavigationState::zeros();
    cross_track_state.eta[1] = 5.0;
    const auto cross_track = cross_track_guidance.update(
        cross_track_state, waypoint, hydrox::GNCMode::WAYPOINT_3D, 0.1);
    failures += expect(cross_track.cross_track_error_m > 4.9 &&
                           cross_track.course_ref_rad < -0.4,
                       "LOS steers west from the east side of a northbound segment");

    hydrox::SurfaceVesselGuidance broadside_guidance(params);
    hydrox::SurfaceVesselGuidance missed_guidance(params);
    auto missed_state = hydrox::NavigationState::zeros();
    missed_state.eta[0] = 52.0;
    missed_state.eta[1] = 3.0;
    const auto missed = missed_guidance.update(
        missed_state, waypoint, hydrox::GNCMode::WAYPOINT_3D, 0.1);
    failures += expect(std::abs(missed.course_ref_rad - std::atan2(-3.0, -2.0)) < 1.e-12,
                       "missed finite waypoint turns back, not down the leg extension");
    failures += expect(missed.surge_ref_mps == 0.0,
                       "missed waypoint recovery turns before driving away");
    auto broadside_state = hydrox::NavigationState::zeros();
    broadside_state.eta[5] = 0.5 * 3.14159265358979323846;
    const auto broadside = broadside_guidance.update(
        broadside_state, waypoint, hydrox::GNCMode::WAYPOINT_3D, 0.1);
    failures += expect(std::abs(broadside.surge_ref_mps) < 1.0e-12,
                       "guidance starts with zero surge while broadside to the path");

    hydrox::SurfaceVesselGuidance crab_guidance(params);
    state = hydrox::NavigationState::zeros();
    state.nu[0] = 0.8;
    state.nu[1] = -0.4;
    const auto crabbed = crab_guidance.update(
        state, waypoint, hydrox::GNCMode::WAYPOINT_3D, 0.1);
    failures += expect(crabbed.heading_ref_rad > 0.0,
                       "negative sway produces positive crab compensation");

    // Vertical state is intentionally outside horizontal surface guidance.
    const double heading_before_vertical_change = crabbed.heading_ref_rad;
    const double speed_before_vertical_change = crabbed.surge_ref_mps;
    state.eta[2] = 100.0;
    state.eta[3] = 0.8;
    state.eta[4] = -0.6;
    state.nu[2] = 5.0;
    const auto wave_displaced = crab_guidance.update(
        state, waypoint, hydrox::GNCMode::WAYPOINT_3D, 0.0);
    failures += expect(
        std::abs(wave_displaced.heading_ref_rad - heading_before_vertical_change) < 1.0e-12 &&
            std::abs(wave_displaced.surge_ref_mps - speed_before_vertical_change) < 1.0e-12,
        "heave roll and pitch do not enter horizontal guidance");

    hydrox::SurfaceVesselGuidance terminal_guidance(params);
    auto coasting = hydrox::NavigationState::zeros();
    coasting.eta[0] = waypoint.wp_n;
    coasting.nu[0] = 0.8;
    terminal_guidance.reset(coasting);
    waypoint.surge_ref = 0.0;
    waypoint.heading_ref = 1.1;
    waypoint.hold_heading = true;
    const auto terminal = terminal_guidance.update(
        coasting, waypoint, hydrox::GNCMode::WAYPOINT_3D, 0.1);
    failures += expect(std::abs(terminal.heading_ref_rad - 1.1) < 1.0e-12,
                       "terminal policy preserves the planner-latched heading");
    failures += expect(std::abs(terminal.surge_ref_mps - 0.7) < 1.0e-12,
                       "terminal policy decelerates instead of dropping speed discontinuously");

    auto near_boundary = hydrox::NavigationState::zeros();
    near_boundary.eta[0] = waypoint.wp_n - 3.0;
    terminal_guidance.reset(near_boundary);
    const auto quiet_band = terminal_guidance.update(
        near_boundary, waypoint, hydrox::GNCMode::WAYPOINT_3D, 0.1);
    failures += expect(
        std::abs(quiet_band.heading_ref_rad - waypoint.heading_ref) < 1.0e-12 &&
            std::abs(quiet_band.surge_ref_mps) < 1.0e-12,
        "terminal station keeping ignores drift inside the outer hysteresis boundary");

    auto drifted = hydrox::NavigationState::zeros();
    drifted.eta[0] = waypoint.wp_n - 5.0;
    terminal_guidance.reset(drifted);
    const auto reacquiring = terminal_guidance.update(
        drifted, waypoint, hydrox::GNCMode::WAYPOINT_3D, 0.1);
    failures += expect(
        std::abs(reacquiring.heading_ref_rad) < 1.0e-12 &&
            std::abs(reacquiring.surge_ref_mps - 0.1) < 1.0e-12,
        "terminal station keeping re-enters at bounded speed after drifting outside the radius");
    drifted.eta[0] = waypoint.wp_n - 3.0;
    const auto recaptured = terminal_guidance.update(
        drifted, waypoint, hydrox::GNCMode::WAYPOINT_3D, 0.1);
    failures += expect(
        std::abs(recaptured.heading_ref_rad - waypoint.heading_ref) < 1.0e-12 &&
            std::abs(recaptured.surge_ref_mps) < 1.0e-12,
        "terminal station keeping restores the latched heading after recapture");

    auto controlled_stop_params = params;
    controlled_stop_params.station_keep_surge_mps = 0.0;
    hydrox::SurfaceVesselGuidance controlled_stop(controlled_stop_params);
    drifted.eta[0] = waypoint.wp_n - 20.0;
    controlled_stop.reset(drifted);
    const auto stopped = controlled_stop.update(
        drifted, waypoint, hydrox::GNCMode::WAYPOINT_3D, 0.1);
    failures += expect(
        std::abs(stopped.heading_ref_rad - waypoint.heading_ref) < 1.0e-12 &&
            std::abs(stopped.surge_ref_mps) < 1.0e-12,
        "zero station-keep speed prevents drift-induced terminal U-turns");

    if (failures == 0)
        std::cout << "test_surface_guidance: all checks passed\n";
    return failures == 0 ? 0 : 1;
}
