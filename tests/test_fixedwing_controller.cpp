#include "gnc/fixedwing_controller.h"
#include "gnc/fixedwing_allocator.h"

#include <iostream>

namespace
{
int expect(bool condition, const char* message)
{
    if (condition)
        return 0;
    std::cerr << "FAIL: " << message << '\n';
    return 1;
}
}

int main()
{
    hydrox::FixedWingController::Params params;
    params.pitch_trim_moment_Nm = 0.0;
    hydrox::FixedWingController controller(params);
    hydrox::GNCSetpoint waypoint;
    waypoint.wp_n = 100.0;
    waypoint.wp_d = -45.0;
    waypoint.surge_ref = 8.0;
    controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);
    controller.set_setpoint(waypoint);

    auto state = hydrox::NavigationState::zeros();
    state.airspeed_valid = true;
    state.eta[2] = -45.0;
    state.eta[3] = 0.2;
    state.nu[3] = 0.4;
    state.eta[4] = 0.1;
    state.nu[4] = 0.3;
    const auto tau = controller.update(state, 0.01);

    int failures = 0;
    failures += expect(tau[3] < 0.0,
                       "fixed wing roll loop counters positive roll and roll rate");
    failures += expect(tau[4] < 0.0,
                       "fixed wing pitch loop counters positive pitch and pitch rate");
    failures += expect(
        tau[0] > params.cruise_force_N && tau[0] <= params.max_forward_force_N,
        "fixed wing speed loop raises physical forward force for an underspeed request");

    // A fixed wing cannot stop on a waypoint.  Once the planner marks the
    // terminal hold, retain the latched arrival heading instead of turning
    // back towards a waypoint that is now behind or beside the aircraft.
    auto terminal_state = hydrox::NavigationState::zeros();
    terminal_state.eta[2] = -45.0;
    auto terminal_waypoint = waypoint;
    terminal_waypoint.wp_n = 0.0;
    terminal_waypoint.wp_e = 100.0;
    terminal_state.nu[0] = params.cruise_speed_mps;
    terminal_waypoint.heading_ref = 0.0;
    terminal_waypoint.surge_ref = 0.0;
    terminal_waypoint.hold_heading = true;
    controller.reset(terminal_state);
    controller.set_setpoint(terminal_waypoint);
    const auto terminal_tau = controller.update(terminal_state, 0.01);
    failures += expect(
        std::abs(terminal_tau[3]) < 1.0e-12,
        "terminal hold retains the arrival heading instead of returning to the waypoint");
    failures += expect(
        std::abs(terminal_tau[0] - params.cruise_force_N) < 1.0e-12,
        "terminal hold preserves physical cruise thrust at safe airspeed");

    auto path_state = hydrox::NavigationState::zeros();
    path_state.eta[0] = 50.0;
    path_state.eta[1] = 5.0;
    path_state.eta[2] = -45.0;
    auto path_waypoint = waypoint;
    path_waypoint.use_path_segment = true;
    path_waypoint.path_start_n = 0.0;
    path_waypoint.path_start_e = 0.0;
    path_waypoint.wp_n = 100.0;
    path_waypoint.wp_e = 0.0;
    path_waypoint.lookahead_m = 10.0;
    controller.reset(path_state);
    controller.set_setpoint(path_waypoint);
    const auto path_tau = controller.update(path_state, 0.01);
    failures += expect(
        path_tau[3] < 0.0,
        "fixed-wing LOS guidance steers west from the east side of a northbound path");

    // L1 uses ground course, then feeds a coordinated yaw rate with the same
    // turn sign instead of skidding through aileron-only turns.
    auto east_waypoint = waypoint;
    east_waypoint.wp_n = 0.0;
    east_waypoint.wp_e = 100.0;
    auto northbound = hydrox::NavigationState::zeros();
    northbound.nu[0] = 12.0;
    controller.reset(northbound);
    controller.set_setpoint(east_waypoint);
    const auto right_turn_tau = controller.update(northbound, 1.0);
    failures += expect(
        right_turn_tau[3] > 0.0 && right_turn_tau[5] > 0.0,
        "fixed-wing L1 right turn produces matching bank and coordinated yaw rate");

    // A pitched-up aircraft moving along its body X axis is climbing in NED
    // even when body w is zero. Altitude damping must use NED vertical speed.
    hydrox::FixedWingController::Params ned_vertical_params;
    ned_vertical_params.altitude_kp = 0.0;
    ned_vertical_params.altitude_ki = 0.0;
    ned_vertical_params.altitude_kd = 1.0;
    ned_vertical_params.pitch_trim_moment_Nm = 0.0;
    ned_vertical_params.pitch_attitude_kp = 1.0;
    ned_vertical_params.max_pitch_reference_rate_radps = 10.0;
    hydrox::FixedWingController ned_vertical_controller(ned_vertical_params);
    ned_vertical_controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);
    ned_vertical_controller.set_setpoint(waypoint);
    auto climbing = hydrox::NavigationState::zeros();
    climbing.eta[2] = -45.0;
    climbing.eta[4] = 0.2;
    climbing.nu[0] = 10.0;
    ned_vertical_controller.reset(climbing);
    const auto climb_damping_tau = ned_vertical_controller.update(climbing, 0.1);
    failures += expect(
        climb_damping_tau[4] < 0.0,
        "fixed-wing altitude loop damps NED climb when body w is zero");

    auto slow = hydrox::NavigationState::zeros();
    slow.eta[2] = -45.0;
    slow.nu[0] = 5.0;
    slow.airspeed_valid = true;
    slow.equivalent_airspeed_mps = 5.0;
    ned_vertical_params.altitude_kd = 0.0;
    ned_vertical_params.underspeed_pitch_gain = 0.2;
    hydrox::FixedWingController underspeed_controller(ned_vertical_params);
    underspeed_controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);
    underspeed_controller.set_setpoint(waypoint);
    underspeed_controller.reset(slow);
    const auto underspeed_tau = underspeed_controller.update(slow, 0.1);
    failures += expect(underspeed_tau[4] < 0.0,
                       "fixed-wing underspeed protection sacrifices altitude before stall");

    hydrox::FixedWingAllocator::Params allocator_params;
    allocator_params.aileron_per_moment_inv_Nm = -1.8 / params.Ixx_kgm2;
    allocator_params.elevator_per_moment_inv_Nm = 2.0 / params.Iyy_kgm2;
    hydrox::FixedWingAllocator allocator(allocator_params);
    const auto actuator = allocator.allocate(tau, state.nu[0]);
    failures += expect(
        tau[3] < 0.0 && actuator.ch[1] > 0.0f,
        "Cessna negative roll correction maps to the imported positive aileron channel");
    auto positive_pitch_tau = tau;
    positive_pitch_tau.setZero();
    positive_pitch_tau[4] = 0.02;
    const auto pitch_actuator = allocator.allocate(positive_pitch_tau, state.nu[0]);
    failures += expect(pitch_actuator.ch[0] > 0.0f,
                       "Cessna positive pitch correction maps to the positive elevator channel");
    hydrox::Wrench cruise_tau;
    cruise_tau.setZero();
    cruise_tau[0] = 4.32;
    hydrox::FixedWingAllocator cruise_allocator;
    const auto launch_throttle = cruise_allocator.allocate(cruise_tau, 0.0);
    const auto trimmed_throttle = cruise_allocator.allocate(cruise_tau, 12.0);
    const auto inverted_overspeed_throttle =
        cruise_allocator.allocate(cruise_tau, -100.0);
    failures += expect(
        std::abs(launch_throttle.ch[3] - 0.36f) < 1.0e-6f &&
            std::abs(trimmed_throttle.ch[3] - 0.36f) < 1.0e-6f &&
            std::abs(inverted_overspeed_throttle.ch[3] - 0.36f) < 1.0e-6f,
        "Cessna allocator maps physical thrust once and does not own the speed loop");

    failures += expect(
        launch_throttle.valid() &&
            launch_throttle.layout ==
                hydrox::ActuatorLayout::ConventionalFixedWing &&
            launch_throttle.active_count == 4 &&
            launch_throttle.contract[0].quantity ==
                hydrox::ActuatorQuantity::Position &&
            launch_throttle.contract[3].quantity ==
                hydrox::ActuatorQuantity::Thrust,
        "Cessna allocator declares surface-position and pusher-thrust channels");

    // A trimmed airframe can have a persistent altitude bias.  The bounded
    // outer-loop integral must add a climb demand when the aircraft is below
    // its commanded NED altitude, and reset cleanly between missions.
    hydrox::FixedWingController::Params altitude_params;
    altitude_params.altitude_kp = 0.0;
    altitude_params.altitude_kd = 0.0;
    altitude_params.altitude_ki = 1.0;
    altitude_params.altitude_integral_limit = 0.1;
    altitude_params.pitch_limit_rad = 0.5;
    altitude_params.pitch_trim_moment_Nm = 0.0;
    altitude_params.pitch_attitude_kp = 1.0;
    altitude_params.max_pitch_accel_radps2 = 0.5;
    altitude_params.min_flight_speed_mps = 0.0;
    hydrox::FixedWingController altitude_controller(altitude_params);
    altitude_controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);
    auto altitude_waypoint = waypoint;
    altitude_waypoint.wp_d = -10.0;
    altitude_controller.set_setpoint(altitude_waypoint);
    auto altitude_state = hydrox::NavigationState::zeros();
    altitude_state.eta[2] = 0.0;
    const auto biased_tau = altitude_controller.update(altitude_state, 1.0);
    failures += expect(biased_tau[4] > 0.0,
                       "altitude integral adds a climb demand for negative NED depth error");
    altitude_controller.reset(altitude_state);
    altitude_waypoint.wp_d = 0.0;
    altitude_controller.set_setpoint(altitude_waypoint);
    const auto reset_tau = altitude_controller.update(altitude_state, 0.01);
    failures += expect(reset_tau[4] == 0.0,
                       "altitude integral resets between fixed-wing missions");

    // The pitch inner loop must reject a persistent trim-model mismatch.
    // A pure PD loop would require permanent attitude error to do that.
    hydrox::FixedWingController::Params pitch_integral_params;
    pitch_integral_params.altitude_kp = 0.0;
    pitch_integral_params.altitude_kd = 0.0;
    pitch_integral_params.pitch_trim_moment_Nm = 0.01;
    pitch_integral_params.pitch_attitude_kp = 0.0;
    pitch_integral_params.pitch_attitude_ki = 1.0;
    pitch_integral_params.pitch_error_integral_limit = 0.2;
    pitch_integral_params.pitch_rate_kd = 0.0;
    pitch_integral_params.max_pitch_accel_radps2 = 0.5;
    pitch_integral_params.min_flight_speed_mps = 0.0;
    hydrox::FixedWingController pitch_integral_controller(pitch_integral_params);
    pitch_integral_controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);
    pitch_integral_controller.set_setpoint(waypoint);
    auto pitch_bias_state = hydrox::NavigationState::zeros();
    pitch_bias_state.eta[2] = waypoint.wp_d;
    pitch_bias_state.eta[4] = 0.1;
    pitch_integral_controller.reset(pitch_bias_state);
    const auto pitch_bias_tau =
        pitch_integral_controller.update(pitch_bias_state, 1.0);
    failures += expect(
        pitch_bias_tau[4] < pitch_integral_params.pitch_trim_moment_Nm,
                       "fixed-wing pitch integral rejects a nose-up trim bias");
    pitch_bias_state.eta[4] = 0.0;
    pitch_integral_controller.reset(pitch_bias_state);
    const auto cleared_pitch_tau =
        pitch_integral_controller.update(pitch_bias_state, 0.01);
    failures += expect(
        std::abs(cleared_pitch_tau[4] -
                 pitch_integral_params.pitch_trim_moment_Nm) < 1.0e-12,
        "fixed-wing pitch integral resets to its physical trim moment");

    // A moving NED depth reference is a commanded flight path, not a series
    // of altitude steps. Descending depth references must command nose-down
    // feed-forward before an altitude error develops.
    hydrox::FixedWingController::Params path_angle_params;
    path_angle_params.altitude_kp = 0.0;
    path_angle_params.altitude_kd = 0.0;
    path_angle_params.altitude_ki = 0.0;
    path_angle_params.pitch_trim_moment_Nm = 0.0;
    path_angle_params.pitch_attitude_kp = 1.0;
    path_angle_params.pitch_attitude_ki = 0.0;
    path_angle_params.max_pitch_reference_rate_radps = 10.0;
    path_angle_params.depth_reference_rate_filter_tau_s = 0.0;
    path_angle_params.min_flight_speed_mps = 0.0;
    hydrox::FixedWingController path_angle_controller(path_angle_params);
    path_angle_controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);
    auto path_angle_state = hydrox::NavigationState::zeros();
    path_angle_state.nu[0] = 10.0;
    auto descending_waypoint = waypoint;
    descending_waypoint.wp_d = -45.0;
    path_angle_controller.set_setpoint(descending_waypoint);
    path_angle_controller.reset(path_angle_state);
    (void)path_angle_controller.update(path_angle_state, 0.1);
    descending_waypoint.wp_d = -44.9;
    path_angle_controller.set_setpoint(descending_waypoint);
    const auto descending_tau = path_angle_controller.update(path_angle_state, 0.1);
    failures += expect(descending_tau[4] < 0.0,
                       "fixed-wing flight-path feed-forward anticipates descent");
    // Same airspeed, different GPS ground speed: throttle must remain unchanged.
    auto wind_state = hydrox::NavigationState::zeros();
    wind_state.airspeed_valid = true;
    wind_state.equivalent_airspeed_mps = 12.0;
    wind_state.nu[0] = 7.0;
    controller.reset(wind_state);
    const double headwind_throttle = controller.update(wind_state, 0.01)[0];
    wind_state.nu[0] = 17.0;
    controller.reset(wind_state);
    failures += expect(std::abs(controller.update(wind_state, 0.01)[0] - headwind_throttle) < 1.e-9,
        "wind changes ground speed but not measured-air-speed throttle feedback");
    wind_state.airspeed_valid = false;
    controller.reset(wind_state);
    failures += expect(std::abs(controller.update(wind_state, 0.01)[0] - params.cruise_force_N) < 1.e-9,
        "lost airspeed uses declared trim-throttle fallback, not ground speed");
    if (failures == 0)
        std::cout << "test_fixedwing_controller: all checks passed\n";
    return failures == 0 ? 0 : 1;
}
