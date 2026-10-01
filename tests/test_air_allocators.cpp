#include "gnc/multirotor_allocator.h"
#include "gnc/multirotor_controller.h"
#include "gnc/vtol_controller.h"
#include "gnc/vtol_allocator.h"

#include <cmath>
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

bool approx(double a, double b, double epsilon = 1.0e-6)
{
    return std::abs(a - b) <= epsilon;
}
}

int main()
{
    int fails = 0;

    hydrox::QuadrotorAllocator::Params quad_params;
    quad_params.max_total_thrust_N = 34.19432;
    quad_params.roll_pitch_moment_arm_m = 0.2460731599 * 0.7071067811865476;
    quad_params.yaw_moment_per_thrust_m = 1.3677728e-7 / 8.54858e-6;
    hydrox::QuadrotorAllocator quad(quad_params);
    hydrox::Wrench tau = hydrox::Wrench::Zero();
    tau[2] = 2.0 * 9.80665;
    const auto hover = quad.allocate(tau, 0.0);
    const double expected_hover = tau[2] / quad_params.max_total_thrust_N;
    for (int i = 0; i < 4; ++i)
        fails += expect(approx(hover.ch[i], expected_hover, 1.0e-5),
                        "X500 hover uses the physical UE maximum lift");
    fails += expect(
        hover.valid() && hover.layout == hydrox::ActuatorLayout::QuadX &&
            hover.active_count == 4 &&
            hover.contract[0].quantity == hydrox::ActuatorQuantity::Thrust &&
            !hover.contract[0].reversible,
        "X500 allocator declares four unidirectional normalized-thrust channels");

    tau[3] = 0.5;
    const auto positive_roll = quad.allocate(tau, 0.0);
    fails += expect(positive_roll.ch[0] > positive_roll.ch[1] &&
                    positive_roll.ch[3] > positive_roll.ch[2],
                    "positive roll adds thrust to the left rotor pair");
    const double max_thrust_per_rotor = quad_params.max_total_thrust_N / 4.0;
    const double reconstructed_roll_moment =
        quad_params.roll_pitch_moment_arm_m * max_thrust_per_rotor *
        (positive_roll.ch[0] + positive_roll.ch[3] -
         positive_roll.ch[1] - positive_roll.ch[2]);
    fails += expect(approx(reconstructed_roll_moment, tau[3], 1.0e-5),
                    "roll allocation preserves the requested physical N*m");

    tau[3] = 0.0;
    tau[4] = 0.5;
    const auto positive_pitch = quad.allocate(tau, 0.0);
    fails += expect(positive_pitch.ch[0] > positive_pitch.ch[3] &&
                    positive_pitch.ch[1] > positive_pitch.ch[2],
                    "positive pitch adds thrust to the front rotor pair");
    const double reconstructed_pitch_moment =
        quad_params.roll_pitch_moment_arm_m * max_thrust_per_rotor *
        (positive_pitch.ch[0] + positive_pitch.ch[1] -
         positive_pitch.ch[2] - positive_pitch.ch[3]);
    fails += expect(approx(reconstructed_pitch_moment, tau[4], 1.0e-5),
                    "pitch allocation preserves the requested physical N*m");

    tau[4] = 0.0;
    tau[5] = 0.05;
    const auto positive_yaw = quad.allocate(tau, 0.0);
    fails += expect(positive_yaw.ch[0] > positive_yaw.ch[1] &&
                    positive_yaw.ch[2] > positive_yaw.ch[3],
                    "positive yaw follows the canonical CCW/CW spin order");
    const double reconstructed_yaw_moment =
        quad_params.yaw_moment_per_thrust_m * max_thrust_per_rotor *
        (positive_yaw.ch[0] + positive_yaw.ch[2] -
         positive_yaw.ch[1] - positive_yaw.ch[3]);
    fails += expect(approx(reconstructed_yaw_moment, tau[5], 1.0e-5),
                    "yaw allocation preserves the requested physical N*m");

    hydrox::MultirotorController multirotor_controller;
    hydrox::GNCSetpoint multirotor_waypoint;
    multirotor_waypoint.wp_n = 100.0;
    multirotor_waypoint.wp_d = -45.0;
    multirotor_waypoint.surge_ref = 1.0;
    multirotor_controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);
    multirotor_controller.reset(hydrox::NavigationState::zeros());
    multirotor_controller.set_setpoint(multirotor_waypoint);
    auto multirotor_overspeed = hydrox::NavigationState::zeros();
    multirotor_overspeed.eta[2] = -45.0;
    multirotor_overspeed.nu[0] = 4.0;
    const auto multirotor_braking =
        multirotor_controller.update(multirotor_overspeed, 0.01);
    fails += expect(multirotor_braking[4] > 0.0,
                    "X500 speed limit commands nose-up braking even for a distant waypoint");
    auto multirotor_attitude = hydrox::NavigationState::zeros();
    multirotor_attitude.eta[2] = -45.0;
    multirotor_attitude.eta[3] = 0.1;
    const auto attitude_wrench = multirotor_controller.update(multirotor_attitude, 0.01);
    const auto default_multirotor_params = hydrox::MultirotorController::Params{};
    fails += expect(approx(attitude_wrench[3], -default_multirotor_params.Ixx *
                          default_multirotor_params.roll_kp * 0.1, 1.0e-9),
                    "attitude loop converts angular acceleration to N*m with Ixx");

    hydrox::MultirotorController::Params rolling_compensation_params;
    rolling_compensation_params.mass = 2.0643076923076924;
    rolling_compensation_params.rotor_thrust_coefficient = 8.54858e-6;
    rolling_compensation_params.rotor_rolling_moment_coefficient = 1.0e-6;
    // Isolate the imported rotor rolling-moment compensation from the
    // position/velocity trajectory controller exercised by other tests.
    rolling_compensation_params.xy_kp = 0.0;
    rolling_compensation_params.xy_kd = 0.0;
    hydrox::MultirotorController rolling_compensation_controller(
        rolling_compensation_params);
    hydrox::GNCSetpoint straight_north;
    straight_north.wp_n = 100.0;
    straight_north.wp_d = -45.0;
    straight_north.surge_ref = 3.0;
    rolling_compensation_controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);
    rolling_compensation_controller.reset(hydrox::NavigationState::zeros());
    rolling_compensation_controller.set_setpoint(straight_north);
    auto forward_flight = hydrox::NavigationState::zeros();
    forward_flight.eta[2] = -45.0;
    forward_flight.nu[0] = 3.0;
    const auto forward_compensation =
        rolling_compensation_controller.update(forward_flight, 0.01);
    const double mean_hover_omega = std::sqrt(
        rolling_compensation_params.mass * rolling_compensation_params.g /
        (4.0 * rolling_compensation_params.rotor_thrust_coefficient));
    const double rolling_moment_gain =
        4.0 * mean_hover_omega *
        rolling_compensation_params.rotor_rolling_moment_coefficient;
    fails += expect(approx(forward_compensation[3],
                          rolling_moment_gain * forward_flight.nu[0], 1.0e-9),
                    "X500 cancels forward-speed rotor rolling moment");

    hydrox::GNCSetpoint straight_east = straight_north;
    straight_east.wp_n = 0.0;
    straight_east.wp_e = 100.0;
    straight_east.surge_ref = 2.0;
    rolling_compensation_controller.set_setpoint(straight_east);
    auto lateral_flight = hydrox::NavigationState::zeros();
    lateral_flight.eta[2] = -45.0;
    lateral_flight.nu[1] = 2.0;
    const auto lateral_compensation =
        rolling_compensation_controller.update(lateral_flight, 0.01);
    fails += expect(approx(lateral_compensation[4],
                          rolling_moment_gain * lateral_flight.nu[1], 1.0e-9),
                    "X500 cancels lateral-speed rotor pitching moment");


    auto tilted_level_flight = hydrox::NavigationState::zeros();
    tilted_level_flight.eta[2] = -45.0;
    tilted_level_flight.eta[4] = -0.3;
    tilted_level_flight.nu[0] = std::cos(0.3) * 3.0;
    tilted_level_flight.nu[2] = -std::sin(0.3) * 3.0;
    const auto tilted_level_wrench =
        multirotor_controller.update(tilted_level_flight, 0.01);
    fails += expect(
        tilted_level_wrench[2] >
            hydrox::MultirotorController::Params{}.mass * 9.80665,
        "X500 tilted level flight uses NED down velocity and compensates collective thrust");

    auto multirotor_yaw_hold = hydrox::NavigationState::zeros();
    multirotor_yaw_hold.eta[2] = -45.0;
    multirotor_yaw_hold.eta[5] = 0.7;
    multirotor_controller.reset(multirotor_yaw_hold);
    const auto multirotor_yaw_wrench =
        multirotor_controller.update(multirotor_yaw_hold, 0.01);
    fails += expect(std::abs(multirotor_yaw_wrench[5]) < 1.0e-9,
                    "X500 waypoint entry holds its current yaw");

    hydrox::VtolAllocator::Params vtol_params;
    vtol_params.max_total_lift_N = 180.0;
    hydrox::VtolAllocator vtol(vtol_params);
    tau.setZero();
    tau[2] = 5.0 * 9.80665;
    tau[0] = 3.0;
    tau[3] = 0.25;
    tau[4] = -0.2;
    tau[5] = 0.1;
    const auto vtol_cmd = vtol.allocate(tau, 0.0);
    fails += expect(vtol_cmd.ch[0] > vtol_cmd.ch[1],
                    "VTOL lift rotors share the quadrotor roll convention");
    fails += expect(
        vtol_cmd.ch[4] < 0.0f && vtol_cmd.ch[5] < 0.0f &&
            vtol_cmd.ch[6] < 0.0f,
        "VTOL surfaces match the imported StandardVTOL joint mixes");
    fails += expect(vtol_cmd.ch[7] > 0.0f,
                    "VTOL channel 7 drives the forward pusher");
    fails += expect(
        vtol_cmd.valid() &&
            vtol_cmd.layout == hydrox::ActuatorLayout::LiftCruiseVtol &&
            vtol_cmd.active_count == 8 &&
            vtol_cmd.contract[0].quantity == hydrox::ActuatorQuantity::Thrust &&
            vtol_cmd.contract[4].quantity == hydrox::ActuatorQuantity::Position &&
            vtol_cmd.contract[7].quantity == hydrox::ActuatorQuantity::Thrust,
        "VTOL allocator declares rotor, surface, and pusher channel quantities");

    auto axis_params = vtol_params;
    axis_params.elevator_gain_inv_Nm = 0.4;
    axis_params.aileron_gain_inv_Nm = 0.2;
    axis_params.rudder_gain_inv_Nm = 0.1;
    const auto axis_cmd = hydrox::VtolAllocator(axis_params).allocate(tau, 0.0);
    fails += expect(approx(axis_cmd.ch[4], 0.4 * tau[4]) &&
                    approx(axis_cmd.ch[5], -0.2 * tau[3]) &&
                    approx(axis_cmd.ch[6], -0.1 * tau[5]),
                    "VTOL surface axes use their explicit independent gains");
    for (int axis = 0; axis < 3; ++axis)
    {
        auto disabled_params = axis_params;
        if (axis == 0) disabled_params.elevator_gain_inv_Nm = 0.0;
        if (axis == 1) disabled_params.aileron_gain_inv_Nm = 0.0;
        if (axis == 2) disabled_params.rudder_gain_inv_Nm = 0.0;
        const auto disabled_cmd = hydrox::VtolAllocator(disabled_params).allocate(tau, 0.0);
        for (int channel = 0; channel < 8; ++channel)
            fails += expect(approx(disabled_cmd.ch[channel],
                                   channel == axis + 4 ? 0.0 : axis_cmd.ch[channel]),
                            "zero surface gain disables only the selected VTOL axis");
    }

    tau.setZero();
    tau[2] = 5.0 * 9.80665;
    const auto hover_cmd = vtol.allocate(tau, 0.0);
    fails += expect(std::abs(hover_cmd.ch[7]) < 1.0e-6f,
                    "VTOL hover leaves the pusher off without a speed error");

    hydrox::VtolController::Params controller_params;
    controller_params.xy_kp = 0.35;
    controller_params.xy_kd = 1.0;
    controller_params.max_tilt_rad = 0.28;
    hydrox::VtolController controller(controller_params);
    hydrox::GNCSetpoint waypoint;
    waypoint.wp_n = 8.0;
    waypoint.wp_d = -45.0;
    waypoint.surge_ref = 1.0;
    controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);
    controller.set_setpoint(waypoint);
    auto overspeed_state = hydrox::NavigationState::zeros();
    overspeed_state.eta[2] = -45.0;
    overspeed_state.nu[0] = 4.0;
    const auto braking_wrench = controller.update(overspeed_state, 0.01);
    fails += expect(braking_wrench[4] > 0.0,
                    "VTOL forward overspeed commands a nose-up braking pitch");
    fails += expect(std::abs(braking_wrench[0]) < 1.0e-9,
                    "VTOL waypoint tracking keeps the pusher disabled in lift flight");

    auto terminal_velocity_hold = waypoint;
    terminal_velocity_hold.wp_n = 20.0;
    terminal_velocity_hold.surge_ref = 0.0;
    terminal_velocity_hold.hold_heading = true;
    controller.set_setpoint(terminal_velocity_hold);
    auto below_old_switch = hydrox::NavigationState::zeros();
    below_old_switch.eta[2] = -45.0;
    below_old_switch.nu[0] = 0.19;
    const auto below_switch_wrench = controller.update(below_old_switch, 0.01);
    auto above_old_switch = below_old_switch;
    above_old_switch.nu[0] = 0.21;
    const auto above_switch_wrench = controller.update(above_old_switch, 0.01);
    fails += expect(below_switch_wrench[4] < 0.0 && above_switch_wrench[4] < 0.0,
                    "VTOL terminal velocity target stays continuous across the legacy threshold");
    auto yaw_hold_state = hydrox::NavigationState::zeros();
    yaw_hold_state.eta[2] = -45.0;
    yaw_hold_state.eta[5] = 0.7;
    controller.reset(yaw_hold_state);
    const auto hover_route_wrench = controller.update(yaw_hold_state, 0.01);
    fails += expect(hover_route_wrench[5] < 0.0,
                    "VTOL lift-mode route aligns its nose with the active leg");

    auto terminal_heading_hold = waypoint;
    terminal_heading_hold.hold_heading = true;
    terminal_heading_hold.heading_ref = 0.7;
    controller.set_setpoint(terminal_heading_hold);
    const auto terminal_heading_wrench = controller.update(yaw_hold_state, 0.01);
    fails += expect(std::abs(terminal_heading_wrench[5]) < 1.0e-9,
                    "VTOL terminal hover preserves its captured arrival heading");

    auto cruise_waypoint = waypoint;
    cruise_waypoint.wp_n = 200.0;
    cruise_waypoint.surge_ref = 12.0;
    cruise_waypoint.hold_heading = false;
    auto transition_state = hydrox::NavigationState::zeros();
    transition_state.airspeed_valid = true;
    transition_state.eta[2] = -45.0;
    controller.reset(transition_state);
    controller.set_setpoint(cruise_waypoint);
    const auto front_transition_wrench =
        controller.update(transition_state, 0.1);
    fails += expect(
        controller.flight_mode() ==
                hydrox::VtolController::FlightMode::FrontTransition &&
            front_transition_wrench[0] > 0.0 &&
            front_transition_wrench[0] <= 0.5,
        "VTOL high-speed route enters front transition and enables the pusher");

    transition_state.nu[0] = 11.6;
    transition_state.equivalent_airspeed_mps = 11.6;
    for (int i = 0; i < 6; ++i)
        (void)controller.update(transition_state, 0.1);
    fails += expect(
        controller.flight_mode() == hydrox::VtolController::FlightMode::Cruise,
        "VTOL enters wing-borne cruise only after reaching transition airspeed");

    const auto straight_cruise_wrench =
        controller.update(transition_state, 0.1);
    cruise_waypoint.wp_n = 0.0;
    cruise_waypoint.wp_e = 200.0;
    cruise_waypoint.use_path_segment = true;
    cruise_waypoint.path_start_n = 0.0;
    cruise_waypoint.path_start_e = 0.0;
    cruise_waypoint.lookahead_m = 20.0;
    controller.set_setpoint(cruise_waypoint);
    const auto cruise_turn_wrench = controller.update(transition_state, 0.1);
    fails += expect(cruise_turn_wrench[3] > 0.0,
                    "VTOL cruise uses banked course control for a right turn");
    fails += expect(
        std::abs(cruise_turn_wrench[5]) < 0.08,
        "VTOL path switch rate-limits its course reference instead of snapping yaw");
    fails += expect(
        cruise_turn_wrench[0] >= 0.95 * straight_cruise_wrench[0],
        "VTOL cruise preserves pusher force through a large course change");


    // A path-segment command must follow the segment rather than pointing at
    // the endpoint from the current cross-track position.
    auto segment_capture = cruise_waypoint;
    segment_capture.path_start_n = 0.0;
    segment_capture.path_start_e = 0.0;
    segment_capture.wp_n = 200.0;
    segment_capture.wp_e = 0.0;
    auto cross_track_state = transition_state;
    cross_track_state.eta[0] = 60.0;
    cross_track_state.eta[1] = 30.0;
    cross_track_state.eta[5] = 0.0;
    controller.reset(cross_track_state);
    controller.set_setpoint(segment_capture);
    const auto segment_capture_wrench = controller.update(cross_track_state, 0.1);
    fails += expect(
        segment_capture_wrench[3] < 0.0,
        "VTOL L1 guidance banks back toward the active path segment");

    auto terminal_hover = cruise_waypoint;
    terminal_hover.surge_ref = 0.0;
    terminal_hover.hold_heading = true;
    controller.set_setpoint(terminal_hover);
    const auto back_transition_wrench =
        controller.update(transition_state, 0.1);
    fails += expect(
        controller.flight_mode() ==
                hydrox::VtolController::FlightMode::BackTransition &&
            std::abs(back_transition_wrench[0]) < 1.0e-9,
        "VTOL terminal hold enters back transition and closes the pusher");

    transition_state.nu[0] = 2.0;
    for (int i = 0; i < 6; ++i)
        (void)controller.update(transition_state, 0.1);
    fails += expect(
        controller.flight_mode() == hydrox::VtolController::FlightMode::Hover,
        "VTOL returns to lift-mode hover only after slowing below its gate");

    controller.reset(transition_state);
    controller.set_setpoint(cruise_waypoint);
    transition_state.airspeed_valid = false;
    transition_state.nu[0] = 25.0;
    (void)controller.update(transition_state, 0.1);
    fails += expect(controller.flight_mode() == hydrox::VtolController::FlightMode::Hover,
        "high ground speed without pitot cannot authorize front transition");
    transition_state.airspeed_valid = true;
    transition_state.equivalent_airspeed_mps = 12.0;
    for (int i = 0; i < 10; ++i) (void)controller.update(transition_state, 0.1);
    fails += expect(controller.flight_mode() == hydrox::VtolController::FlightMode::Cruise,
        "valid pitot permits cruise");
    transition_state.airspeed_valid = false;
    (void)controller.update(transition_state, 0.1);
    fails += expect(controller.flight_mode() == hydrox::VtolController::FlightMode::BackTransition,
        "pitot loss in cruise requests back transition");
    (void)controller.update(transition_state, 0.1);
    fails += expect(controller.flight_mode() == hydrox::VtolController::FlightMode::BackTransition,
        "persistent cruise command must not restart transition without pitot");
    if (fails == 0)
        std::cout << "test_air_allocators: all checks passed\n";
    return fails == 0 ? 0 : 1;
}
