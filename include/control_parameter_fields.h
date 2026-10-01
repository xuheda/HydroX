// Copyright (c) 2026 OceanX.
#pragma once
#include "control_parameters.h"

namespace hydrox
{
// One explicit list defines the numeric schema. Every selected field is required.
// JSON paths retain the controller parameter names and their documented SI units.
template <class P, class Visitor> void visit_control_parameters(P &p, Visitor &&visit)
{
    visit("control_model.mass_kg", p.mass_total);
    visit("control_model.motor.tau_m", p.motor.tau_m);
    visit("control_model.motor.KT_0", p.motor.KT_0);
    visit("control_model.motor.KQ_0", p.motor.KQ_0);
    visit("control_model.motor.D_prop", p.motor.D_prop);
    visit("control_model.motor.rho", p.motor.rho);
    visit("control_model.motor.eta_motor", p.motor.eta_motor);
    visit("control_model.motor.V_bat", p.motor.V_bat);
    visit("control_model.motor.rpm_max", p.motor.rpm_max);
    switch (p.archetype)
    {
    case VehicleArchetype::SlenderBodyFin:
        visit("control_model.pitch_inertia_kg_m2", p.M44_pitch);
        visit("control_model.controller.depth.kp", p.gnc.depth.kp);
        visit("control_model.controller.depth.kd", p.gnc.depth.kd);
        visit("control_model.controller.depth.ki", p.gnc.depth.ki);
        visit("control_model.controller.depth.wn", p.gnc.depth.wn);
        visit("control_model.controller.depth.zeta", p.gnc.depth.zeta);
        visit("control_model.controller.depth.vmax", p.gnc.depth.vmax);
        visit("control_model.controller.depth.ei_max", p.gnc.depth.ei_max);
        visit("control_model.controller.depth.theta_max", p.gnc.depth.theta_max);
        visit("control_model.controller.pitch.kp", p.gnc.pitch.kp);
        visit("control_model.controller.pitch.kd", p.gnc.pitch.kd);
        visit("control_model.controller.pitch.ki", p.gnc.pitch.ki);
        visit("control_model.controller.pitch.ei_max", p.gnc.pitch.ei_max);
        visit("control_model.controller.pitch.tau_max", p.gnc.pitch.tau_max);
        visit("control_model.controller.heading.T", p.gnc.heading.T);
        visit("control_model.controller.heading.K", p.gnc.heading.K);
        visit("control_model.controller.heading.wn", p.gnc.heading.wn);
        visit("control_model.controller.heading.zeta", p.gnc.heading.zeta);
        visit("control_model.controller.heading.vmax", p.gnc.heading.vmax);
        visit("control_model.controller.heading.kd", p.gnc.heading.kd);
        visit("control_model.controller.heading.ks", p.gnc.heading.ks);
        visit("control_model.controller.heading.phi_b", p.gnc.heading.phi_b);
        visit("control_model.controller.heading.ei_max", p.gnc.heading.ei_max);
        visit("control_model.controller.surge.kp", p.gnc.surge.kp);
        visit("control_model.controller.surge.ki", p.gnc.surge.ki);
        visit("control_model.controller.surge.kd", p.gnc.surge.kd);
        visit("control_model.controller.surge.integral_limit", p.gnc.surge.integral_limit);
        visit("control_model.controller.surge.unwind_gain", p.gnc.surge.unwind_gain);
        visit("control_model.controller.surge.drag_ff", p.gnc.surge.drag_ff);
        visit("control_model.controller.surge.tau_max", p.gnc.surge.tau_max);
        visit("control_model.controller.surge.forward_min_tau", p.gnc.surge.forward_min_tau);
        visit("control_model.controller.surge.tau_rate_limit", p.gnc.surge.tau_rate_limit);
        visit("control_model.controller.surge.overspeed_deadband", p.gnc.surge.overspeed_deadband);
        visit("control_model.controller.surge.overspeed_brake_gain",
              p.gnc.surge.overspeed_brake_gain);
        visit("control_model.controller.surge.overspeed_brake_tau_max",
              p.gnc.surge.overspeed_brake_tau_max);
        visit("control_model.controller.yaw_rate.kp", p.gnc.yaw_rate.kp);
        visit("control_model.controller.yaw_rate.ki", p.gnc.yaw_rate.ki);
        visit("control_model.controller.yaw_rate.kd", p.gnc.yaw_rate.kd);
        visit("control_model.controller.yaw_rate.ei_max", p.gnc.yaw_rate.ei_max);
        visit("control_model.controller.yaw_rate.tau_max", p.gnc.yaw_rate.tau_max);
        visit("control_model.controller.yaw_rate.command_gain", p.gnc.yaw_rate.command_gain);
        visit("control_model.controller.yaw_rate.feed_forward", p.gnc.yaw_rate.feed_forward);
        visit("control_model.controller.yaw_rate.ref_filter_tau", p.gnc.yaw_rate.ref_filter_tau);
        visit("control_model.controller.yaw_rate.ref_slew_limit", p.gnc.yaw_rate.ref_slew_limit);
        visit("control_model.controller.turn_speed.drop_start_radps",
              p.gnc.turn_speed.drop_start_radps);
        visit("control_model.controller.turn_speed.drop_gain_mps_per_radps",
              p.gnc.turn_speed.drop_gain_mps_per_radps);
        visit("control_model.controller.turn_speed.drop_max_mps", p.gnc.turn_speed.drop_max_mps);
        visit("control_model.allocator.rho", p.allocator.rho);
        visit("control_model.allocator.S_fin", p.allocator.S_fin);
        visit("control_model.allocator.CL_s", p.allocator.CL_s);
        visit("control_model.allocator.CL_r", p.allocator.CL_r);
        visit("control_model.allocator.x_fin", p.allocator.x_fin);
        visit("control_model.allocator.D_prop", p.allocator.D_prop);
        visit("control_model.allocator.KT_0", p.allocator.KT_0);
        visit("control_model.allocator.n_max_rpm", p.allocator.n_max_rpm);
        visit("control_model.allocator.max_thrust_N", p.allocator.max_thrust_N);
        visit("control_model.allocator.delta_max_deg", p.allocator.delta_max_deg);
        visit("control_model.allocator.u_min", p.allocator.u_min);
        visit("control_model.allocator.fin_angles_deg", p.allocator.fin_angles_deg);
        break;
    case VehicleArchetype::Thruster:
        visit("control_model.max_thrust_per_actuator_n", p.max_thrust_per_thruster_N);
        visit("control_model.controller.surge.kp", p.thruster_gnc.surge.kp);
        visit("control_model.controller.surge.ki", p.thruster_gnc.surge.ki);
        visit("control_model.controller.surge.kd", p.thruster_gnc.surge.kd);
        visit("control_model.controller.surge.integral_limit", p.thruster_gnc.surge.integral_limit);
        visit("control_model.controller.surge.accel_max", p.thruster_gnc.surge.accel_max);
        visit("control_model.controller.sway.kp", p.thruster_gnc.sway.kp);
        visit("control_model.controller.sway.ki", p.thruster_gnc.sway.ki);
        visit("control_model.controller.sway.kd", p.thruster_gnc.sway.kd);
        visit("control_model.controller.sway.integral_limit", p.thruster_gnc.sway.integral_limit);
        visit("control_model.controller.sway.accel_max", p.thruster_gnc.sway.accel_max);
        visit("control_model.controller.heave.kp", p.thruster_gnc.heave.kp);
        visit("control_model.controller.heave.ki", p.thruster_gnc.heave.ki);
        visit("control_model.controller.heave.kd", p.thruster_gnc.heave.kd);
        visit("control_model.controller.heave.integral_limit", p.thruster_gnc.heave.integral_limit);
        visit("control_model.controller.heave.accel_max", p.thruster_gnc.heave.accel_max);
        visit("control_model.controller.roll.kp", p.thruster_gnc.roll.kp);
        visit("control_model.controller.roll.ki", p.thruster_gnc.roll.ki);
        visit("control_model.controller.roll.kd", p.thruster_gnc.roll.kd);
        visit("control_model.controller.roll.integral_limit", p.thruster_gnc.roll.integral_limit);
        visit("control_model.controller.roll.accel_max", p.thruster_gnc.roll.accel_max);
        visit("control_model.controller.pitch.kp", p.thruster_gnc.pitch.kp);
        visit("control_model.controller.pitch.ki", p.thruster_gnc.pitch.ki);
        visit("control_model.controller.pitch.kd", p.thruster_gnc.pitch.kd);
        visit("control_model.controller.pitch.integral_limit", p.thruster_gnc.pitch.integral_limit);
        visit("control_model.controller.pitch.accel_max", p.thruster_gnc.pitch.accel_max);
        visit("control_model.controller.yaw.kp", p.thruster_gnc.yaw.kp);
        visit("control_model.controller.yaw.ki", p.thruster_gnc.yaw.ki);
        visit("control_model.controller.yaw.kd", p.thruster_gnc.yaw.kd);
        visit("control_model.controller.yaw.integral_limit", p.thruster_gnc.yaw.integral_limit);
        visit("control_model.controller.yaw.accel_max", p.thruster_gnc.yaw.accel_max);
        break;
    case VehicleArchetype::Surface:
        visit("control_model.max_thrust_per_actuator_n", p.max_thrust_per_thruster_N);
        visit("control_model.channel_surge_limit_n", p.surface_channel_surge_limit_N);
        visit("control_model.channel_lever_arm_m", p.surface_channel_lever_arm_m);
        visit("control_model.controller.surge_kp", p.surface_gnc.surge_kp);
        visit("control_model.controller.surge_kd", p.surface_gnc.surge_kd);
        visit("control_model.controller.surge_accel_filter_tau_s",
              p.surface_gnc.surge_accel_filter_tau_s);
        visit("control_model.controller.surge_ki", p.surface_gnc.surge_ki);
        visit("control_model.controller.surge_integral_limit", p.surface_gnc.surge_integral_limit);
        visit("control_model.controller.surge_drag_linear_N_per_mps",
              p.surface_gnc.surge_drag_linear_N_per_mps);
        visit("control_model.controller.surge_drag_quadratic_N_per_mps2",
              p.surface_gnc.surge_drag_quadratic_N_per_mps2);
        visit("control_model.controller.max_brake_force_N", p.surface_gnc.max_brake_force_N);
        visit("control_model.controller.surge_feedback_force_limit_N",
              p.surface_gnc.surge_feedback_force_limit_N);
        visit("control_model.controller.yaw_kp", p.surface_gnc.yaw_kp);
        visit("control_model.controller.yaw_kd", p.surface_gnc.yaw_kd);
        visit("control_model.controller.max_yaw_rate_radps", p.surface_gnc.max_yaw_rate_radps);
        visit("control_model.controller.max_force_N", p.surface_gnc.max_force_N);
        visit("control_model.controller.max_moment_Nm", p.surface_gnc.max_moment_Nm);
        visit("control_model.controller.waypoint_surge_mps", p.surface_gnc.waypoint_surge_mps);
        visit("control_model.controller.station_keep_surge_mps",
              p.surface_gnc.station_keep_surge_mps);
        visit("control_model.controller.station_keep_hysteresis_m",
              p.surface_gnc.station_keep_hysteresis_m);
        visit("control_model.controller.los_lookahead_m", p.surface_gnc.los_lookahead_m);
        visit("control_model.controller.ilos_integral_gain", p.surface_gnc.ilos_integral_gain);
        visit("control_model.controller.ilos_integral_limit", p.surface_gnc.ilos_integral_limit);
        visit("control_model.controller.sideslip_compensation_gain",
              p.surface_gnc.sideslip_compensation_gain);
        visit("control_model.controller.sideslip_filter_time_constant_s",
              p.surface_gnc.sideslip_filter_time_constant_s);
        visit("control_model.controller.max_crab_angle_rad", p.surface_gnc.max_crab_angle_rad);
        visit("control_model.controller.max_accel_mps2", p.surface_gnc.max_accel_mps2);
        visit("control_model.controller.max_decel_mps2", p.surface_gnc.max_decel_mps2);
        break;
    case VehicleArchetype::Multirotor:
        visit("control_model.max_total_lift_n", p.max_total_lift_N);
        visit("control_model.lift_roll_pitch_moment_arm_m", p.lift_roll_pitch_moment_arm_m);
        visit("control_model.lift_yaw_moment_per_thrust_m", p.lift_yaw_moment_per_thrust_m);
        visit("control_model.controller.g", p.multirotor_gnc.g);
        visit("control_model.controller.Ixx", p.multirotor_gnc.Ixx);
        visit("control_model.controller.rotor_thrust_coefficient",
              p.multirotor_gnc.rotor_thrust_coefficient);
        visit("control_model.controller.rotor_rolling_moment_coefficient",
              p.multirotor_gnc.rotor_rolling_moment_coefficient);
        visit("control_model.controller.Iyy", p.multirotor_gnc.Iyy);
        visit("control_model.controller.Izz", p.multirotor_gnc.Izz);
        visit("control_model.controller.z_kp", p.multirotor_gnc.z_kp);
        visit("control_model.controller.z_ki", p.multirotor_gnc.z_ki);
        visit("control_model.controller.z_integral_accel_limit",
              p.multirotor_gnc.z_integral_accel_limit);
        visit("control_model.controller.z_kd", p.multirotor_gnc.z_kd);
        visit("control_model.controller.roll_kp", p.multirotor_gnc.roll_kp);
        visit("control_model.controller.roll_kd", p.multirotor_gnc.roll_kd);
        visit("control_model.controller.pitch_kp", p.multirotor_gnc.pitch_kp);
        visit("control_model.controller.pitch_kd", p.multirotor_gnc.pitch_kd);
        visit("control_model.controller.yaw_kp", p.multirotor_gnc.yaw_kp);
        visit("control_model.controller.yaw_kd", p.multirotor_gnc.yaw_kd);
        visit("control_model.controller.xy_kp", p.multirotor_gnc.xy_kp);
        visit("control_model.controller.xy_kd", p.multirotor_gnc.xy_kd);
        visit("control_model.controller.xy_velocity_ki", p.multirotor_gnc.xy_velocity_ki);
        visit("control_model.controller.xy_accel_damping", p.multirotor_gnc.xy_accel_damping);
        visit("control_model.controller.xy_integral_accel_limit",
              p.multirotor_gnc.xy_integral_accel_limit);
        visit("control_model.controller.xy_anti_windup_gain", p.multirotor_gnc.xy_anti_windup_gain);
        visit("control_model.controller.hold_max_speed_mps", p.multirotor_gnc.hold_max_speed_mps);
        visit("control_model.controller.velocity_reference_tau_s",
              p.multirotor_gnc.velocity_reference_tau_s);
        visit("control_model.controller.max_xy_jerk_mps3", p.multirotor_gnc.max_xy_jerk_mps3);
        visit("control_model.controller.measured_accel_filter_tau_s",
              p.multirotor_gnc.measured_accel_filter_tau_s);
        visit("control_model.controller.max_tilt_rad", p.multirotor_gnc.max_tilt_rad);
        visit("control_model.controller.max_xy_accel", p.multirotor_gnc.max_xy_accel);
        visit("control_model.controller.max_xy_brake_accel", p.multirotor_gnc.max_xy_brake_accel);
        visit("control_model.controller.max_roll_accel_radps2",
              p.multirotor_gnc.max_roll_accel_radps2);
        visit("control_model.controller.max_pitch_accel_radps2",
              p.multirotor_gnc.max_pitch_accel_radps2);
        visit("control_model.controller.max_yaw_accel_radps2",
              p.multirotor_gnc.max_yaw_accel_radps2);
        visit("control_model.controller.max_z_accel", p.multirotor_gnc.max_z_accel);
        break;
    case VehicleArchetype::FixedWing:
        visit("control_model.controller.cruise_speed_mps", p.fixedwing_gnc.cruise_speed_mps);
        visit("control_model.controller.cruise_force_N", p.fixedwing_gnc.cruise_force_N);
        visit("control_model.controller.speed_force_kp_N_per_mps",
              p.fixedwing_gnc.speed_force_kp_N_per_mps);
        visit("control_model.controller.max_forward_force_N", p.fixedwing_gnc.max_forward_force_N);
        visit("control_model.controller.Ixx_kgm2", p.fixedwing_gnc.Ixx_kgm2);
        visit("control_model.controller.Iyy_kgm2", p.fixedwing_gnc.Iyy_kgm2);
        visit("control_model.controller.Izz_kgm2", p.fixedwing_gnc.Izz_kgm2);
        visit("control_model.controller.yaw_rate_kp_radps2_per_radps",
              p.fixedwing_gnc.yaw_rate_kp_radps2_per_radps);
        visit("control_model.controller.max_yaw_accel_radps2",
              p.fixedwing_gnc.max_yaw_accel_radps2);
        visit("control_model.controller.altitude_kp", p.fixedwing_gnc.altitude_kp);
        visit("control_model.controller.altitude_kd", p.fixedwing_gnc.altitude_kd);
        visit("control_model.controller.altitude_ki", p.fixedwing_gnc.altitude_ki);
        visit("control_model.controller.altitude_integral_limit",
              p.fixedwing_gnc.altitude_integral_limit);
        visit("control_model.controller.depth_reference_rate_filter_tau_s",
              p.fixedwing_gnc.depth_reference_rate_filter_tau_s);
        visit("control_model.controller.depth_reference_rate_stale_s",
              p.fixedwing_gnc.depth_reference_rate_stale_s);
        visit("control_model.controller.max_depth_reference_rate_mps",
              p.fixedwing_gnc.max_depth_reference_rate_mps);
        visit("control_model.controller.flight_path_feedforward_gain",
              p.fixedwing_gnc.flight_path_feedforward_gain);
        visit("control_model.controller.pitch_limit_rad", p.fixedwing_gnc.pitch_limit_rad);
        visit("control_model.controller.max_pitch_reference_rate_radps",
              p.fixedwing_gnc.max_pitch_reference_rate_radps);
        visit("control_model.controller.min_flight_speed_mps",
              p.fixedwing_gnc.min_flight_speed_mps);
        visit("control_model.controller.max_flight_speed_mps",
              p.fixedwing_gnc.max_flight_speed_mps);
        visit("control_model.controller.underspeed_pitch_gain",
              p.fixedwing_gnc.underspeed_pitch_gain);
        visit("control_model.controller.overspeed_pitch_gain",
              p.fixedwing_gnc.overspeed_pitch_gain);
        visit("control_model.controller.l1_period_s", p.fixedwing_gnc.l1_period_s);
        visit("control_model.controller.l1_damping", p.fixedwing_gnc.l1_damping);
        visit("control_model.controller.l1_min_distance_m", p.fixedwing_gnc.l1_min_distance_m);
        visit("control_model.controller.l1_max_course_error_rad",
              p.fixedwing_gnc.l1_max_course_error_rad);
        visit("control_model.controller.course_kp", p.fixedwing_gnc.course_kp);
        visit("control_model.controller.roll_limit_rad", p.fixedwing_gnc.roll_limit_rad);
        visit("control_model.controller.max_roll_reference_rate_radps",
              p.fixedwing_gnc.max_roll_reference_rate_radps);
        visit("control_model.controller.max_coordinated_yaw_rate_radps",
              p.fixedwing_gnc.max_coordinated_yaw_rate_radps);
        visit("control_model.controller.roll_attitude_kp", p.fixedwing_gnc.roll_attitude_kp);
        visit("control_model.controller.roll_rate_kd", p.fixedwing_gnc.roll_rate_kd);
        visit("control_model.controller.max_roll_accel_radps2",
              p.fixedwing_gnc.max_roll_accel_radps2);
        visit("control_model.controller.pitch_attitude_kp", p.fixedwing_gnc.pitch_attitude_kp);
        visit("control_model.controller.pitch_attitude_ki", p.fixedwing_gnc.pitch_attitude_ki);
        visit("control_model.controller.pitch_error_integral_limit",
              p.fixedwing_gnc.pitch_error_integral_limit);
        visit("control_model.controller.pitch_rate_kd", p.fixedwing_gnc.pitch_rate_kd);
        visit("control_model.controller.pitch_trim_moment_Nm",
              p.fixedwing_gnc.pitch_trim_moment_Nm);
        visit("control_model.controller.max_pitch_accel_radps2",
              p.fixedwing_gnc.max_pitch_accel_radps2);
        visit("control_model.allocator.elevator_per_moment_inv_Nm",
              p.fixedwing_allocator.elevator_per_moment_inv_Nm);
        visit("control_model.allocator.aileron_per_moment_inv_Nm",
              p.fixedwing_allocator.aileron_per_moment_inv_Nm);
        visit("control_model.allocator.rudder_per_moment_inv_Nm",
              p.fixedwing_allocator.rudder_per_moment_inv_Nm);
        visit("control_model.allocator.reference_airspeed_mps",
              p.fixedwing_allocator.reference_airspeed_mps);
        visit("control_model.allocator.minimum_control_airspeed_mps",
              p.fixedwing_allocator.minimum_control_airspeed_mps);
        visit("control_model.allocator.elevator_limit_rad",
              p.fixedwing_allocator.elevator_limit_rad);
        visit("control_model.allocator.aileron_limit_rad", p.fixedwing_allocator.aileron_limit_rad);
        visit("control_model.allocator.rudder_limit_rad", p.fixedwing_allocator.rudder_limit_rad);
        visit("control_model.allocator.max_forward_thrust_N",
              p.fixedwing_allocator.max_forward_thrust_N);
        break;
    case VehicleArchetype::VTOL:
        visit("control_model.max_total_lift_n", p.max_total_lift_N);
        visit("control_model.lift_roll_pitch_moment_arm_m", p.lift_roll_pitch_moment_arm_m);
        visit("control_model.lift_yaw_moment_per_thrust_m", p.lift_yaw_moment_per_thrust_m);
        visit("control_model.controller.g", p.vtol_gnc.g);
        visit("control_model.controller.z_kp", p.vtol_gnc.z_kp);
        visit("control_model.controller.z_kd", p.vtol_gnc.z_kd);
        visit("control_model.controller.roll_kp", p.vtol_gnc.roll_kp);
        visit("control_model.controller.roll_kd", p.vtol_gnc.roll_kd);
        visit("control_model.controller.pitch_kp", p.vtol_gnc.pitch_kp);
        visit("control_model.controller.pitch_kd", p.vtol_gnc.pitch_kd);
        visit("control_model.controller.yaw_kp", p.vtol_gnc.yaw_kp);
        visit("control_model.controller.yaw_kd", p.vtol_gnc.yaw_kd);
        visit("control_model.controller.max_roll_moment_Nm", p.vtol_gnc.max_roll_moment_Nm);
        visit("control_model.controller.max_pitch_moment_Nm", p.vtol_gnc.max_pitch_moment_Nm);
        visit("control_model.controller.max_yaw_moment_Nm", p.vtol_gnc.max_yaw_moment_Nm);
        visit("control_model.controller.xy_kp", p.vtol_gnc.xy_kp);
        visit("control_model.controller.xy_kd", p.vtol_gnc.xy_kd);
        visit("control_model.controller.max_tilt_rad", p.vtol_gnc.max_tilt_rad);
        visit("control_model.controller.hover_max_speed_mps", p.vtol_gnc.hover_max_speed_mps);
        visit("control_model.controller.hover_hold_max_speed_mps",
              p.vtol_gnc.hover_hold_max_speed_mps);
        visit("control_model.controller.cruise_speed_mps", p.vtol_gnc.cruise_speed_mps);
        visit("control_model.controller.pusher_trim_force_N", p.vtol_gnc.pusher_trim_force_N);
        visit("control_model.controller.pusher_drag_force_N_per_mps2",
              p.vtol_gnc.pusher_drag_force_N_per_mps2);
        visit("control_model.controller.pusher_speed_kp_N_per_mps",
              p.vtol_gnc.pusher_speed_kp_N_per_mps);
        visit("control_model.controller.max_pusher_force_N", p.vtol_gnc.max_pusher_force_N);
        visit("control_model.controller.transition_command_min_speed_mps",
              p.vtol_gnc.transition_command_min_speed_mps);
        visit("control_model.controller.transition_start_mps", p.vtol_gnc.transition_start_mps);
        visit("control_model.controller.transition_end_mps", p.vtol_gnc.transition_end_mps);
        visit("control_model.controller.backtransition_complete_mps",
              p.vtol_gnc.backtransition_complete_mps);
        visit("control_model.controller.cruise_course_kp", p.vtol_gnc.cruise_course_kp);
        visit("control_model.controller.cruise_roll_limit_rad", p.vtol_gnc.cruise_roll_limit_rad);
        visit("control_model.controller.cruise_l1_period_s", p.vtol_gnc.cruise_l1_period_s);
        visit("control_model.controller.cruise_l1_damping", p.vtol_gnc.cruise_l1_damping);
        visit("control_model.controller.cruise_l1_min_distance_m",
              p.vtol_gnc.cruise_l1_min_distance_m);
        visit("control_model.controller.cruise_l1_max_course_error_rad",
              p.vtol_gnc.cruise_l1_max_course_error_rad);
        visit("control_model.controller.max_course_reference_rate_radps",
              p.vtol_gnc.max_course_reference_rate_radps);
        visit("control_model.controller.cruise_altitude_kp", p.vtol_gnc.cruise_altitude_kp);
        visit("control_model.controller.cruise_altitude_kd", p.vtol_gnc.cruise_altitude_kd);
        visit("control_model.controller.cruise_pitch_limit_rad", p.vtol_gnc.cruise_pitch_limit_rad);
        visit("control_model.controller.pusher_heading_gate_rad",
              p.vtol_gnc.pusher_heading_gate_rad);
        visit("control_model.controller.pusher_speed_ramp_mps2", p.vtol_gnc.pusher_speed_ramp_mps2);
        visit("control_model.allocator.elevator_gain_inv_Nm",
              p.vtol_allocator.elevator_gain_inv_Nm);
        visit("control_model.allocator.aileron_gain_inv_Nm", p.vtol_allocator.aileron_gain_inv_Nm);
        visit("control_model.allocator.rudder_gain_inv_Nm", p.vtol_allocator.rudder_gain_inv_Nm);
        visit("control_model.allocator.surface_limit_rad", p.vtol_allocator.surface_limit_rad);
        visit("control_model.allocator.max_pusher_thrust_N", p.vtol_allocator.max_pusher_thrust_N);
        break;
    case VehicleArchetype::DifferentialDrive:
        visit("control_model.controller.surge_kp", p.ground_gnc.surge_kp);
        visit("control_model.controller.surge_ki", p.ground_gnc.surge_ki);
        visit("control_model.controller.surge_integral_limit", p.ground_gnc.surge_integral_limit);
        visit("control_model.controller.yaw_heading_kp", p.ground_gnc.yaw_heading_kp);
        visit("control_model.controller.yaw_rate_kp", p.ground_gnc.yaw_rate_kp);
        visit("control_model.controller.max_force_N", p.ground_gnc.max_force_N);
        visit("control_model.controller.max_moment_Nm", p.ground_gnc.max_moment_Nm);
        visit("control_model.controller.waypoint_surge_mps", p.ground_gnc.waypoint_surge_mps);
        visit("control_model.controller.waypoint_stop_radius_m",
              p.ground_gnc.waypoint_stop_radius_m);
        visit("control_model.controller.waypoint_slowdown_m", p.ground_gnc.waypoint_slowdown_m);
        visit("control_model.controller.max_yaw_rate_radps", p.ground_gnc.max_yaw_rate_radps);
        visit("control_model.allocator.wheel_radius_m", p.ground_allocator.wheel_radius_m);
        visit("control_model.allocator.track_width_m", p.ground_allocator.track_width_m);
        visit("control_model.allocator.max_wheel_angular_speed_radps",
              p.ground_allocator.max_wheel_angular_speed_radps);
        visit("control_model.allocator.longitudinal_speed_gain_N_per_mps",
              p.ground_allocator.longitudinal_speed_gain_N_per_mps);
        break;
    }
}
} // namespace hydrox
