// Copyright (c) 2026 OceanX
#include "gnc/vtol_controller.h"

#include <algorithm>
#include <cmath>

namespace hydrox
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double clamp_abs(double v, double limit)
{
    return std::max(-limit, std::min(limit, v));
}

Eigen::Vector2d clamp_norm(const Eigen::Vector2d& value, double limit)
{
    const double safe_limit = std::max(0.0, limit);
    const double norm = value.norm();
    if (norm > safe_limit && norm > 1.0e-9)
        return value * (safe_limit / norm);
    return value;
}

double clamp01(double v)
{
    return std::max(0.0, std::min(1.0, v));
}

double wrap_pi(double a)
{
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

double smoothstep(double value)
{
    const double x = clamp01(value);
    return x * x * (3.0 - 2.0 * x);
}

double lerp(double a, double b, double alpha)
{
    return a + clamp01(alpha) * (b - a);
}

double slew_angle(double current, double target, double rate_limit, double dt)
{
    if (!(dt > 0.0) || !(rate_limit > 0.0))
        return wrap_pi(target);
    return wrap_pi(current + clamp_abs(
        wrap_pi(target - current), rate_limit * dt));
}

Eigen::Vector3d ned_velocity(const NavigationState& state)
{
    const double roll = state.eta[3];
    const double pitch = state.eta[4];
    const double yaw = state.eta[5];
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    return {
        cy * cp * state.nu[0] +
            (cy * sp * sr - sy * cr) * state.nu[1] +
            (cy * sp * cr + sy * sr) * state.nu[2],
        sy * cp * state.nu[0] +
            (sy * sp * sr + cy * cr) * state.nu[1] +
            (sy * sp * cr - cy * sr) * state.nu[2],
        -sp * state.nu[0] + cp * sr * state.nu[1] +
            cp * cr * state.nu[2]};
}
}

VtolController::VtolController(const Params& p) : _p(p) {}

void VtolController::enter_flight_mode(
    FlightMode mode, const NavigationState& state)
{
    if (mode == _flight_mode)
        return;
    _flight_mode = mode;
    _flight_mode_elapsed_s = 0.0;
    if (mode == FlightMode::Hover || mode == FlightMode::BackTransition)
    {
        _heading_hold = _sp.hold_heading ? _sp.heading_ref : state.eta[5];
        _heading_hold_valid = true;
    }
}

void VtolController::reset(const NavigationState& state)
{
    _transition_fault = false;
    _transition_entry_depth_m = state.depth_m;
    _flight_mode = FlightMode::Hover;
    _flight_mode_elapsed_s = 0.0;
    _pusher_speed_reference_mps = 0.0;
    _course_reference_rad = state.eta[5];
    _course_reference_valid = true;
    _heading_hold = state.eta[5];
    _heading_hold_valid = true;
}

void VtolController::set_mode(GNCMode mode)
{
    // A reference/source change is not a physical transition to hover.
    // Only disabling the controller resets the flight-phase state machine.
    if (mode == GNCMode::DISABLED)
    {
        _transition_fault = false;
        _flight_mode = FlightMode::Hover;
        _flight_mode_elapsed_s = 0.0;
        _pusher_speed_reference_mps = 0.0;
        _course_reference_valid = false;
        _heading_hold_valid = false;
    }
    _mode = mode;
}

Wrench VtolController::update(const NavigationState& state, double dt)
{
    Wrench tau;
    tau.setZero();
    if (_mode == GNCMode::DISABLED)
        return tau;

    const double control_dt = std::isfinite(dt) && dt > 0.0
                                  ? std::min(dt, 0.1)
                                  : 0.0;
    _flight_mode_elapsed_s += control_dt;

    const Eigen::Vector3d velocity_ned = ned_velocity(state);
    const double ground_speed = velocity_ned.head<2>().norm();
    const bool airspeed_valid = state.airspeed_valid &&
        std::isfinite(state.equivalent_airspeed_mps) && state.equivalent_airspeed_mps >= 0.0;
    const double flight_speed = airspeed_valid ? state.equivalent_airspeed_mps : 0.0;
    const double actual_course = ground_speed > 1.0
                                     ? std::atan2(velocity_ned.y(), velocity_ned.x())
                                     : state.eta[5];

    double depth_ref = _sp.depth_ref;
    double dn = 0.0;
    double de = 0.0;
    double horizontal_distance = 0.0;
    double requested_speed = std::max(0.0, _sp.surge_ref);
    double raw_course_ref = _sp.heading_ref;
    if (_mode == GNCMode::WAYPOINT_3D)
    {
        depth_ref = _sp.wp_d;
        dn = _sp.wp_n - state.eta[0];
        de = _sp.wp_e - state.eta[1];
        horizontal_distance = std::hypot(dn, de);
        if (requested_speed <= 0.1 && !_sp.hold_heading)
            requested_speed = _p.cruise_speed_mps;
        if (!_sp.hold_heading && horizontal_distance > 0.25)
            raw_course_ref = std::atan2(de, dn);

        // Aim at a speed-scaled point ahead of the aircraft's orthogonal
        // projection on the active segment. This preserves path continuity
        // instead of steering at a moving point target.
        if (!_sp.hold_heading && _sp.use_path_segment)
        {
            const double segment_n = _sp.wp_n - _sp.path_start_n;
            const double segment_e = _sp.wp_e - _sp.path_start_e;
            const double segment_length = std::hypot(segment_n, segment_e);
            if (segment_length > 0.25)
            {
                const double damping = std::max(0.1, _p.cruise_l1_damping);
                const double guidance_speed = std::max(
                    ground_speed,
                    std::max(_p.hover_max_speed_mps, requested_speed));
                const double l1_distance = std::max(
                    std::max(_p.cruise_l1_min_distance_m, _sp.lookahead_m),
                    damping * std::max(1.0, _p.cruise_l1_period_s) *
                        guidance_speed / kPi);
                const double unit_n = segment_n / segment_length;
                const double unit_e = segment_e / segment_length;
                const double rel_n = state.eta[0] - _sp.path_start_n;
                const double rel_e = state.eta[1] - _sp.path_start_e;
                const double along_track = rel_n * unit_n + rel_e * unit_e;
                const double target_n = _sp.path_start_n +
                    (along_track + l1_distance) * unit_n;
                const double target_e = _sp.path_start_e +
                    (along_track + l1_distance) * unit_e;
                raw_course_ref = std::atan2(
                    target_e - state.eta[1], target_n - state.eta[0]);
            }
        }
    }

    if (!_course_reference_valid)
    {
        _course_reference_rad = state.eta[5];
        _course_reference_valid = true;
    }
    _course_reference_rad = slew_angle(
        _course_reference_rad,
        raw_course_ref,
        _p.max_course_reference_rate_radps,
        control_dt);
    const double course_ref = _course_reference_rad;

    if (!_heading_hold_valid)
    {
        _heading_hold = state.eta[5];
        _heading_hold_valid = true;
    }

    const bool cruise_requested =
        !_transition_fault && _mode == GNCMode::WAYPOINT_3D && !_sp.hold_heading &&
        requested_speed >= _p.transition_command_min_speed_mps;
    switch (_flight_mode)
    {
    case FlightMode::Hover:
        if (cruise_requested && airspeed_valid)
        {
            _transition_entry_depth_m = state.depth_m;
            enter_flight_mode(FlightMode::FrontTransition, state);
        }
        break;
    case FlightMode::FrontTransition:
        if (_flight_mode_elapsed_s > 15.0 || state.depth_m > _transition_entry_depth_m + 3.0)
        {
            _transition_fault = true;
            enter_flight_mode(FlightMode::BackTransition, state);
        }
        else if (!cruise_requested || !airspeed_valid)
            enter_flight_mode(FlightMode::BackTransition, state);
        else if (_flight_mode_elapsed_s >= 0.5 &&
                 flight_speed >= std::min(
                     _p.transition_end_mps - 0.5,
                     std::max(_p.transition_start_mps + 0.5,
                              requested_speed - 0.5)))
            enter_flight_mode(FlightMode::Cruise, state);
        break;
    case FlightMode::Cruise:
        if (!cruise_requested || !airspeed_valid)
            enter_flight_mode(FlightMode::BackTransition, state);
        break;
    case FlightMode::BackTransition:
        // A deadline is an alarm, never proof that the aircraft has stopped.
        // Keep lift/braking active until the measured speed permits hover.
        if (_flight_mode_elapsed_s > 20.0) _transition_fault = true;
        if (cruise_requested && airspeed_valid)
            enter_flight_mode(FlightMode::FrontTransition, state);
        else if (_flight_mode_elapsed_s >= 0.5 &&
                 ground_speed <= _p.backtransition_complete_mps)
            enter_flight_mode(FlightMode::Hover, state);
        break;
    }

    const double yaw = state.eta[5];
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const Eigen::Vector2d position_error(dn, de);
    const Eigen::Vector2d velocity_xy = velocity_ned.head<2>();
    const double max_hover_accel = std::max(
        0.1, std::max(1.0, _p.g) * std::max(0.0, _p.max_tilt_rad));
    double hover_speed_limit = requested_speed > 0.1
                                   ? requested_speed
                                   : _p.hover_hold_max_speed_mps;
    if (_flight_mode == FlightMode::FrontTransition ||
        _flight_mode == FlightMode::Cruise)
    {
        hover_speed_limit = _p.hover_max_speed_mps;
    }
    hover_speed_limit = std::max(0.0, hover_speed_limit);
    Eigen::Vector2d velocity_target = Eigen::Vector2d::Zero();
    if (horizontal_distance > 1.0e-6 && hover_speed_limit > 1.0e-6)
    {
        const double velocity_gain =
            std::max(0.0, _p.xy_kp) / std::max(0.1, _p.xy_kd);
        const double position_speed = hover_speed_limit * std::tanh(
            velocity_gain * horizontal_distance / hover_speed_limit);
        const double stopping_speed = std::sqrt(
            2.0 * max_hover_accel * horizontal_distance);
        const double desired_speed = std::min(
            hover_speed_limit, std::min(position_speed, stopping_speed));
        velocity_target = desired_speed / horizontal_distance * position_error;
    }
    const Eigen::Vector2d accel_xy = clamp_norm(
        std::max(0.0, _p.xy_kd) * (velocity_target - velocity_xy),
        max_hover_accel);
    const double accel_n = accel_xy.x();
    const double accel_e = accel_xy.y();
    const double accel_forward = cy * accel_n + sy * accel_e;
    const double accel_right = -sy * accel_n + cy * accel_e;
    double hover_pitch_ref = clamp_abs(
        -accel_forward / std::max(1.0, _p.g), _p.max_tilt_rad);
    double hover_roll_ref = clamp_abs(
        accel_right / std::max(1.0, _p.g), _p.max_tilt_rad);
    if (_flight_mode == FlightMode::FrontTransition)
    {
        hover_pitch_ref = 0.0;
        hover_roll_ref = 0.0;
    }

    const double course_error = clamp_abs(
        wrap_pi(course_ref - actual_course),
        std::max(0.1, _p.cruise_l1_max_course_error_rad));
    const double damping = std::max(0.1, _p.cruise_l1_damping);
    const double l1_distance = std::max(
        std::max(_p.cruise_l1_min_distance_m, _sp.lookahead_m),
        damping * std::max(1.0, _p.cruise_l1_period_s) *
            std::max(ground_speed, _p.transition_start_mps) / kPi);
    const double l1_gain = 4.0 * damping * damping;
    const double guidance_speed = std::max(
        ground_speed, _p.transition_start_mps);
    const double lateral_accel =
        l1_gain * guidance_speed * guidance_speed /
        std::max(1.0, l1_distance) * std::sin(course_error);
    const double cruise_roll_ref = clamp_abs(
        std::atan2(lateral_accel, _p.g), _p.cruise_roll_limit_rad);
    const double d_error = depth_ref - state.eta[2];
    const double cruise_pitch_ref = clamp_abs(
        -_p.cruise_altitude_kp * d_error +
            _p.cruise_altitude_kd * velocity_ned.z(),
        _p.cruise_pitch_limit_rad);

    double wing_blend = 0.0;
    if (_flight_mode == FlightMode::FrontTransition)
    {
        wing_blend = smoothstep(
            (flight_speed - _p.transition_start_mps) /
            std::max(0.5, _p.transition_end_mps - _p.transition_start_mps));
    }
    else if (_flight_mode == FlightMode::Cruise)
    {
        wing_blend = 1.0;
    }

    const double roll_ref = lerp(hover_roll_ref, cruise_roll_ref, wing_blend);
    const double pitch_ref = lerp(hover_pitch_ref, cruise_pitch_ref, wing_blend);
    const bool wing_heading_control =
        _flight_mode == FlightMode::FrontTransition ||
        _flight_mode == FlightMode::Cruise;
    // A lift+cruise airframe may translate sideways in hover, but sustained
    // crab flight is a poor operational contract for a winged vehicle. Align
    // the nose with each active route leg in lift mode as well; only a
    // terminal hold preserves the captured arrival heading.
    const bool hover_route_heading_control =
        _flight_mode == FlightMode::Hover &&
        _mode == GNCMode::WAYPOINT_3D && !_sp.hold_heading &&
        horizontal_distance > 0.25;
    const double heading_ref = _sp.hold_heading
        ? _sp.heading_ref
        : (wing_heading_control || hover_route_heading_control
               ? course_ref
               : (_heading_hold_valid ? _heading_hold : state.eta[5]));

    const double down_accel = clamp_abs(
        _p.z_kp * d_error - _p.z_kd * velocity_ned.z(), 4.0);
    const double tilt_projection = std::max(
        0.70, std::cos(state.eta[3]) * std::cos(state.eta[4]));
    tau[2] = _p.mass * (_p.g - down_accel) / tilt_projection;
    tau[3] = clamp_abs(
        _p.roll_kp * (roll_ref - state.eta[3]) -
            _p.roll_kd * state.nu[3],
        std::max(0.0, _p.max_roll_moment_Nm));
    tau[4] = clamp_abs(
        _p.pitch_kp * (pitch_ref - state.eta[4]) -
            _p.pitch_kd * state.nu[4],
        std::max(0.0, _p.max_pitch_moment_Nm));
    tau[5] = _sp.use_yaw_rate_ref
        ? clamp_abs(_p.yaw_kd * (_sp.yaw_rate_ref - state.nu[5]),
                    std::max(0.0, _p.max_yaw_moment_Nm))
        : clamp_abs(
              _p.yaw_kp * wrap_pi(heading_ref - state.eta[5]) -
                  _p.yaw_kd * state.nu[5],
              std::max(0.0, _p.max_yaw_moment_Nm));

    if (wing_heading_control)
    {
        const double gate = std::max(0.1, _p.pusher_heading_gate_rad);
        const double heading_alignment = clamp01(
            (gate - std::abs(wrap_pi(course_ref - state.eta[5]))) / gate);
        // Gate acceleration while aligning for the front transition, but do
        // not remove the airspeed that provides control-surface authority in
        // wing-borne cruise. Course changes are handled by bank control.
        const double pusher_target =
            _flight_mode == FlightMode::FrontTransition
                ? requested_speed * heading_alignment
                : requested_speed;
        _pusher_speed_reference_mps += clamp_abs(
            pusher_target - _pusher_speed_reference_mps,
            std::max(0.0, _p.pusher_speed_ramp_mps2) * control_dt);
        const double forward_speed = flight_speed;
        tau[0] = std::max(0.0, std::min(
            std::max(0.0, _p.max_pusher_force_N),
            _p.pusher_trim_force_N +
                _p.pusher_drag_force_N_per_mps2 *
                    _pusher_speed_reference_mps * _pusher_speed_reference_mps +
                _p.pusher_speed_kp_N_per_mps *
                    (_pusher_speed_reference_mps - forward_speed)));
    }
    else
    {
        // Back transition is a safety-critical deceleration request: close
        // the pusher immediately and let lift-mode velocity feedback brake.
        _pusher_speed_reference_mps = 0.0;
        tau[0] = 0.0;
    }
    return tau;
}
}
