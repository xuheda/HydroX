// Copyright (c) 2026 OceanX. Author: xuheda
#include "gnc/fixedwing_controller.h"

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

        double wrap_pi(double a)
        {
            while (a > kPi)
                a -= 2.0 * kPi;
            while (a < -kPi)
                a += 2.0 * kPi;
            return a;
        }

        double slew(double current, double target, double rate_limit, double dt)
        {
            if (!(dt > 0.0) || !(rate_limit > 0.0))
                return target;
            return current + clamp_abs(target - current, rate_limit * dt);
        }

        Eigen::Vector3d ned_velocity(const NavigationState &state)
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
    } // namespace

    FixedWingController::FixedWingController(const Params &p) : _p(p) {}

    void FixedWingController::reset(const NavigationState &state)
    {
        _altitude_error_integral = 0.0;
        _pitch_error_integral = 0.0;
        _roll_reference_rad = state.eta[3];
        _pitch_reference_rad = state.eta[4];
        _depth_reference_elapsed_s = 0.0;
        _depth_reference_rate_observation_mps = 0.0;
        _filtered_depth_reference_rate_mps = 0.0;
        _references_initialized = true;
        _depth_reference_initialized = false;
    }

    void FixedWingController::set_mode(GNCMode mode)
    {
        if (mode != _mode)
        {
            _altitude_error_integral = 0.0;
            _pitch_error_integral = 0.0;
            _depth_reference_elapsed_s = 0.0;
            _depth_reference_rate_observation_mps = 0.0;
            _filtered_depth_reference_rate_mps = 0.0;
            _references_initialized = false;
            _depth_reference_initialized = false;
        }
        _mode = mode;
    }

    Wrench FixedWingController::update(const NavigationState &state, double dt)
    {
        Wrench tau;
        tau.setZero();
        if (_mode == GNCMode::DISABLED)
            return tau;

        double d_ref = _sp.depth_ref;
        double course_ref = _sp.heading_ref;
        const double requested_speed =
            (_sp.surge_ref > 0.1) ? _sp.surge_ref : _p.cruise_speed_mps;
        const double speed_ref = std::max(_p.min_flight_speed_mps, requested_speed);
        const double control_dt = std::isfinite(dt) && dt > 0.0
                                      ? std::min(dt, 0.1)
                                      : 0.0;
        const Eigen::Vector3d velocity_ned = ned_velocity(state);
        const double ground_speed = velocity_ned.head<2>().norm();
        // Explicit degraded mode: retain trim speed, never label GPS ground speed as airspeed.
        const bool airspeed_valid = state.airspeed_valid &&
            std::isfinite(state.equivalent_airspeed_mps) && state.equivalent_airspeed_mps >= 0.0;
        const double flight_speed = airspeed_valid ? state.equivalent_airspeed_mps : _p.cruise_speed_mps;
        const double actual_course = ground_speed > 1.0
                                         ? std::atan2(velocity_ned.y(), velocity_ned.x())
                                         : state.eta[5];

        if (_mode == GNCMode::WAYPOINT_3D)
        {
            d_ref = _sp.wp_d;
            if (!_sp.hold_heading && !_sp.use_path_segment)
                course_ref = std::atan2(_sp.wp_e - state.eta[1],
                                        _sp.wp_n - state.eta[0]);
        }

        // L1 guidance aims at a point one speed-scaled look-ahead distance
        // ahead of the aircraft's along-track projection. This responds to
        // cross-track position and velocity direction in one damped law,
        // unlike a raw heading-error gain that changes character with speed.
        const double damping = std::max(0.1, _p.l1_damping);
        const double l1_distance = std::max(
            std::max(_p.l1_min_distance_m, _sp.lookahead_m),
            damping * std::max(1.0, _p.l1_period_s) *
                std::max(ground_speed, _p.min_flight_speed_mps) / kPi);
        if (_mode == GNCMode::WAYPOINT_3D &&
            _sp.use_path_segment && !_sp.hold_heading)
        {
            const double segment_n = _sp.wp_n - _sp.path_start_n;
            const double segment_e = _sp.wp_e - _sp.path_start_e;
            const double segment_length = std::hypot(segment_n, segment_e);
            if (segment_length > 0.25)
            {
                const double unit_n = segment_n / segment_length;
                const double unit_e = segment_e / segment_length;
                const double rel_n = state.eta[0] - _sp.path_start_n;
                const double rel_e = state.eta[1] - _sp.path_start_e;
                const double along_track = rel_n * unit_n + rel_e * unit_e;
                const double target_n =
                    _sp.path_start_n + (along_track + l1_distance) * unit_n;
                const double target_e =
                    _sp.path_start_e + (along_track + l1_distance) * unit_e;
                course_ref = std::atan2(target_e - state.eta[1],
                                        target_n - state.eta[0]);
            }
        }

        const double course_error = clamp_abs(
            wrap_pi(course_ref - actual_course),
            std::max(0.1, _p.l1_max_course_error_rad));
        const double l1_gain = 4.0 * damping * damping;
        const double guidance_speed =
            std::max(ground_speed, _p.min_flight_speed_mps);
        const double lateral_accel =
            l1_gain * guidance_speed * guidance_speed /
            std::max(1.0, l1_distance) * std::sin(course_error);
        const double roll_target = clamp_abs(
            std::atan2(lateral_accel, 9.80665), _p.roll_limit_rad);

        // Infer the velocity of the planner's moving depth reference.  The
        // command may be held for several controller cycles, so measure over
        // the elapsed time between actual reference changes.  A stale command
        // decays to zero and cannot leave a permanent climb/descent demand.
        if (!_depth_reference_initialized)
        {
            _last_depth_reference_m = d_ref;
            _depth_reference_elapsed_s = 0.0;
            _depth_reference_rate_observation_mps = 0.0;
            _filtered_depth_reference_rate_mps = 0.0;
            _depth_reference_initialized = true;
        }
        else if (control_dt > 0.0)
        {
            _depth_reference_elapsed_s += control_dt;
            const double reference_delta = d_ref - _last_depth_reference_m;
            if (std::abs(reference_delta) > 1.0e-5)
            {
                _depth_reference_rate_observation_mps = clamp_abs(
                    reference_delta / std::max(control_dt, _depth_reference_elapsed_s),
                    std::max(0.0, _p.max_depth_reference_rate_mps));
                _last_depth_reference_m = d_ref;
                _depth_reference_elapsed_s = 0.0;
            }
            else if (_depth_reference_elapsed_s >
                     std::max(control_dt, _p.depth_reference_rate_stale_s))
            {
                _depth_reference_rate_observation_mps = 0.0;
            }
            const double rate_alpha = _p.depth_reference_rate_filter_tau_s > 0.0
                                          ? control_dt / (_p.depth_reference_rate_filter_tau_s + control_dt)
                                          : 1.0;
            _filtered_depth_reference_rate_mps += rate_alpha *
                (_depth_reference_rate_observation_mps - _filtered_depth_reference_rate_mps);
        }

        const double d_err = d_ref - state.eta[2];
        double integral_candidate = _altitude_error_integral;
        const bool underspeed = flight_speed < _p.min_flight_speed_mps;
        if (control_dt > 0.0 && !underspeed)
            integral_candidate = clamp_abs(
                integral_candidate + d_err * control_dt,
                std::max(0.0, _p.altitude_integral_limit));
        const double desired_down_velocity =
            _filtered_depth_reference_rate_mps;
        const double flight_path_feedforward = -std::atan2(
            desired_down_velocity,
            std::max(flight_speed, _p.min_flight_speed_mps));
        auto pitch_from_integral = [&](double integral)
        {
            double result = -_p.altitude_kp * d_err -
                            _p.altitude_ki * integral +
                            _p.altitude_kd *
                                (velocity_ned.z() - desired_down_velocity) +
                            _p.flight_path_feedforward_gain *
                                flight_path_feedforward;
            if (underspeed)
                result -= _p.underspeed_pitch_gain *
                          (_p.min_flight_speed_mps - flight_speed);
            else if (flight_speed > _p.max_flight_speed_mps)
                result += _p.overspeed_pitch_gain *
                          (flight_speed - _p.max_flight_speed_mps);
            return result;
        };
        double pitch_unbounded = pitch_from_integral(integral_candidate);
        const bool altitude_winds_into_limit =
            (pitch_unbounded > _p.pitch_limit_rad && d_err < 0.0) ||
            (pitch_unbounded < -_p.pitch_limit_rad && d_err > 0.0);
        if (!underspeed && !altitude_winds_into_limit)
            _altitude_error_integral = integral_candidate;
        else
            pitch_unbounded = pitch_from_integral(_altitude_error_integral);
        const double pitch_target =
            clamp_abs(pitch_unbounded, _p.pitch_limit_rad);

        if (!_references_initialized)
        {
            _roll_reference_rad = state.eta[3];
            _pitch_reference_rad = state.eta[4];
            _references_initialized = true;
        }
        _roll_reference_rad = slew(
            _roll_reference_rad, roll_target,
            _p.max_roll_reference_rate_radps, control_dt);
        _pitch_reference_rad = slew(
            _pitch_reference_rad, pitch_target,
            _p.max_pitch_reference_rate_radps, control_dt);

        // Rate-damped inner loops produce angular accelerations, which are
        // converted here to physical body moments before allocation.
        const double roll_accel = clamp_abs(
            _p.roll_attitude_kp * (_roll_reference_rad - state.eta[3]) -
                _p.roll_rate_kd * state.nu[3],
            _p.max_roll_accel_radps2);
        const double pitch_error =
            _pitch_reference_rad - state.eta[4];
        double pitch_integral_candidate = _pitch_error_integral;
        if (control_dt > 0.0 && !underspeed)
            pitch_integral_candidate = clamp_abs(
                pitch_integral_candidate + pitch_error * control_dt,
                std::max(0.0, _p.pitch_error_integral_limit));
        const double pitch_inertia = std::max(std::abs(_p.Iyy_kgm2), 1.0e-6);
        auto pitch_accel_from_integral = [&](double integral)
        {
            return _p.pitch_trim_moment_Nm / pitch_inertia +
                   _p.pitch_attitude_kp * pitch_error +
                   _p.pitch_attitude_ki * integral -
                   _p.pitch_rate_kd * state.nu[4];
        };
        double pitch_accel_unbounded =
            pitch_accel_from_integral(pitch_integral_candidate);
        const bool pitch_winds_into_limit =
            (pitch_accel_unbounded > _p.max_pitch_accel_radps2 && pitch_error > 0.0) ||
            (pitch_accel_unbounded < -_p.max_pitch_accel_radps2 && pitch_error < 0.0);
        if (!underspeed && !pitch_winds_into_limit)
            _pitch_error_integral = pitch_integral_candidate;
        else
            pitch_accel_unbounded =
                pitch_accel_from_integral(_pitch_error_integral);
        const double pitch_accel = clamp_abs(
            pitch_accel_unbounded,
            _p.max_pitch_accel_radps2);

        const double forward_speed = airspeed_valid ? flight_speed : speed_ref;
        tau[0] = std::max(0.0, std::min(
            std::max(0.0, _p.max_forward_force_N),
            _p.cruise_force_N +
                _p.speed_force_kp_N_per_mps * (speed_ref - forward_speed)));
        tau[3] = std::max(std::abs(_p.Ixx_kgm2), 1.0e-6) * roll_accel;
        tau[4] = pitch_inertia * pitch_accel;
        const double coordinated_yaw_rate = clamp_abs(
            9.80665 * std::tan(_roll_reference_rad) /
                std::max(flight_speed, _p.min_flight_speed_mps),
            _p.max_coordinated_yaw_rate_radps);
        const double yaw_rate_ref = _sp.use_yaw_rate_ref
                                        ? _sp.yaw_rate_ref
                                        : coordinated_yaw_rate;
        const double yaw_accel = clamp_abs(
            _p.yaw_rate_kp_radps2_per_radps * (yaw_rate_ref - state.nu[5]),
            _p.max_yaw_accel_radps2);
        tau[5] = std::max(std::abs(_p.Izz_kgm2), 1.0e-6) * yaw_accel;
        return tau;
    }
} // namespace hydrox
