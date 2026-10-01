// Copyright (c) 2026 OceanX. Author: xuheda
#include "gnc/multirotor_controller.h"

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

        Eigen::Vector2d clamp_norm(const Eigen::Vector2d &value, double limit)
        {
            const double safe_limit = std::max(0.0, limit);
            const double norm = value.norm();
            if (norm > safe_limit && norm > 1.0e-9)
                return value * (safe_limit / norm);
            return value;
        }
    } // namespace

    MultirotorController::MultirotorController(const Params &p) : _p(p) {}

    void MultirotorController::reset(const NavigationState &state)
    {
        _heading_hold = state.eta[5];
        _heading_hold_valid = true;
        const Eigen::Vector3d velocity = ned_velocity(state);
        _velocity_ref_ned = velocity.head<2>();
        _last_velocity_ned = velocity.head<2>();
        _acceleration_ref_ned.setZero();
        _filtered_accel_ned.setZero();
        _velocity_error_integral.setZero();
        _z_error_integral = 0.0;
        _translation_initialized = true;
    }

    void MultirotorController::set_mode(GNCMode mode)
    {
        if (mode != _mode)
        {
            _velocity_error_integral.setZero();
            _acceleration_ref_ned.setZero();
            _z_error_integral = 0.0;
            _translation_initialized = false;
        }
        _mode = mode;
    }

    Wrench MultirotorController::update(const NavigationState &state, double dt)
    {
        Wrench tau;
        tau.setZero();
        if (_mode == GNCMode::DISABLED)
            return tau;

        double d_ref = _sp.depth_ref;
        double heading_ref = _sp.heading_ref;
        double roll_ref = 0.0;
        double pitch_ref = 0.0;
        const double control_dt = std::isfinite(dt) && dt > 0.0
                                      ? std::min(dt, 0.1)
                                      : 0.0;
        const Eigen::Vector3d velocity_ned = ned_velocity(state);
        const Eigen::Vector2d velocity_xy = velocity_ned.head<2>();
        const double yaw = state.eta[5];
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);
        if (_mode == GNCMode::WAYPOINT_3D)
        {
            d_ref = _sp.wp_d;
            // A multirotor translates laterally without yawing. Hold the
            // entry heading so crossing a waypoint cannot inject a 180-degree
            // yaw transient into the horizontal attitude loops.
            heading_ref = _heading_hold_valid ? _heading_hold : state.eta[5];

            if (!_translation_initialized)
            {
                _velocity_ref_ned = velocity_xy;
                _last_velocity_ned = velocity_xy;
                _acceleration_ref_ned.setZero();
                _filtered_accel_ned.setZero();
                _velocity_error_integral.setZero();
                _translation_initialized = true;
            }

            if (control_dt > 0.0)
            {
                const Eigen::Vector2d measured_accel =
                    (velocity_xy - _last_velocity_ned) / control_dt;
                const double filter_tau = std::max(
                    1.0e-3, _p.measured_accel_filter_tau_s);
                const double alpha = control_dt / (filter_tau + control_dt);
                _filtered_accel_ned +=
                    alpha * (measured_accel - _filtered_accel_ned);
            }
            _last_velocity_ned = velocity_xy;

            const double dn = _sp.wp_n - state.eta[0];
            const double de = _sp.wp_e - state.eta[1];
            const double distance = std::hypot(dn, de);
            Eigen::Vector2d velocity_target = Eigen::Vector2d::Zero();
            if (distance > 1.0e-6)
            {
                const double speed_limit = _sp.surge_ref > 0.05
                                               ? _sp.surge_ref
                                               : _p.hold_max_speed_mps;
                const double safe_speed_limit = std::max(0.0, speed_limit);
                const double position_speed = safe_speed_limit > 1.0e-6
                    ? safe_speed_limit * std::tanh(
                          std::max(0.0, _p.xy_kp) * distance /
                          safe_speed_limit)
                    : 0.0;
                const double stopping_speed = std::sqrt(
                    2.0 * std::max(0.05, _p.max_xy_brake_accel) * distance);
                const double desired_speed = std::min(
                    safe_speed_limit,
                    std::min(position_speed, stopping_speed));
                velocity_target = desired_speed / distance *
                    Eigen::Vector2d(dn, de);
            }

            if (control_dt > 0.0)
            {
                const double reference_tau = std::max(
                    0.02, _p.velocity_reference_tau_s);
                Eigen::Vector2d target_reference_accel =
                    (velocity_target - _velocity_ref_ned) / reference_tau;
                target_reference_accel = clamp_norm(
                    target_reference_accel,
                    std::max(_p.max_xy_accel, _p.max_xy_brake_accel));
                Eigen::Vector2d accel_delta =
                    target_reference_accel - _acceleration_ref_ned;
                accel_delta = clamp_norm(
                    accel_delta,
                    std::max(0.0, _p.max_xy_jerk_mps3) * control_dt);
                _acceleration_ref_ned += accel_delta;
                const Eigen::Vector2d previous_velocity_ref = _velocity_ref_ned;
                _velocity_ref_ned += _acceleration_ref_ned * control_dt;
                if ((velocity_target - previous_velocity_ref).dot(
                        velocity_target - _velocity_ref_ned) <= 0.0)
                {
                    _velocity_ref_ned = velocity_target;
                    _acceleration_ref_ned.setZero();
                }
            }

            const Eigen::Vector2d velocity_error =
                _velocity_ref_ned - velocity_xy;
            Eigen::Vector2d integral_candidate = _velocity_error_integral;
            if (control_dt > 0.0 && _p.xy_velocity_ki > 0.0)
                integral_candidate += velocity_error * control_dt;
            if (_p.xy_velocity_ki > 0.0)
            {
                integral_candidate = clamp_norm(
                    integral_candidate,
                    _p.xy_integral_accel_limit / _p.xy_velocity_ki);
            }
            else
            {
                integral_candidate.setZero();
            }

            const Eigen::Vector2d accel_unbounded =
                _acceleration_ref_ned + _p.xy_kd * velocity_error +
                _p.xy_velocity_ki * integral_candidate -
                _p.xy_accel_damping * _filtered_accel_ned;
            const bool braking = accel_unbounded.dot(velocity_xy) < 0.0;
            const double accel_limit = braking
                                           ? std::max(_p.max_xy_accel,
                                                      _p.max_xy_brake_accel)
                                           : _p.max_xy_accel;
            const Eigen::Vector2d acceleration_cmd =
                clamp_norm(accel_unbounded, accel_limit);
            if (_p.xy_velocity_ki > 0.0 && control_dt > 0.0)
            {
                integral_candidate +=
                    _p.xy_anti_windup_gain *
                    (acceleration_cmd - accel_unbounded) /
                    _p.xy_velocity_ki * control_dt;
                _velocity_error_integral = clamp_norm(
                    integral_candidate,
                    _p.xy_integral_accel_limit / _p.xy_velocity_ki);
            }

            const double acc_n = acceleration_cmd.x();
            const double acc_e = acceleration_cmd.y();
            const double acc_forward = cy * acc_n + sy * acc_e;
            const double acc_right = -sy * acc_n + cy * acc_e;
            pitch_ref = clamp_abs(-acc_forward / _p.g, _p.max_tilt_rad);
            roll_ref = clamp_abs(acc_right / _p.g, _p.max_tilt_rad);
        }

        // Depth lives in map_ned, so its derivative must be the map-frame
        // down velocity. BodyFRD w also contains the projection of horizontal
        // motion whenever the aircraft is tilted and destabilizes altitude if
        // it is used directly here.
        const double roll = state.eta[3];
        const double pitch = state.eta[4];
        const double cr = std::cos(roll);
        const double cp = std::cos(pitch);
        const double vel_d = velocity_ned.z();
        const double z_err = d_ref - state.eta[2];
        double z_integral_candidate = _z_error_integral;
        if (control_dt > 0.0 && _p.z_ki > 0.0)
        {
            z_integral_candidate += z_err * control_dt;
            z_integral_candidate = clamp_abs(
                z_integral_candidate,
                _p.z_integral_accel_limit / _p.z_ki);
        }
        const double z_accel_unbounded =
            _p.z_kp * z_err - _p.z_kd * vel_d +
            _p.z_ki * z_integral_candidate;
        const double z_accel_down = clamp_abs(
            z_accel_unbounded, _p.max_z_accel);
        if (_p.z_ki > 0.0 &&
            (z_accel_down == z_accel_unbounded ||
             z_err * z_accel_unbounded < 0.0))
        {
            _z_error_integral = z_integral_candidate;
        }

        // Positive tau[2] is interpreted by the multirotor allocator as upward
        // collective thrust. In NED, negative z_accel_down means climb. Divide
        // by the vertical projection so commanded NED acceleration is retained
        // while the aircraft is tilted for horizontal translation.
        const double vertical_projection = std::max(0.5, cr * cp);
        tau[2] = _p.mass * (_p.g - z_accel_down) / vertical_projection;

        const double yaw_err = wrap_pi(heading_ref - yaw);
        const double roll_accel = clamp_abs(
            _p.roll_kp * (roll_ref - state.eta[3]) -
                _p.roll_kd * state.nu[3],
            _p.max_roll_accel_radps2);
        const double pitch_accel = clamp_abs(
            _p.pitch_kp * (pitch_ref - state.eta[4]) -
                _p.pitch_kd * state.nu[4],
            _p.max_pitch_accel_radps2);
        tau[3] = std::max(_p.Ixx, 1.0e-6) * roll_accel;
        tau[4] = std::max(_p.Iyy, 1.0e-6) * pitch_accel;
        if (_p.rotor_thrust_coefficient > 0.0 &&
            _p.rotor_rolling_moment_coefficient > 0.0 && tau[2] > 0.0)
        {
            // The imported motor model applies
            //   -abs(omega) * rolling_coefficient * velocity_perpendicular
            // at every lift rotor. Estimate the mean rotor speed from the
            // requested collective and cancel that deterministic moment. The
            // attitude loop then does not need a persistent several-degree
            // error merely to fly a straight line.
            const double mean_rotor_omega = std::sqrt(
                tau[2] / (4.0 * _p.rotor_thrust_coefficient));
            const double rolling_moment_gain =
                4.0 * mean_rotor_omega *
                _p.rotor_rolling_moment_coefficient;
            tau[3] += rolling_moment_gain * state.nu[0];
            tau[4] += rolling_moment_gain * state.nu[1];
        }

        double yaw_accel = 0.0;
        if (_sp.use_yaw_rate_ref)
            yaw_accel = _p.yaw_kd * (_sp.yaw_rate_ref - state.nu[5]);
        else
            yaw_accel = _p.yaw_kp * yaw_err - _p.yaw_kd * state.nu[5];
        tau[5] = std::max(_p.Izz, 1.0e-6) * clamp_abs(
            yaw_accel, _p.max_yaw_accel_radps2);

        return tau;
    }
} // namespace hydrox
