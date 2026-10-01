// Copyright (c) 2026 OceanX. Author: xuheda
#include "gnc/surface_controller.h"
#include "gnc/surface_control_demand.h"

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

        SurfaceVesselGuidance::Params guidance_params(
            const SurfaceVesselController::Params &params)
        {
            SurfaceVesselGuidance::Params guidance;
            guidance.waypoint_surge_mps = params.waypoint_surge_mps;
            guidance.station_keep_surge_mps = params.station_keep_surge_mps;
            guidance.station_keep_hysteresis_m = params.station_keep_hysteresis_m;
            guidance.los_lookahead_m = params.los_lookahead_m;
            guidance.ilos_integral_gain = params.ilos_integral_gain;
            guidance.ilos_integral_limit = params.ilos_integral_limit;
            guidance.sideslip_compensation_gain = params.sideslip_compensation_gain;
            guidance.sideslip_filter_time_constant_s =
                params.sideslip_filter_time_constant_s;
            guidance.max_crab_angle_rad = params.max_crab_angle_rad;
            guidance.max_accel_mps2 = params.max_accel_mps2;
            guidance.max_decel_mps2 = params.max_decel_mps2;
            return guidance;
        }
    } // namespace

    SurfaceVesselController::SurfaceVesselController(const Params &p)
        : _p(p), _guidance(guidance_params(p))
    {}

    void SurfaceVesselController::reset(const NavigationState &state)
    {
        _surge_error_integral = 0.0;
        _last_surge_mps = state.nu[0];
        _filtered_surge_accel_mps2 = 0.0;
        _surge_derivative_initialized = true;
        _guidance.reset(state);
    }

    Wrench SurfaceVesselController::update(const NavigationState &state, double dt)
    {
        SurfaceControlDemand demand;
        if (_mode == GNCMode::DISABLED)
            return demand.to_wrench();

        const double control_dt = std::max(0.0, std::min(dt, 0.2));
        const SurfaceGuidanceCommand guidance = _guidance.update(state, _sp, _mode, control_dt);
        const double heading_ref = guidance.heading_ref_rad;
        const double surge_ref = guidance.surge_ref_mps;
        const double u = state.nu[0];
        const double r = state.nu[5];
        if (!_surge_derivative_initialized)
        {
            _last_surge_mps = u;
            _filtered_surge_accel_mps2 = 0.0;
            _surge_derivative_initialized = true;
        }
        else if (control_dt > 1.0e-6)
        {
            const double measured_accel = (u - _last_surge_mps) / control_dt;
            const double filter_tau = std::max(
                1.0e-3, _p.surge_accel_filter_tau_s);
            const double alpha = 1.0 - std::exp(-control_dt / filter_tau);
            _filtered_surge_accel_mps2 +=
                (measured_accel - _filtered_surge_accel_mps2) * alpha;
            _last_surge_mps = u;
        }
        const double yaw_err = wrap_pi(heading_ref - state.eta[5]);
        const double surge_error = surge_ref - u;
        const double feed_forward =
            _p.surge_drag_linear_N_per_mps * surge_ref +
            _p.surge_drag_quadratic_N_per_mps2 *
                std::abs(surge_ref) * surge_ref;
        const double integral_limit = std::max(0.0, _p.surge_integral_limit);
        const double proposed_integral = clamp_abs(
            _surge_error_integral + surge_error * control_dt,
            integral_limit);
        const double raw_feedback =
            _p.surge_kp * surge_error +
            _p.surge_ki * proposed_integral -
            _p.surge_kd * _filtered_surge_accel_mps2;
        const double feedback_limit = std::max(
            0.0, _p.surge_feedback_force_limit_N);
        const double limited_feedback = feedback_limit > 0.0
            ? clamp_abs(raw_feedback, feedback_limit)
            : raw_feedback;
        const double forward_limit = std::max(0.0, _p.max_force_N);
        const double brake_limit = _p.max_brake_force_N > 0.0
            ? _p.max_brake_force_N
            : 0.35 * forward_limit;
        const double proposed_force = feed_forward + limited_feedback;
        const double saturated_force = std::max(
            -brake_limit, std::min(forward_limit, proposed_force));
        const bool feedback_can_integrate = feedback_limit <= 0.0 ||
            raw_feedback == limited_feedback ||
            (raw_feedback > feedback_limit && surge_error < 0.0) ||
            (raw_feedback < -feedback_limit && surge_error > 0.0);
        const bool actuator_can_integrate =
            proposed_force == saturated_force ||
            (proposed_force > forward_limit && surge_error < 0.0) ||
            (proposed_force < -brake_limit && surge_error > 0.0);
        if (feedback_can_integrate && actuator_can_integrate)
            _surge_error_integral = proposed_integral;
        const double feedback =
            _p.surge_kp * surge_error +
            _p.surge_ki * _surge_error_integral -
            _p.surge_kd * _filtered_surge_accel_mps2;
        const double bounded_feedback = feedback_limit > 0.0
            ? clamp_abs(feedback, feedback_limit)
            : feedback;
        demand.surge_force_N = std::max(
            -brake_limit,
            std::min(
                forward_limit,
                feed_forward + bounded_feedback));
        if (std::abs(surge_ref) < 0.01 && std::abs(u) < 0.03)
        {
            _surge_error_integral = 0.0;
            demand.surge_force_N = 0.0;
        }

        // Factor the former heading PD into a heading outer loop and yaw-rate
        // inner loop. With no yaw-rate saturation this is algebraically the
        // same controller, while the explicit rate limit prevents snap turns.
        double yaw_rate_ref = _sp.yaw_rate_ref;
        if (!_sp.use_yaw_rate_ref)
        {
            const double rate_gain = _p.yaw_kd > 1.0e-6
                ? _p.yaw_kp / _p.yaw_kd
                : 0.0;
            yaw_rate_ref = clamp_abs(
                rate_gain * yaw_err,
                std::max(0.0, _p.max_yaw_rate_radps));
        }
        demand.yaw_moment_Nm = _p.yaw_kd > 1.0e-6
            ? clamp_abs(
                _p.yaw_kd * (yaw_rate_ref - r),
                _p.max_moment_Nm)
            : clamp_abs(_p.yaw_kp * yaw_err, _p.max_moment_Nm);

        return demand.to_wrench();
    }
} // namespace hydrox
