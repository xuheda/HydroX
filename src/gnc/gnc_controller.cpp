/** gnc_controller.cpp — SlenderBodyAUVController (cascade control, outputs generalized force tau) */
#include "gnc/gnc_controller.h"
#include <cmath>
#include <algorithm>

namespace hydrox
{
    static double clamp_abs(double v, double max_abs)
    {
        if (max_abs <= 0.0 || !std::isfinite(max_abs))
            return v;
        return std::max(-max_abs, std::min(max_abs, v));
    }

    static double turn_compensated_surge_ref(
        double surge_ref,
        bool use_yaw_rate_ref,
        double yaw_rate_ref,
        double yaw_rate_command_gain,
        const SlenderBodyAUVController::Params::TurnSpeedCompensation &p)
    {
        if (!use_yaw_rate_ref ||
            p.drop_gain_mps_per_radps <= 0.0 ||
            p.drop_max_mps <= 0.0 ||
            surge_ref <= 0.0)
        {
            return surge_ref;
        }

        const double commanded_yaw_rate = std::abs(yaw_rate_ref * yaw_rate_command_gain);
        const double excess = commanded_yaw_rate - std::max(0.0, p.drop_start_radps);
        if (excess <= 0.0)
            return surge_ref;

        const double drop = std::min(p.drop_max_mps, p.drop_gain_mps_per_radps * excess);
        return std::max(0.0, surge_ref - drop);
    }

    SlenderBodyAUVController::SlenderBodyAUVController(const Params &p)
        : _depth_pid(p.depth), _pitch_pid(p.pitch),
          _heading_smc(p.heading), _surge_p(p.surge),
          _yaw_rate_p(p.yaw_rate), _turn_speed_p(p.turn_speed) {}

    double SlenderBodyAUVController::shaped_yaw_rate_ref(double raw_ref, double dt)
    {
        if (!_yaw_rate_ref_initialized || dt <= 0.0 || !std::isfinite(dt))
        {
            _yaw_rate_ref_filtered = raw_ref;
            _yaw_rate_ref_initialized = true;
            return _yaw_rate_ref_filtered;
        }

        double target = raw_ref;
        if (_yaw_rate_p.ref_filter_tau > 1.0e-6)
        {
            const double alpha = std::min(1.0, std::max(0.0, dt / (_yaw_rate_p.ref_filter_tau + dt)));
            target = _yaw_rate_ref_filtered + alpha * (raw_ref - _yaw_rate_ref_filtered);
        }

        if (_yaw_rate_p.ref_slew_limit > 1.0e-6)
        {
            const double max_delta = _yaw_rate_p.ref_slew_limit * dt;
            target = _yaw_rate_ref_filtered +
                     clamp_abs(target - _yaw_rate_ref_filtered, max_delta);
        }

        _yaw_rate_ref_filtered = target;
        return _yaw_rate_ref_filtered;
    }

    void SlenderBodyAUVController::reset(const NavigationState &state)
    {
        _depth_pid.reset(state.depth_m);
        _pitch_pid.reset(state.eta[4]);
        _heading_smc.reset(state.eta[5]);
        _surge_p.reset();
        _yaw_rate_i = 0.0;
        _yaw_rate_ref_filtered = 0.0;
        _yaw_rate_ref_initialized = false;
    }

    Wrench SlenderBodyAUVController::update(const NavigationState &state, double dt)
    {
        Wrench tau;
        tau.setZero();

        if (_mode == GNCMode::DISABLED)
            return tau; // Zero generalized force

        if (_mode == GNCMode::SURFACE)
        {
            // Keep depth/pitch PID reset while at surface so integral is clean on re-dive.
            _depth_pid.reset(state.depth_m);
            _pitch_pid.reset(state.eta[4]);
            double heading = state.eta[5];
            double yaw_rate = state.nu[5];
            double surge = state.nu[0];
            if (_sp.use_yaw_rate_ref)
            {
                // Yaw-rate tracking: feed-forward supplies the steady turn moment,
                // feedback damps tracking error without inflating the command.
                const double yaw_rate_ref = shaped_yaw_rate_ref(
                    _sp.yaw_rate_ref * _yaw_rate_p.command_gain, dt);
                const double yr_err = yaw_rate_ref - yaw_rate;
                _yaw_rate_i += yr_err * dt;
                _yaw_rate_i = std::max(-_yaw_rate_p.ei_max, std::min(_yaw_rate_p.ei_max, _yaw_rate_i));
                tau[5] = _yaw_rate_p.feed_forward * yaw_rate_ref +
                         _yaw_rate_p.kp * yr_err +
                         _yaw_rate_p.ki * _yaw_rate_i -
                         _yaw_rate_p.kd * yaw_rate;
                tau[5] = clamp_abs(tau[5], _yaw_rate_p.tau_max);
            }
            else
                tau[5] = _heading_smc.step(heading, yaw_rate, _sp.heading_ref, dt);
            const double surge_ref_for_control = turn_compensated_surge_ref(
                _sp.surge_ref,
                _sp.use_yaw_rate_ref,
                _sp.yaw_rate_ref,
                _yaw_rate_p.command_gain,
                _turn_speed_p);
            tau[0] = _surge_p.step(surge, surge_ref_for_control, dt);
            return tau;
        }

        double depth_m = state.depth_m;
        double heave_rate = state.nu[2];
        double heading = state.eta[5];
        double yaw_rate = state.nu[5];
        double surge = state.nu[0];
        double pitch = state.eta[4];
        double pitch_rate = state.nu[4];

        // ---- Cascade Depth Control ----
        // Outer loop: depth -> desired pitch angle
        double theta_ref = _depth_pid.step(depth_m, heave_rate, _sp.depth_ref, dt);

        // Near-surface fade: prevents forced diving when near the surface, allowing the AUV to float up naturally
        constexpr double kSurfaceZone = 1.5; // m — fade starts below this depth
        if (depth_m < kSurfaceZone)
        {
            // Clamp to [0,1]: prevents negative fade when AUV breaches the surface
            double fade = std::max(0.0, depth_m / kSurfaceZone);
            theta_ref *= fade;
        }

        // Inner loop: pitch -> pitch torque
        double tau_M = _pitch_pid.step(pitch, pitch_rate, theta_ref, dt);

        // Torpedo-type AUV does not directly control heave; it relies on pitch + velocity to generate lift.
        // Keep the external surge_ref unchanged for policy/env parity, but bias the inner surge loop during
        // hard yaw-rate turns so the achieved speed stays near the point-mass env speed limit.
        const double surge_ref_for_control = turn_compensated_surge_ref(
            _sp.surge_ref,
            _sp.use_yaw_rate_ref,
            _sp.yaw_rate_ref,
            _yaw_rate_p.command_gain,
            _turn_speed_p);
        double tau_X = _surge_p.step(surge, surge_ref_for_control, dt);
        // Turn-drag feedforward: a slender AUV bleeds speed in the policy's hard
        // turns (induced/sideslip drag the point-mass training env doesn't model).
        // Add thrust ~ |yaw_rate|*surge^2 so the vehicle holds its commanded speed
        // THROUGH turns, matching the env the policy was trained on. It is GATED by
        // yaw_rate — zero in straight runs — so it adds nothing at startup/cruise
        // (no integral windup, no fly-off, unlike enlarging the surge integral
        // clamp) and is bounded by the allocator's propeller cap in turns.
        if (_sp.use_yaw_rate_ref && surge > 0.2)
        {
            // Scale by surge_ref^2 (the COMMANDED speed), not actual surge^2 — the
            // latter is a positive feedback (faster -> more FF -> faster) that ran
            // the surge away to 5-6 m/s. surge_ref is constant per tick, so the FF
            // compensates the turn drag expected AT the target speed without self-
            // exciting; the surge PI still trims around it.
            constexpr double kTurnDragFF = 0.0;  // 2026-06-15: FF disabled. It overspeed the AUV to ~4.6 m/s -> velocity obs ~1.44 (out of [0,1]) -> policy OOD/thrash. The time-step alignment (loop_dt=model_dt/command_scale) is the real fix; the surge controller alone tracks surge_ref and keeps speed in-distribution.
            const double v_ref = _sp.surge_ref;
            double turn_ff = kTurnDragFF * std::fabs(yaw_rate) * v_ref * v_ref;
            if (turn_ff > 3000.0)
                turn_ff = 3000.0;
            tau_X += turn_ff;
        }
        double tau_N = 0.0;
        if (_sp.use_yaw_rate_ref)
        {
            // Yaw-rate tracking: the policy command is angular velocity. Feed-forward
            // supplies the steady turning moment, while feedback damps error without
            // turning a 30 deg/s command into a larger internal target.
            const double yaw_rate_ref = shaped_yaw_rate_ref(
                _sp.yaw_rate_ref * _yaw_rate_p.command_gain, dt);
            const double yr_err = yaw_rate_ref - yaw_rate;
            _yaw_rate_i += yr_err * dt;
            _yaw_rate_i = std::max(-_yaw_rate_p.ei_max, std::min(_yaw_rate_p.ei_max, _yaw_rate_i));
            tau_N = _yaw_rate_p.feed_forward * yaw_rate_ref +
                    _yaw_rate_p.kp * yr_err +
                    _yaw_rate_p.ki * _yaw_rate_i -
                    _yaw_rate_p.kd * yaw_rate;
            tau_N = clamp_abs(tau_N, _yaw_rate_p.tau_max);
        }
        else
        {
            tau_N = _heading_smc.step(heading, yaw_rate, _sp.heading_ref, dt);
        }

        tau[0] = tau_X;
        // tau_Z = 0: slender-body AUV changes depth via pitch attitude, not direct heave force
        tau[4] = tau_M;
        tau[5] = tau_N;

        return tau;
    }

} // namespace hydrox
