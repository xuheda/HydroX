#pragma once
#include "hydrox/default_parameter.h"
/** surge_p.h — Surge velocity P controller */
#include <algorithm>
#include <cmath>

namespace hydrox
{

    class SurgeP
    {
    public:
        struct Params
        {
            double kp = 160.0;
            double ki = 60.0;
            double kd = 8.0;
            // Limit the PI memory enough to avoid turn-exit windup while still
            // trimming straight-line drag. True turn-speed fidelity should come
            // from anti-windup/feed-forward, not a larger integral clamp.
            double integral_limit = 20.0;
            // Extra unwind when the speed error changes sign. This lets rapid
            // deceleration commands clear stored positive thrust before the
            // vehicle coasts through the next phase.
            double unwind_gain = 1.0;
            // Forward-speed drag feed-forward: tau += drag_ff * ref * |ref|.
            // This keeps cruise thrust near the expected drag load so the PI
            // term trims instead of banging between full throttle and reverse.
            double drag_ff = 0.0;
            // Physical surge-force authority; <= 0 leaves the output unbounded.
            double tau_max = 0.0;
            // Optional actuator-like shaping for one-way propellers. When
            // forward_min_tau is set to 0, a positive speed reference coasts
            // instead of commanding reverse thrust after a small overspeed.
            double forward_min_tau = -1.0e9;
            double tau_rate_limit = 0.0;
            // When the vehicle is already faster than a positive speed
            // reference, cap the command to a bounded braking force. This avoids
            // feed-forward or residual integral holding positive thrust above
            // the training-env speed limit.
            double overspeed_deadband = 0.0;
            double overspeed_brake_gain = 0.0;
            double overspeed_brake_tau_max = 0.0;
        };
        explicit SurgeP(const Params &p = detail::default_parameter<Params>()) : _p(p) {}

        void reset()
        {
            _integral = 0.0;
            _last_tau = 0.0;
        }

        /** @return tau_X (N) */
        double step(double surge, double surge_ref, double dt)
        {
            if (surge_ref > -1.0e-3 && surge_ref < 1.0e-3)
                _integral = 0.0;

            const double e = surge_ref - surge;
            const double drag_ff = _p.drag_ff * surge_ref * std::abs(surge_ref);
            const bool unwinding = (_integral > 0.0 && e < 0.0) ||
                                   (_integral < 0.0 && e > 0.0);
            const double i_gain = unwinding ? _p.unwind_gain : 1.0;
            const double limited_dt = std::max(0.0, dt);
            const double candidate_integral = std::max(
                -_p.integral_limit,
                std::min(_p.integral_limit, _integral + i_gain * e * limited_dt));
            const double candidate_tau =
                drag_ff + _p.kp * e + _p.ki * candidate_integral - _p.kd * surge;
            const bool force_is_bounded = _p.tau_max > 1.0e-6;
            const bool can_integrate =
                !force_is_bounded ||
                std::abs(candidate_tau) < _p.tau_max ||
                candidate_tau * e < 0.0;
            if (can_integrate)
                _integral = candidate_integral;

            double tau = drag_ff + _p.kp * e + _p.ki * _integral - _p.kd * surge;
            if (surge_ref > 1.0e-3 &&
                _p.overspeed_brake_gain > 1.0e-6 &&
                _p.overspeed_brake_tau_max > 1.0e-6 &&
                e < -_p.overspeed_deadband)
            {
                const double excess = (-e) - _p.overspeed_deadband;
                const double brake_tau = -std::min(
                    _p.overspeed_brake_tau_max,
                    _p.overspeed_brake_gain * excess);
                if (tau > brake_tau)
                    tau = brake_tau;
            }
            if (surge_ref > 1.0e-3 && tau < _p.forward_min_tau)
                tau = _p.forward_min_tau;

            if (_p.tau_rate_limit > 1.0e-6 && dt > 1.0e-6)
            {
                const double max_step = _p.tau_rate_limit * dt;
                if (tau > _last_tau + max_step)
                    tau = _last_tau + max_step;
                else if (tau < _last_tau - max_step)
                    tau = _last_tau - max_step;
            }
            if (_p.tau_max > 1.0e-6)
                tau = std::max(-_p.tau_max, std::min(_p.tau_max, tau));
            _last_tau = tau;
            return tau;
        }

    private:
        Params _p;
        double _integral = 0.0;
        double _last_tau = 0.0;
    };

} // namespace hydrox
