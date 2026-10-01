#include "learning/residual_rl.h"

#include <algorithm>
#include <cmath>

namespace hydrox::learning
{
namespace
{
    double clamp(double value, double low, double high)
    {
        return std::max(low, std::min(value, high));
    }

    bool finite_wrench(const Wrench &value)
    {
        for (int i = 0; i < value.size(); ++i)
            if (!std::isfinite(value[i]))
                return false;
        return true;
    }
} // namespace

void ResidualSafetyFilter::reset()
{
    previous_delta_.setZero();
}

Wrench ResidualSafetyFilter::apply(const Wrench &base, const ResidualAction &action, double dt)
{
    if (!params_.enabled)
    {
        reset();
        return base;
    }

    const bool accepted =
        action.valid && std::isfinite(action.confidence) &&
        action.confidence >= params_.min_confidence &&
        finite_wrench(action.normalized);
    if (!accepted)
    {
        // A missed/degraded policy response removes learned authority without
        // inserting a wrench discontinuity.  Decay the already-applied
        // residual through the same per-axis slew bound; an axis with no rate
        // bound drops immediately to the classical controller. Session reset
        // remains an unconditional zero through reset().
        for (int i = 0; i < previous_delta_.size(); ++i)
        {
            const double rate = std::max(0.0, params_.max_rate[i]);
            if (rate > 0.0 && std::isfinite(dt) && dt > 0.0)
            {
                const double max_step = rate * dt;
                previous_delta_[i] += clamp(
                    -previous_delta_[i], -max_step, max_step);
            }
            else
            {
                previous_delta_[i] = 0.0;
            }
        }
        return base + previous_delta_;
    }

    const double blend = clamp(params_.blend, 0.0, 1.0);
    Wrench requested = Wrench::Zero();
    for (int i = 0; i < requested.size(); ++i)
    {
        const double limit = std::max(0.0, params_.max_delta[i]);
        requested[i] = clamp(action.normalized[i], -1.0, 1.0) * limit * blend;

        const double rate = std::max(0.0, params_.max_rate[i]);
        if (rate > 0.0 && std::isfinite(dt) && dt > 0.0)
        {
            const double max_step = rate * dt;
            requested[i] = previous_delta_[i] +
                           clamp(requested[i] - previous_delta_[i], -max_step, max_step);
        }
    }

    previous_delta_ = requested;
    return base + requested;
}

ResidualRlModule::ResidualRlModule(const ResidualSafetyFilter::Params &params,
                                   std::unique_ptr<IResidualPolicy> policy)
    : policy_(std::move(policy)), safety_filter_(params)
{
    if (!policy_)
        policy_ = std::make_unique<NullResidualPolicy>();
}

void ResidualRlModule::reset()
{
    safety_filter_.reset();
    policy_->reset();
}

Wrench ResidualRlModule::update(const ResidualObservation &observation, double dt)
{
    ResidualObservation causal = observation;
    causal.previous_applied_delta = safety_filter_.previous_delta();
    return safety_filter_.apply(
        causal.base_wrench, policy_->infer(causal), dt);
}
} // namespace hydrox::learning
