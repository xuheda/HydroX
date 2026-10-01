#include "learning/residual_rl.h"

#include <cmath>
#include <cstdio>
#include <memory>

namespace
{
    bool equal(double a, double b)
    {
        return std::abs(a - b) < 1e-9;
    }

    int fail(const char *message)
    {
        std::fprintf(stderr, "[test_residual_rl] %s\n", message);
        return 1;
    }

    class CapturingPolicy final : public hydrox::learning::IResidualPolicy
    {
    public:
        void reset() override { ++reset_calls; }

        hydrox::learning::ResidualAction infer(
            const hydrox::learning::ResidualObservation &observation) override
        {
            last = observation;
            ++calls;
            hydrox::learning::ResidualAction action;
            action.valid = true;
            action.confidence = 1.0;
            action.normalized[0] = 1.0;
            return action;
        }

        hydrox::learning::ResidualObservation last{};
        int calls = 0;
        int reset_calls = 0;
    };
}

int main()
{
    using namespace hydrox;
    using namespace hydrox::learning;

    Wrench base = Wrench::Zero();
    base[0] = 10.0;
    ResidualAction action;
    action.valid = true;
    action.confidence = 0.9;
    action.normalized[0] = 2.0; // Must be clipped before scaling.
    action.normalized[4] = -0.5;

    ResidualSafetyFilter::Params params;
    params.enabled = true;
    params.blend = 0.5;
    params.max_delta[0] = 8.0;
    params.max_delta[4] = 4.0;
    ResidualSafetyFilter filter(params);
    const Wrench corrected = filter.apply(base, action, 0.01);
    if (!equal(corrected[0], 14.0) || !equal(corrected[4], -1.0))
        return fail("normalized residual was not correctly bounded and scaled");

    params.max_rate[0] = 10.0;
    ResidualSafetyFilter rate_limited(params);
    const Wrench first = rate_limited.apply(base, action, 0.1);
    if (!equal(first[0], 11.0))
        return fail("residual slew limit was not applied");
    const Wrench second = rate_limited.apply(base, action, 0.1);
    if (!equal(second[0], 12.0))
        return fail("residual slew state was not retained");

    action.confidence = 0.1;
    const Wrench decaying = rate_limited.apply(base, action, 0.05);
    if (!equal(decaying[0], 11.5))
        return fail("low-confidence residual did not decay through its slew limit");
    const Wrench rejected = rate_limited.apply(base, action, 1.0);
    if (!equal(rejected[0], base[0]))
        return fail("low-confidence residual did not finish fail-closed decay");

    auto policy = std::make_unique<CapturingPolicy>();
    CapturingPolicy *policy_probe = policy.get();
    ResidualSafetyFilter::Params module_params;
    module_params.enabled = true;
    module_params.blend = 0.5;
    module_params.max_delta[0] = 8.0;
    ResidualRlModule module(module_params, std::move(policy));
    module.reset();
    if (policy_probe->reset_calls != 1)
        return fail("module reset did not reset causal policy state");
    ResidualObservation observation;
    observation.base_wrench = base;
    observation.previous_actuator.ch[4] = 0.3f;
    module.update(observation, 0.1);
    if (policy_probe->calls != 1 ||
        !equal(policy_probe->last.previous_applied_delta[0], 0.0) ||
        std::abs(policy_probe->last.previous_actuator.ch[4] - 0.3f) > 1e-6f)
    {
        return fail("first causal policy context was not zero/previous-actuator exact");
    }
    module.update(observation, 0.1);
    if (policy_probe->calls != 2 ||
        !equal(policy_probe->last.previous_applied_delta[0], 4.0))
    {
        return fail("policy did not receive the actually applied previous residual");
    }

    return 0;
}
