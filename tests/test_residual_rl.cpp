#include "learning/residual_rl.h"

#include <cmath>
#include <cstdio>

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

    action.confidence = 0.1;
    const Wrench rejected = rate_limited.apply(base, action, 0.1);
    if (!equal(rejected[0], base[0]))
        return fail("low-confidence residual was not rejected");

    return 0;
}
