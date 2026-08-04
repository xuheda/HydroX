#include "fossen_vehicle_params.h"
#include "learning/control_shadow.h"

#include <cmath>
#include <cstdio>

namespace
{
    int fail(const char *message)
    {
        std::fprintf(stderr, "[test_control_shadow] %s\n", message);
        return 1;
    }

    bool finite(const hydrox::Wrench &wrench)
    {
        return wrench.allFinite();
    }
} // namespace

int main()
{
    using namespace hydrox;
    using namespace hydrox::learning;

    const FossenControlParams params = builtin_fossen_control_params("EcaA9");
    ResidualSafetyFilter::Params safety;
    safety.enabled = true;
    safety.blend = 0.10;
    safety.min_confidence = 0.75;
    safety.max_delta[0] = 60.0;
    safety.max_delta[4] = 8.0;
    safety.max_delta[5] = 18.0;
    safety.max_rate[0] = 120.0;
    safety.max_rate[4] = 16.0;
    safety.max_rate[5] = 36.0;
    ControlShadowReplay replay(params, safety);

    ControlShadowInput input;
    input.reset_controller = true;
    input.dt_s = 0.02;
    input.mode = GNCMode::DEPTH_HOLD;
    input.state.depth_m = 50.0;
    input.state.eta[2] = 50.0;
    input.state.eta[5] = 0.10;
    input.state.nu[0] = 1.0;
    input.setpoint.depth_ref = 55.0;
    input.setpoint.heading_ref = 0.35;
    input.setpoint.surge_ref = 1.4;
    input.residual.valid = true;
    input.residual.confidence = 1.0;
    input.residual.normalized[0] = 1.0;
    input.residual.normalized[4] = -1.0;
    input.residual.normalized[5] = 1.0;

    const ControlShadowOutput first = replay.step(input);
    if (!finite(first.base_wrench) || !finite(first.final_wrench))
        return fail("shadow replay produced non-finite wrenches");
    if (std::abs(first.applied_delta[0] - 2.4) > 1e-9 ||
        std::abs(first.applied_delta[4] + 0.32) > 1e-9 ||
        std::abs(first.applied_delta[5] - 0.72) > 1e-9)
        return fail("shadow replay did not apply the shared residual slew limits");
    for (float value : first.final_actuator.ch)
        if (!std::isfinite(value) || std::abs(value) > 1.000001F)
            return fail("allocator output was not finite and normalized");

    input.reset_controller = false;
    input.residual.valid = false;
    const ControlShadowOutput rejected = replay.step(input);
    if ((rejected.applied_delta.array().abs() > 1e-12).any())
        return fail("invalid residual must leave the original controller output unchanged");
    return 0;
}
