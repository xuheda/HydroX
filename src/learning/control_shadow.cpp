#include "learning/control_shadow.h"

#include <cmath>
#include <stdexcept>

namespace hydrox::learning
{
namespace
{
    bool finite_state(const NavigationState &state)
    {
        return state.eta.allFinite() && state.nu.allFinite() &&
               std::isfinite(state.depth_m);
    }
} // namespace

ControlShadowReplay::ControlShadowReplay(
    const FossenControlParams &params,
    const ResidualSafetyFilter::Params &residual_params)
    : stack_(build_control_stack(params)), residual_filter_(residual_params)
{
    if (!params.valid || !stack_.controller || !stack_.allocator)
        throw std::invalid_argument("ControlShadowReplay requires valid controller parameters");
}

ControlShadowOutput ControlShadowReplay::step(const ControlShadowInput &input)
{
    if (!finite_state(input.state) || !std::isfinite(input.dt_s) || input.dt_s <= 0.0)
        throw std::invalid_argument("ControlShadowReplay received a non-finite state or invalid dt");

    if (input.reset_controller)
    {
        stack_.controller->reset(input.state);
        residual_filter_.reset();
    }

    // SITL retains controller state while repeatedly receiving an unchanged
    // setpoint.  Calling the setters is therefore safe and gives every replay
    // row the complete explicit contract.
    stack_.controller->set_mode(input.mode);
    stack_.controller->set_setpoint(input.setpoint);

    ControlShadowOutput output;
    output.base_wrench = stack_.controller->update(input.state, input.dt_s);
    output.final_wrench = residual_filter_.apply(
        output.base_wrench, input.residual, input.dt_s);
    output.applied_delta = output.final_wrench - output.base_wrench;
    output.base_actuator = stack_.allocator->allocate(
        output.base_wrench, input.state.surge());
    output.final_actuator = stack_.allocator->allocate(
        output.final_wrench, input.state.surge());
    return output;
}
} // namespace hydrox::learning
