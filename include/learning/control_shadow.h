#pragma once

// Offline-only control-stack replay for learning validation.
//
// This is deliberately separate from the SITL runtime loop.  It rebuilds the
// normal controller/allocator stack and evaluates a recorded state/setpoint
// sequence through the same tau -> actuator path, with the same residual
// safety filter.  It owns no transport and has no actuator authority.

#include "fossen_vehicle_params.h"
#include "gnc/control_factory.h"
#include "learning/residual_rl.h"

#include <memory>

namespace hydrox::learning
{
    struct ControlShadowInput
    {
        NavigationState state = NavigationState::zeros();
        GNCSetpoint setpoint{};
        GNCMode mode = GNCMode::DISABLED;
        ResidualAction residual{};
        double dt_s = 0.01;
        // Must be true on the first row of every independently recorded run.
        // It mirrors SITL's reset-before-update behavior without guessing at
        // mission events that were not recorded.
        bool reset_controller = false;
    };

    struct ControlShadowOutput
    {
        Wrench base_wrench = Wrench::Zero();
        Wrench final_wrench = Wrench::Zero();
        Wrench applied_delta = Wrench::Zero();
        ActuatorCmd base_actuator{};
        ActuatorCmd final_actuator{};
    };

    class ControlShadowReplay final
    {
    public:
        ControlShadowReplay(const FossenControlParams &params,
                            const ResidualSafetyFilter::Params &residual_params);

        ControlShadowOutput step(const ControlShadowInput &input);

    private:
        ControlStack stack_;
        ResidualSafetyFilter residual_filter_;
    };
} // namespace hydrox::learning
