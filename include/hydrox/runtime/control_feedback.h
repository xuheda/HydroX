#pragma once

#include "mavlink_hil.h"
#include "types.h"

#include <cstdint>

namespace hydrox::runtime
{
    /** State source presented to the common GNC pipeline. */
    enum class ControlFeedbackSource : uint8_t
    {
        EstimatedState = 0,
        TruthDebug = 1,
    };

    struct ControlFeedbackSelection
    {
        NavigationState state = NavigationState::zeros();
        bool used_truth = false;
    };

    inline ControlFeedbackSelection select_control_feedback(
        const NavigationState &estimated_state,
        const HilTruthStateMsg &truth_state,
        bool truth_is_recent,
        ControlFeedbackSource source)
    {
        ControlFeedbackSelection selection;
        selection.state = estimated_state;

        if (source != ControlFeedbackSource::TruthDebug ||
            !truth_is_recent ||
            !truth_state.valid)
        {
            return selection;
        }

        for (int i = 0; i < 6; ++i)
        {
            selection.state.eta[i] = truth_state.eta[i];
            selection.state.nu[i] = truth_state.nu[i];
        }
        selection.state.depth_m = selection.state.eta[2];
        selection.state.dvl_valid = true;
        selection.used_truth = true;
        return selection;
    }
} // namespace hydrox::runtime
