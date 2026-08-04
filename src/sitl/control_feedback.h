#pragma once

#include "hydrox/runtime/control_feedback.h"

namespace hydrox::sitl
{
    // Compatibility aliases. The implementation belongs to the common runtime
    // so SITL and HITL cannot acquire different feedback semantics.
    using ControlFeedbackSource = runtime::ControlFeedbackSource;
    using ControlFeedbackSelection = runtime::ControlFeedbackSelection;
    using runtime::select_control_feedback;
} // namespace hydrox::sitl
