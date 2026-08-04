#pragma once

// Compatibility aliases for downstream SITL integrations. New code should use
// hydrox/runtime/control_session.h directly.
#include "hydrox/runtime/control_session.h"

namespace hydrox::sitl
{
    using UeControlSessionGate = runtime::ControlSessionGate;
    using UeControlSessionPhase = runtime::ControlSessionPhase;

    inline const char *ue_control_session_phase_name(
        UeControlSessionPhase phase) noexcept
    {
        return runtime::control_session_phase_name(phase);
    }
}
