#include "hydrox/safety/command_arbiter.h"

#include "hydrox/runtime/hil_contract.h"

namespace hydrox::safety
{
    bool CommandArbiter::candidate_is_valid(
        const ControlCandidate &candidate,
        ControlSource expected_source,
        uint64_t now_us) noexcept
    {
        return candidate.valid && candidate.source == expected_source &&
               candidate.received_at_us <= now_us &&
               candidate.valid_until_us >= now_us &&
               candidate.mode != GNCMode::DISABLED &&
               runtime::valid_gnc_setpoint(candidate.setpoint, candidate.mode);
    }

    AuthorizedReference CommandArbiter::select(
        const SupervisorOutput &supervisor,
        const CandidateSet &candidates,
        uint64_t now_us) const noexcept
    {
        AuthorizedReference result{};
        result.reason = supervisor.reason;
        if (!supervisor.actuator_authorized)
        {
            result.reason = SafetyReason::NoAuthorizedReference;
            return result;
        }

        const ControlCandidate *candidate = nullptr;
        ControlSource expected_source = ControlSource::None;
        switch (supervisor.mode)
        {
        case VehicleMode::ExternalControl:
            candidate = &candidates.external;
            expected_source = ControlSource::External;
            break;
        case VehicleMode::CommandHold:
        case VehicleMode::FailsafeStabilize:
        case VehicleMode::FailsafeHold:
        case VehicleMode::FailsafeSurface:
            candidate = &candidates.failsafe;
            expected_source = ControlSource::Failsafe;
            break;
        case VehicleMode::EmergencyAbort:
            candidate = &candidates.emergency;
            expected_source = ControlSource::EmergencyAbort;
            break;
        default:
            result.reason = SafetyReason::NoAuthorizedReference;
            return result;
        }

        if (!candidate_is_valid(*candidate, expected_source, now_us))
        {
            result.reason = SafetyReason::InvalidReference;
            return result;
        }

        result.source = candidate->source;
        result.session_id = candidate->session_id;
        result.sequence = candidate->sequence;
        result.mode = candidate->mode;
        result.setpoint = candidate->setpoint;
        result.valid = true;
        result.actuator_authorized = true;
        return result;
    }
} // namespace hydrox::safety
