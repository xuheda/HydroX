#include "hydrox/safety/vehicle_supervisor.h"

#include <cmath>
#include <limits>

namespace hydrox::safety
{
    namespace
    {
        bool is_active_control_mode(VehicleMode mode) noexcept
        {
            switch (mode)
            {
            case VehicleMode::ExternalControl:
            case VehicleMode::CommandHold:
            case VehicleMode::FailsafeStabilize:
            case VehicleMode::FailsafeHold:
            case VehicleMode::FailsafeSurface:
            case VehicleMode::EmergencyAbort:
                return true;
            default:
                return false;
            }
        }

        uint64_t seconds_to_microseconds(double seconds) noexcept
        {
            if (!std::isfinite(seconds) || seconds <= 0.0)
            {
                return 0;
            }
            constexpr double maximum =
                static_cast<double>(std::numeric_limits<uint64_t>::max());
            const double value = seconds * 1'000'000.0;
            return value >= maximum ? std::numeric_limits<uint64_t>::max()
                                    : static_cast<uint64_t>(value);
        }
    } // namespace

    VehicleSupervisor::VehicleSupervisor(const SafetyProfile &profile) noexcept
        : profile_(profile)
    {
        reset();
    }

    void VehicleSupervisor::reset(uint64_t now_us) noexcept
    {
        mode_ = VehicleMode::Boot;
        reason_ = SafetyReason::None;
        transition_sequence_ = 0;
        mode_entered_at_us_ = now_us;
        hold_started_at_us_ = now_us;
        armed_ = false;
        fault_latched_ = false;
        session_suspended_ = false;
    }

    void VehicleSupervisor::transition_to(VehicleMode mode,
                                          SafetyReason reason,
                                          uint64_t now_us) noexcept
    {
        reason_ = reason;
        if (mode == mode_)
        {
            return;
        }
        mode_ = mode;
        mode_entered_at_us_ = now_us;
        ++transition_sequence_;
        if (mode == VehicleMode::CommandHold)
        {
            hold_started_at_us_ = now_us;
        }
    }

    VehicleMode VehicleSupervisor::external_loss_target() const noexcept
    {
        switch (profile_.external_loss_action)
        {
        case SafetyAction::Stabilize:
            return VehicleMode::FailsafeStabilize;
        case SafetyAction::HoldOrLoiter:
            return VehicleMode::FailsafeHold;
        case SafetyAction::ControlledSurface:
            return VehicleMode::FailsafeSurface;
        case SafetyAction::EmergencyAbort:
            return VehicleMode::EmergencyAbort;
        case SafetyAction::DisableOutput:
            return VehicleMode::OutputDisabled;
        default:
            return VehicleMode::FailsafeHold;
        }
    }

    void VehicleSupervisor::apply_health_action(
        const SupervisorInput &input) noexcept
    {
        const SafetyAction action = input.health.recommended_action;
        if (fault_latched_ && action_rank(action) < action_rank(mode_action()))
            return; // a lesser/new warning cannot release an existing safety gate
        if (action_rank(action) < action_rank(SafetyAction::HoldCommand))
        {
            return;
        }

        const SafetyReason reason = input.health.reason == SafetyReason::None
                                        ? SafetyReason::ComponentFault
                                        : input.health.reason;
        switch (action)
        {
        case SafetyAction::HoldCommand:
            if (armed_)
            {
                transition_to(VehicleMode::CommandHold, reason, input.now_us);
            }
            break;
        case SafetyAction::Stabilize:
            fault_latched_ = true;
            transition_to(armed_ ? VehicleMode::FailsafeStabilize
                                 : VehicleMode::FaultLocked,
                          reason,
                          input.now_us);
            break;
        case SafetyAction::HoldOrLoiter:
            fault_latched_ = true;
            transition_to(armed_ ? VehicleMode::FailsafeHold
                                 : VehicleMode::FaultLocked,
                          reason,
                          input.now_us);
            break;
        case SafetyAction::ControlledSurface:
            fault_latched_ = true;
            transition_to(armed_ ? (profile_.underwater() ? VehicleMode::FailsafeSurface : VehicleMode::FailsafeHold)
                                 : VehicleMode::FaultLocked,
                          reason,
                          input.now_us);
            break;
        case SafetyAction::EmergencyAbort:
            fault_latched_ = true;
            transition_to(armed_ ? VehicleMode::EmergencyAbort
                                 : VehicleMode::FaultLocked,
                          reason,
                          input.now_us);
            break;
        case SafetyAction::DisableOutput:
            fault_latched_ = true;
            transition_to(armed_ ? VehicleMode::OutputDisabled
                                 : VehicleMode::FaultLocked,
                          reason,
                          input.now_us);
            break;
        default:
            break;
        }
    }

    SupervisorOutput VehicleSupervisor::update(
        const SupervisorInput &input) noexcept
    {
        const uint64_t sequence_before = transition_sequence_;
        if (session_suspended_ != input.session_suspended)
            ++transition_sequence_;
        session_suspended_ = input.session_suspended;
        if (!profile_.valid())
        {
            armed_ = false;
            fault_latched_ = true;
            transition_to(VehicleMode::FaultLocked,
                          SafetyReason::SafetyProfileInvalid,
                          input.now_us);
            SupervisorOutput result = output();
            result.transitioned = transition_sequence_ != sequence_before;
            return result;
        }

        if (input.disarm_requested)
        {
            armed_ = false;
            transition_to(fault_latched_ ||
                                  mode_ == VehicleMode::EmergencyAbort ||
                                  mode_ == VehicleMode::OutputDisabled
                              ? VehicleMode::FaultLocked
                              : VehicleMode::Standby,
                          SafetyReason::DisarmedByOperator,
                          input.now_us);
            SupervisorOutput result = output();
            result.transitioned = transition_sequence_ != sequence_before;
            return result;
        }

        if (mode_ == VehicleMode::FaultLocked)
        {
            if (input.fault_acknowledged && input.health.can_arm &&
                input.health.control_available)
            {
                fault_latched_ = false;
                transition_to(VehicleMode::Standby,
                              SafetyReason::FaultAcknowledged,
                              input.now_us);
            }
            SupervisorOutput result = output();
            result.transitioned = transition_sequence_ != sequence_before;
            return result;
        }

        if (session_suspended_ && mode_ != VehicleMode::Boot &&
            !(mode_ == VehicleMode::Standby && input.arm_requested))
            return output();

        if (mode_ != VehicleMode::Boot || input.boot_complete)
        {
            apply_health_action(input);
        }

        // Deliberate takeover is allowed in flight; a recovered packet alone
        // must never release the latch. Hard faults/output inhibition cannot
        // be cleared with this command.
        if (armed_ && input.resume_requested && input.health.control_available &&
            input.health.can_arm && action_rank(input.health.recommended_action) <
                action_rank(SafetyAction::HoldCommand) &&
            input.external_command_valid && input.external_control_requested &&
            input.external_command_age_us < profile_.external_command_warn_us &&
            (mode_ == VehicleMode::FailsafeStabilize || mode_ == VehicleMode::FailsafeHold ||
             mode_ == VehicleMode::FailsafeSurface))
        {
            fault_latched_ = false;
            transition_to(VehicleMode::ExternalControl, SafetyReason::ExternalControlRecovered, input.now_us);
        }

        switch (mode_)
        {
        case VehicleMode::Boot:
            if (input.boot_complete)
            {
                if (input.health.can_arm && input.health.control_available)
                {
                    transition_to(VehicleMode::Standby,
                                  SafetyReason::None,
                                  input.now_us);
                }
                else
                {
                    fault_latched_ = true;
                    transition_to(VehicleMode::FaultLocked,
                                  SafetyReason::BootCheckFailed,
                                  input.now_us);
                }
            }
            break;
        case VehicleMode::Standby:
            if (input.arm_requested)
            {
                if (input.health.can_arm && input.health.control_available)
                {
                    armed_ = true;
                    transition_to(VehicleMode::ArmedIdle,
                                  SafetyReason::ArmedByOperator,
                                  input.now_us);
                }
                else
                {
                    reason_ = SafetyReason::PrearmDenied;
                }
            }
            break;
        case VehicleMode::ArmedIdle:
            if (input.external_control_requested &&
                input.external_command_valid &&
                input.external_command_age_us <
                    profile_.external_command_warn_us)
            {
                transition_to(VehicleMode::ExternalControl,
                              SafetyReason::ExternalControlAccepted,
                              input.now_us);
            }
            break;
        case VehicleMode::ExternalControl:
            if (input.external_source_released)
            {
                fault_latched_ = true;
                transition_to(VehicleMode::FailsafeStabilize,
                              SafetyReason::ExternalSourceReleased,
                              input.now_us);
            }
            else if (input.external_command_age_us >= profile_.external_command_loss_us)
            {
                fault_latched_ = true;
                transition_to(VehicleMode::FailsafeStabilize,
                              SafetyReason::ExternalCommandLost, input.now_us);
            }
            else if (!input.external_command_valid ||
                     input.external_command_age_us >=
                         profile_.external_command_warn_us)
            {
                transition_to(VehicleMode::CommandHold,
                              SafetyReason::ExternalCommandStale,
                              input.now_us);
            }
            break;
        case VehicleMode::CommandHold:
            if (input.external_source_released)
            {
                fault_latched_ = true;
                transition_to(VehicleMode::FailsafeStabilize,
                              SafetyReason::ExternalSourceReleased,
                              input.now_us);
            }
            else if (input.external_command_valid &&
                     input.external_command_age_us <
                         profile_.external_command_warn_us)
            {
                transition_to(VehicleMode::ExternalControl,
                              SafetyReason::ExternalControlRecovered,
                              input.now_us);
            }
            else
            {
                const uint64_t hold_elapsed =
                    input.now_us >= hold_started_at_us_
                        ? input.now_us - hold_started_at_us_
                        : 0;
                if (input.external_command_age_us >=
                        profile_.external_command_loss_us ||
                    hold_elapsed >= profile_.command_hold_max_us)
                {
                    fault_latched_ = true;
                    transition_to(VehicleMode::FailsafeStabilize,
                                  SafetyReason::ExternalCommandLost,
                                  input.now_us);
                }
            }
            break;
        case VehicleMode::FailsafeStabilize:
        {
            const uint64_t dwell_us =
                input.now_us >= mode_entered_at_us_
                    ? input.now_us - mode_entered_at_us_
                    : 0;
            if (dwell_us >= seconds_to_microseconds(
                                profile_.stabilize_duration_s))
            {
                transition_to(external_loss_target(),
                              SafetyReason::StabilizeDwellComplete,
                              input.now_us);
            }
            break;
        }
        case VehicleMode::FailsafeHold:
        case VehicleMode::FailsafeSurface:
        case VehicleMode::EmergencyAbort:
        case VehicleMode::OutputDisabled:
        case VehicleMode::FaultLocked:
            break;
        }

        SupervisorOutput result = output();
        result.transitioned = transition_sequence_ != sequence_before;
        return result;
    }

    SafetyAction VehicleSupervisor::mode_action() const noexcept
    {
        switch (mode_)
        {
        case VehicleMode::CommandHold: return SafetyAction::HoldCommand;
        case VehicleMode::FailsafeStabilize: return SafetyAction::Stabilize;
        case VehicleMode::FailsafeHold:
            return SafetyAction::HoldOrLoiter;
        case VehicleMode::FailsafeSurface:
            return SafetyAction::ControlledSurface;
        case VehicleMode::EmergencyAbort: return SafetyAction::EmergencyAbort;
        case VehicleMode::OutputDisabled:
        case VehicleMode::FaultLocked: return SafetyAction::DisableOutput;
        default: return SafetyAction::None;
        }
    }

    SupervisorOutput VehicleSupervisor::output() const noexcept
    {
        SupervisorOutput result{};
        result.mode = mode_;
        result.action = mode_action();
        result.reason = reason_;
        result.transition_sequence = transition_sequence_;
        result.mode_entered_at_us = mode_entered_at_us_;
        result.armed = armed_;
        result.session_suspended = session_suspended_;
        result.external_authorized = !session_suspended_ && mode_ == VehicleMode::ExternalControl;
        result.actuator_authorized = !session_suspended_ && armed_ && is_active_control_mode(mode_) &&
                                     mode_ != VehicleMode::OutputDisabled;
        result.operator_ack_required = fault_latched_ ||
                                       mode_ == VehicleMode::FaultLocked;
        return result;
    }
} // namespace hydrox::safety
