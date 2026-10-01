#pragma once

#include "types.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace hydrox::safety
{
    enum class HealthSeverity : uint8_t
    {
        OK = 0,
        INFO,
        WARNING,
        DEGRADED,
        CRITICAL,
        EMERGENCY,
    };

    enum class HealthComponent : uint8_t
    {
        Platform = 0,
        Imu,
        DepthSensor,
        Dvl,
        Gps,
        EstimatorAttitude,
        EstimatorDepth,
        EstimatorVelocity,
        EstimatorPosition,
        ExternalControlLink,
        Controller,
        Allocator,
        Actuator,
        Battery,
        LeakDetector,
        InternalPressure,
        InternalTemperature,
        OperationalLimits,
        Count,
    };

    using HealthMask = uint64_t;

    constexpr std::size_t health_component_count() noexcept
    {
        return static_cast<std::size_t>(HealthComponent::Count);
    }

    constexpr HealthMask health_component_bit(
        HealthComponent component) noexcept
    {
        return HealthMask{1} << static_cast<uint8_t>(component);
    }

    enum class VehicleMode : uint8_t
    {
        Boot = 0,
        Standby,
        ArmedIdle,
        ExternalControl,
        CommandHold,
        FailsafeStabilize,
        FailsafeHold,
        FailsafeSurface,
        EmergencyAbort,
        OutputDisabled,
        FaultLocked,
    };

    enum class SafetyAction : uint8_t
    {
        None = 0,
        Warn,
        LimitEnvelope,
        HoldCommand,
        Stabilize,
        HoldOrLoiter,
        ControlledSurface,
        EmergencyAbort,
        DisableOutput,
    };

    enum class SafetyReason : uint16_t
    {
        None = 0,
        BootCheckFailed,
        PrearmDenied,
        ArmedByOperator,
        DisarmedByOperator,
        FaultAcknowledged,
        ExternalControlAccepted,
        ExternalCommandStale,
        ExternalCommandLost,
        ExternalSourceReleased,
        ExternalControlRecovered,
        StabilizeDwellComplete,
        ComponentMissing,
        ComponentStale,
        ComponentFault,
        ControlCoreUnavailable,
        InvalidReference,
        NoAuthorizedReference,
        SafetyProfileInvalid,
    };

    enum class ControlSource : uint8_t
    {
        None = 0,
        External,
        Failsafe,
        EmergencyAbort,
    };

    constexpr int action_rank(SafetyAction action) noexcept
    {
        switch (action)
        {
        case SafetyAction::None: return 0;
        case SafetyAction::Warn: return 10;
        case SafetyAction::LimitEnvelope: return 20;
        case SafetyAction::HoldCommand: return 30;
        case SafetyAction::Stabilize: return 40;
        case SafetyAction::HoldOrLoiter: return 50;
        case SafetyAction::ControlledSurface: return 60;
        case SafetyAction::EmergencyAbort: return 70;
        case SafetyAction::DisableOutput: return 80;
        }
        return 80;
    }

    constexpr SafetyAction more_conservative_action(
        SafetyAction left,
        SafetyAction right) noexcept
    {
        return action_rank(right) > action_rank(left) ? right : left;
    }

    struct ComponentHealth
    {
        HealthComponent component = HealthComponent::Platform;
        HealthSeverity severity = HealthSeverity::OK;
        SafetyAction recommended_action = SafetyAction::None;
        SafetyReason reason = SafetyReason::None;
        uint32_t detail_code = 0;
        uint64_t observed_at_us = 0;
        uint64_t stale_after_us = 0;
        bool present = true;
        bool healthy = true;
        bool can_arm = true;
        bool control_available = true;
    };

    struct VehicleHealthSnapshot
    {
        HealthSeverity severity = HealthSeverity::OK;
        SafetyAction recommended_action = SafetyAction::None;
        SafetyReason reason = SafetyReason::None;
        HealthComponent primary_component = HealthComponent::Platform;
        uint32_t detail_code = 0;
        HealthMask observed_components = 0;
        HealthMask stale_components = 0;
        HealthMask unhealthy_components = 0;
        bool can_arm = false;
        bool control_available = false;
    };

    struct ControlCandidate
    {
        ControlSource source = ControlSource::None;
        uint64_t session_id = 0;
        uint64_t sequence = 0;
        uint64_t received_at_us = 0;
        uint64_t valid_until_us = 0;
        GNCMode mode = GNCMode::DISABLED;
        GNCSetpoint setpoint{};
        bool valid = false;
    };

    struct CandidateSet
    {
        ControlCandidate external{};
        ControlCandidate failsafe{};
        ControlCandidate emergency{};
    };

    struct AuthorizedReference
    {
        ControlSource source = ControlSource::None;
        uint64_t session_id = 0;
        uint64_t sequence = 0;
        GNCMode mode = GNCMode::DISABLED;
        GNCSetpoint setpoint{};
        SafetyReason reason = SafetyReason::NoAuthorizedReference;
        bool valid = false;
        bool actuator_authorized = false;
    };

    struct SupervisorInput
    {
        uint64_t now_us = 0;
        uint64_t external_command_age_us =
            std::numeric_limits<uint64_t>::max();
        VehicleHealthSnapshot health{};
        // Transport suspension inhibits output without clearing fault latches.
        bool session_suspended = false;
        bool boot_complete = false;
        bool arm_requested = false;
        bool disarm_requested = false;
        bool fault_acknowledged = false;
        bool resume_requested = false;
        bool external_control_requested = false;
        bool external_command_valid = false;
        bool external_source_released = false;
    };

    struct SupervisorOutput
    {
        VehicleMode mode = VehicleMode::Boot;
        SafetyAction action = SafetyAction::None;
        SafetyReason reason = SafetyReason::None;
        uint64_t transition_sequence = 0;
        uint64_t mode_entered_at_us = 0;
        bool armed = false;
        bool external_authorized = false;
        bool actuator_authorized = false;
        bool operator_ack_required = false;
        bool transitioned = false;
        bool session_suspended = false;
    };

    inline const char *vehicle_mode_name(VehicleMode mode) noexcept
    {
        switch (mode)
        {
        case VehicleMode::Boot: return "BOOT";
        case VehicleMode::Standby: return "STANDBY";
        case VehicleMode::ArmedIdle: return "ARMED_IDLE";
        case VehicleMode::ExternalControl: return "EXTERNAL_CONTROL";
        case VehicleMode::CommandHold: return "COMMAND_HOLD";
        case VehicleMode::FailsafeStabilize: return "FAILSAFE_STABILIZE";
        case VehicleMode::FailsafeHold: return "FAILSAFE_HOLD";
        case VehicleMode::FailsafeSurface: return "FAILSAFE_SURFACE";
        case VehicleMode::EmergencyAbort: return "EMERGENCY_ABORT";
        case VehicleMode::OutputDisabled: return "OUTPUT_DISABLED";
        case VehicleMode::FaultLocked: return "FAULT_LOCKED";
        }
        return "UNKNOWN";
    }

    inline const char *safety_action_name(SafetyAction action) noexcept
    {
        switch (action)
        {
        case SafetyAction::None: return "NONE";
        case SafetyAction::Warn: return "WARN";
        case SafetyAction::LimitEnvelope: return "LIMIT_ENVELOPE";
        case SafetyAction::HoldCommand: return "HOLD_COMMAND";
        case SafetyAction::Stabilize: return "STABILIZE";
        case SafetyAction::HoldOrLoiter: return "HOLD_OR_LOITER";
        case SafetyAction::ControlledSurface: return "CONTROLLED_SURFACE";
        case SafetyAction::EmergencyAbort: return "EMERGENCY_ABORT";
        case SafetyAction::DisableOutput: return "DISABLE_OUTPUT";
        }
        return "UNKNOWN";
    }

    inline const char *safety_reason_name(SafetyReason reason) noexcept
    {
        switch (reason)
        {
        case SafetyReason::None: return "NONE";
        case SafetyReason::BootCheckFailed: return "BOOT_CHECK_FAILED";
        case SafetyReason::PrearmDenied: return "PREARM_DENIED";
        case SafetyReason::ArmedByOperator: return "ARMED_BY_OPERATOR";
        case SafetyReason::DisarmedByOperator: return "DISARMED_BY_OPERATOR";
        case SafetyReason::FaultAcknowledged: return "FAULT_ACKNOWLEDGED";
        case SafetyReason::ExternalControlAccepted:
            return "EXTERNAL_CONTROL_ACCEPTED";
        case SafetyReason::ExternalCommandStale:
            return "EXTERNAL_COMMAND_STALE";
        case SafetyReason::ExternalCommandLost:
            return "EXTERNAL_COMMAND_LOST";
        case SafetyReason::ExternalSourceReleased:
            return "EXTERNAL_SOURCE_RELEASED";
        case SafetyReason::ExternalControlRecovered:
            return "EXTERNAL_CONTROL_RECOVERED";
        case SafetyReason::StabilizeDwellComplete:
            return "STABILIZE_DWELL_COMPLETE";
        case SafetyReason::ComponentMissing: return "COMPONENT_MISSING";
        case SafetyReason::ComponentStale: return "COMPONENT_STALE";
        case SafetyReason::ComponentFault: return "COMPONENT_FAULT";
        case SafetyReason::ControlCoreUnavailable:
            return "CONTROL_CORE_UNAVAILABLE";
        case SafetyReason::InvalidReference: return "INVALID_REFERENCE";
        case SafetyReason::NoAuthorizedReference:
            return "NO_AUTHORIZED_REFERENCE";
        case SafetyReason::SafetyProfileInvalid:
            return "SAFETY_PROFILE_INVALID";
        }
        return "UNKNOWN";
    }

    inline const char *control_source_name(ControlSource source) noexcept
    {
        switch (source)
        {
        case ControlSource::None: return "NONE";
        case ControlSource::External: return "EXTERNAL";
        case ControlSource::Failsafe: return "FAILSAFE";
        case ControlSource::EmergencyAbort: return "EMERGENCY_ABORT";
        }
        return "UNKNOWN";
    }
} // namespace hydrox::safety
