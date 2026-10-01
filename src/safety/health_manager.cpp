#include "hydrox/safety/health_manager.h"

#include <cstddef>

namespace hydrox::safety
{
    namespace
    {
        constexpr int severity_rank(HealthSeverity severity) noexcept
        {
            return static_cast<int>(severity);
        }

        bool is_stale(const ComponentHealth &report, uint64_t now_us) noexcept
        {
            return report.stale_after_us > 0 &&
                   now_us > report.observed_at_us &&
                   now_us - report.observed_at_us > report.stale_after_us;
        }
    } // namespace

    HealthManager::HealthManager(
        HealthMask required_for_arm,
        HealthMask required_for_control,
        SafetyAction missing_control_action) noexcept
        : required_for_arm_(required_for_arm),
          required_for_control_(required_for_control),
          missing_control_action_(missing_control_action)
    {
        reset();
    }

    void HealthManager::reset() noexcept
    {
        reports_.fill(ComponentHealth{});
        received_components_ = 0;
    }

    bool HealthManager::update(const ComponentHealth &report) noexcept
    {
        const std::size_t index = static_cast<std::size_t>(report.component);
        if (index >= reports_.size())
        {
            return false;
        }

        reports_[index] = report;
        received_components_ |= health_component_bit(report.component);
        return true;
    }

    VehicleHealthSnapshot HealthManager::snapshot(uint64_t now_us) const noexcept
    {
        VehicleHealthSnapshot result{};
        result.observed_components = received_components_;
        result.can_arm = true;
        result.control_available = true;

        int primary_action_rank = action_rank(SafetyAction::None);
        int primary_severity_rank = severity_rank(HealthSeverity::OK);

        const auto consider_primary = [&](HealthComponent component,
                                          HealthSeverity severity,
                                          SafetyAction action,
                                          SafetyReason reason,
                                          uint32_t detail_code) noexcept
        {
            const int candidate_action_rank = action_rank(action);
            const int candidate_severity_rank = severity_rank(severity);
            if (candidate_action_rank > primary_action_rank ||
                (candidate_action_rank == primary_action_rank &&
                 candidate_severity_rank > primary_severity_rank))
            {
                result.primary_component = component;
                result.reason = reason;
                result.detail_code = detail_code;
                primary_action_rank = candidate_action_rank;
                primary_severity_rank = candidate_severity_rank;
            }
            result.recommended_action =
                more_conservative_action(result.recommended_action, action);
            if (candidate_severity_rank > severity_rank(result.severity))
            {
                result.severity = severity;
            }
        };

        for (std::size_t index = 0; index < reports_.size(); ++index)
        {
            const auto component = static_cast<HealthComponent>(index);
            const HealthMask bit = health_component_bit(component);
            const bool required_for_arm = (required_for_arm_ & bit) != 0;
            const bool required_for_control =
                (required_for_control_ & bit) != 0;
            const bool received = (received_components_ & bit) != 0;

            if (!received)
            {
                if (required_for_arm)
                {
                    result.can_arm = false;
                }
                if (required_for_control)
                {
                    result.control_available = false;
                }
                if (required_for_arm || required_for_control)
                {
                    consider_primary(
                        component,
                        HealthSeverity::CRITICAL,
                        required_for_control ? missing_control_action_
                                             : SafetyAction::Warn,
                        SafetyReason::ComponentMissing,
                        0);
                }
                continue;
            }

            const ComponentHealth &report = reports_[index];
            const bool stale = is_stale(report, now_us);
            const bool unhealthy = !report.present || !report.healthy;
            if (stale)
            {
                result.stale_components |= bit;
            }
            if (unhealthy)
            {
                result.unhealthy_components |= bit;
            }

            if (!report.can_arm || (required_for_arm && (stale || unhealthy)))
            {
                result.can_arm = false;
            }
            if (!report.control_available ||
                (required_for_control && (stale || unhealthy)))
            {
                result.control_available = false;
            }

            HealthSeverity severity = report.severity;
            SafetyAction action = report.recommended_action;
            SafetyReason reason = SafetyReason::None;
            if (stale)
            {
                severity = HealthSeverity::CRITICAL;
                reason = SafetyReason::ComponentStale;
                if (required_for_control)
                {
                    action = more_conservative_action(
                        action, missing_control_action_);
                }
                else if (required_for_arm)
                {
                    action = more_conservative_action(action, SafetyAction::Warn);
                }
            }
            else if (unhealthy || report.severity != HealthSeverity::OK ||
                     report.recommended_action != SafetyAction::None)
            {
                reason = report.reason != SafetyReason::None
                             ? report.reason : SafetyReason::ComponentFault;
                if (unhealthy && required_for_control)
                {
                    action = more_conservative_action(
                        action, missing_control_action_);
                    severity = HealthSeverity::CRITICAL;
                }
            }

            if (reason != SafetyReason::None)
            {
                consider_primary(component,
                                 severity,
                                 action,
                                 reason,
                                 report.detail_code);
            }
        }

        if (result.reason == SafetyReason::None &&
            !result.control_available)
        {
            result.reason = SafetyReason::ControlCoreUnavailable;
            result.severity = HealthSeverity::CRITICAL;
            result.recommended_action = more_conservative_action(
                result.recommended_action, missing_control_action_);
        }
        return result;
    }
} // namespace hydrox::safety
