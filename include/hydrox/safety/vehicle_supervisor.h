#pragma once

#include "hydrox/safety/safety_profile.h"

namespace hydrox::safety
{
    /** Owns vehicle mode, arming authority and fault escalation policy. */
    class VehicleSupervisor
    {
    public:
        explicit VehicleSupervisor(
            const SafetyProfile &profile = SafetyProfile{}) noexcept;

        void reset(uint64_t now_us = 0) noexcept;
        SupervisorOutput update(const SupervisorInput &input) noexcept;
        SupervisorOutput output() const noexcept;

        const SafetyProfile &profile() const noexcept { return profile_; }

    private:
        void transition_to(VehicleMode mode,
                           SafetyReason reason,
                           uint64_t now_us) noexcept;
        void apply_health_action(const SupervisorInput &input) noexcept;
        VehicleMode external_loss_target() const noexcept;
        SafetyAction mode_action() const noexcept;

        SafetyProfile profile_{};
        VehicleMode mode_ = VehicleMode::Boot;
        SafetyReason reason_ = SafetyReason::None;
        uint64_t transition_sequence_ = 0;
        uint64_t mode_entered_at_us_ = 0;
        uint64_t hold_started_at_us_ = 0;
        bool armed_ = false;
        bool fault_latched_ = false;
        bool session_suspended_ = false;
    };
} // namespace hydrox::safety
