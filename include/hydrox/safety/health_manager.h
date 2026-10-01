#pragma once

#include "hydrox/safety/safety_types.h"

#include <array>

namespace hydrox::safety
{
    /** Fixed-size, allocation-free component health aggregator. */
    class HealthManager
    {
    public:
        explicit HealthManager(
            HealthMask required_for_arm = 0,
            HealthMask required_for_control = 0,
            SafetyAction missing_control_action =
                SafetyAction::DisableOutput) noexcept;

        void reset() noexcept;
        bool update(const ComponentHealth &report) noexcept;
        VehicleHealthSnapshot snapshot(uint64_t now_us) const noexcept;

        HealthMask received_components() const noexcept
        {
            return received_components_;
        }

    private:
        std::array<ComponentHealth, health_component_count()> reports_{};
        HealthMask received_components_ = 0;
        HealthMask required_for_arm_ = 0;
        HealthMask required_for_control_ = 0;
        SafetyAction missing_control_action_ = SafetyAction::DisableOutput;
    };
} // namespace hydrox::safety
