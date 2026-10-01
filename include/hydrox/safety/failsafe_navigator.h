#pragma once
#include "hydrox/safety/safety_profile.h"
#include "gnc/control_interfaces.h"

namespace hydrox::safety
{
    struct FailsafeContext
    {
        bool position_available = false;
        bool vertical_available = false;
        FlightPhase flight_phase = FlightPhase::NotApplicable;
    };

    /** Domain-aware reference generation; no actuator bypass or truth-state teleport. */
    class VehicleFailsafeNavigator
    {
    public:
        explicit VehicleFailsafeNavigator(const SafetyProfile &profile = SafetyProfile{}) noexcept;
        void reset() noexcept;
        bool enter(VehicleMode mode, const NavigationState &state,
                   const AuthorizedReference *last_authorized, uint64_t now_us,
                   const FailsafeContext &context) noexcept;
        ControlCandidate update(VehicleMode mode, const NavigationState &state,
                                uint64_t now_us, const FailsafeContext &context) noexcept;
    private:
        static double slew(double value, double target, double rate, double dt) noexcept;
        static bool supported(VehicleMode mode) noexcept;
        void capture_position(const NavigationState &state) noexcept;
        SafetyProfile profile_{};
        VehicleMode mode_ = VehicleMode::Boot;
        GNCSetpoint reference_{};
        double heading_ = 0.0, depth_ = 0.0, surge_ = 0.0, surface_depth_ = 0.0;
        double north_ = 0.0, east_ = 0.0, center_n_ = 0.0, center_e_ = 0.0;
        uint64_t last_update_us_ = 0, sequence_ = 0;
        bool initialized_ = false, position_available_ = false, vtol_cruise_ = false;
    };
}
