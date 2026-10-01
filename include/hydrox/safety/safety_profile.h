#pragma once

#include "hydrox/safety/safety_types.h"

#include <cmath>
#include <cstdint>

namespace hydrox { struct ControlParameters; }

namespace hydrox::safety
{
    enum class VehicleSafetyKind : uint8_t
    {
        FinAuv, ThrusterUuv, SurfaceVessel, Multirotor, FixedWing, Vtol, Ground
    };

    struct SafetyProfile
    {
        VehicleSafetyKind vehicle_kind = VehicleSafetyKind::FinAuv;
        VehicleClass vehicle_class = VehicleClass::UUV;
        bool position_hold_capable = false;
        double loiter_speed_mps = 12.0;
        double loiter_radius_m = 80.0;
        double station_radius_m = 3.0;
        uint64_t external_command_warn_us = 200'000;
        uint64_t external_command_loss_us = 500'000;
        uint64_t command_hold_max_us = 300'000;
        uint64_t internal_reference_valid_us = 200'000;
        double stabilize_duration_s = 3.0;
        double safe_surge_mps = 2.0;
        double minimum_control_surge_mps = 1.5;
        double surface_surge_mps = 0.5;
        double surge_slew_mps2 = 0.5;
        double yaw_rate_slew_radps2 = 0.5;
        double surface_reference_rate_mps = 0.3;
        double surface_capture_depth_m = 0.5;
        SafetyAction external_loss_action = SafetyAction::ControlledSurface;

        bool underwater() const noexcept { return vehicle_class == VehicleClass::UUV; }
        bool accepts_mode(GNCMode mode) const noexcept
        {
            if (mode == GNCMode::SURFACE)
                return underwater() || vehicle_class == VehicleClass::USV;
            if (mode == GNCMode::DP)
                return vehicle_kind == VehicleSafetyKind::ThrusterUuv;
            return mode == GNCMode::DISABLED || mode == GNCMode::DEPTH_HOLD ||
                   mode == GNCMode::WAYPOINT_3D;
        }

        bool valid() const noexcept
        {
            const bool kind_matches_class =
                ((vehicle_kind == VehicleSafetyKind::FinAuv || vehicle_kind == VehicleSafetyKind::ThrusterUuv) && vehicle_class == VehicleClass::UUV) ||
                (vehicle_kind == VehicleSafetyKind::SurfaceVessel && vehicle_class == VehicleClass::USV) ||
                (vehicle_kind == VehicleSafetyKind::Multirotor && vehicle_class == VehicleClass::UAV_MULTIROTOR) ||
                (vehicle_kind == VehicleSafetyKind::FixedWing && vehicle_class == VehicleClass::UAV_FIXED_WING) ||
                (vehicle_kind == VehicleSafetyKind::Vtol && vehicle_class == VehicleClass::UAV_VTOL) ||
                (vehicle_kind == VehicleSafetyKind::Ground && vehicle_class == VehicleClass::UGV_DIFFERENTIAL);
            const bool timeout_order_valid =
                external_command_loss_us > external_command_warn_us;
            const uint64_t hold_budget = timeout_order_valid
                                             ? external_command_loss_us -
                                                   external_command_warn_us
                                             : 0;
            const bool external_action_valid =
                external_loss_action == SafetyAction::Stabilize ||
                external_loss_action == SafetyAction::HoldOrLoiter ||
                (underwater() && external_loss_action == SafetyAction::ControlledSurface) ||
                external_loss_action == SafetyAction::EmergencyAbort ||
                external_loss_action == SafetyAction::DisableOutput;
            return kind_matches_class && external_command_warn_us > 0 &&
                   timeout_order_valid &&
                   command_hold_max_us > 0 &&
                   command_hold_max_us <= hold_budget &&
                   internal_reference_valid_us > 0 &&
                   std::isfinite(stabilize_duration_s) &&
                   stabilize_duration_s >= 0.0 &&
                   std::isfinite(safe_surge_mps) &&
                   std::isfinite(minimum_control_surge_mps) &&
                   std::isfinite(surface_surge_mps) &&
                   minimum_control_surge_mps >= 0.0 &&
                   safe_surge_mps >= minimum_control_surge_mps &&
                   surface_surge_mps >= 0.0 &&
                   std::isfinite(surge_slew_mps2) &&
                   surge_slew_mps2 > 0.0 &&
                   std::isfinite(yaw_rate_slew_radps2) &&
                   yaw_rate_slew_radps2 > 0.0 &&
                   std::isfinite(surface_reference_rate_mps) &&
                   surface_reference_rate_mps > 0.0 &&
                   std::isfinite(surface_capture_depth_m) &&
                   surface_capture_depth_m >= 0.0 &&
                   std::isfinite(loiter_speed_mps) && loiter_speed_mps > 0.0 &&
                   std::isfinite(loiter_radius_m) && loiter_radius_m > 0.0 &&
                   std::isfinite(station_radius_m) && station_radius_m > 0.0 &&
                   external_action_valid;
        }
    };
    // Selected from the same physical control bundle as the controller/allocator.
    SafetyProfile safety_profile_for(const ControlParameters &params) noexcept;
} // namespace hydrox::safety
