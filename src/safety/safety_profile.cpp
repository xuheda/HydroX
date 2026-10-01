#include "hydrox/safety/safety_profile.h"
#include "control_parameters.h"
#include <algorithm>
#include <cmath>

namespace hydrox::safety
{
SafetyProfile safety_profile_for(const ControlParameters &p) noexcept
{
    SafetyProfile result{};
    result.vehicle_class = p.vehicle_class;
    result.external_loss_action = p.vehicle_class == VehicleClass::UUV
        ? SafetyAction::ControlledSurface : SafetyAction::HoldOrLoiter;
    if (p.archetype != VehicleArchetype::SlenderBodyFin)
    {
        result.minimum_control_surge_mps = 0.0;
        result.safe_surge_mps = 0.0;
        result.surface_surge_mps = 0.0;
    }
    switch (p.archetype)
    {
    case VehicleArchetype::SlenderBodyFin:
        result.vehicle_kind = VehicleSafetyKind::FinAuv;
        // Fins still require flow while climbing; do not decelerate to ROV speed.
        result.surface_surge_mps = result.minimum_control_surge_mps;
        break;
    case VehicleArchetype::Thruster:
        result.vehicle_kind = VehicleSafetyKind::ThrusterUuv;
        // Gains do not prove allocator controllability (e.g. a 3-thruster ROV
        // cannot generate sway). Use heave/surge/yaw protection for this family.
        result.position_hold_capable = false;
        break;
    case VehicleArchetype::Surface:
        result.vehicle_kind = VehicleSafetyKind::SurfaceVessel;
        result.position_hold_capable = true; // bounded-radius twin-screw station keeping
        break;
    case VehicleArchetype::Multirotor:
        result.vehicle_kind = VehicleSafetyKind::Multirotor;
        result.position_hold_capable = true;
        break;
    case VehicleArchetype::FixedWing:
        result.vehicle_kind = VehicleSafetyKind::FixedWing;
        result.loiter_speed_mps = std::max(p.fixedwing_gnc.cruise_speed_mps,
                                         p.fixedwing_gnc.min_flight_speed_mps);
        result.loiter_radius_m = std::max(50.0, 1.5 * result.loiter_speed_mps *
            result.loiter_speed_mps / (9.80665 * std::tan(std::clamp(
                p.fixedwing_gnc.roll_limit_rad, 0.05, 0.5))));
        break;
    case VehicleArchetype::VTOL:
        result.vehicle_kind = VehicleSafetyKind::Vtol;
        result.position_hold_capable = true;
        result.loiter_speed_mps = std::max(p.vtol_gnc.cruise_speed_mps,
                                          p.vtol_gnc.transition_command_min_speed_mps);
        result.loiter_radius_m = std::max(60.0, 1.5 * result.loiter_speed_mps *
            result.loiter_speed_mps / (9.80665 * std::tan(std::clamp(
                p.vtol_gnc.cruise_roll_limit_rad, 0.05, 0.5))));
        break;
    case VehicleArchetype::DifferentialDrive:
        result.vehicle_kind = VehicleSafetyKind::Ground;
        break;
    }
    if (!p.valid || !vehicle_class_matches_archetype(p.vehicle_class, p.archetype))
        result.internal_reference_valid_us = 0; // invalid bundles must not silently use AUV policy
    return result;
}
}
