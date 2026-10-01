#pragma once
/** Fully resolved control parameters. VehicleBundle is the only runtime loader. */

#include "gnc/control_allocator.h"
#include "gnc/fixedwing_allocator.h"
#include "gnc/fixedwing_controller.h"
#include "gnc/ground_allocator.h"
#include "gnc/ground_controller.h"
#include "gnc/motor_model.h"
#include "gnc/gnc_controller.h"
#include "gnc/multirotor_controller.h"
#include "gnc/surface_controller.h"
#include "gnc/thruster_controller.h"
#include "gnc/vtol_allocator.h"
#include "gnc/vtol_controller.h"
#include "types.h"
#include "gnc/thruster_allocator.h"
#include <vector>

#include <string>

namespace hydrox
{

    // Control archetype — selects the IController/IAllocator pair. SlenderBodyFin = torpedo (fins+prop);
    // Thruster = ROV/hovering AUV (thruster array); Surface = USV.
    enum class VehicleArchetype
    {
        SlenderBodyFin,
        Thruster,
        Surface,
        Multirotor,
        FixedWing,
        VTOL,
        DifferentialDrive,
    };

    inline bool vehicle_class_matches_archetype(
        VehicleClass vehicle_class,
        VehicleArchetype archetype) noexcept
    {
        switch (archetype)
        {
        case VehicleArchetype::SlenderBodyFin:
        case VehicleArchetype::Thruster:
            return vehicle_class == VehicleClass::UUV;
        case VehicleArchetype::Surface:
            return vehicle_class == VehicleClass::USV;
        case VehicleArchetype::Multirotor:
            return vehicle_class == VehicleClass::UAV_MULTIROTOR;
        case VehicleArchetype::FixedWing:
            return vehicle_class == VehicleClass::UAV_FIXED_WING;
        case VehicleArchetype::VTOL:
            return vehicle_class == VehicleClass::UAV_VTOL;
        case VehicleArchetype::DifferentialDrive:
            return vehicle_class == VehicleClass::UGV_DIFFERENTIAL;
        }
        return false;
    }


    struct ControlParameters
    {
        bool valid = false;
        // Explicit logical actuator layout; never inferred from the vehicle name.
        std::vector<Thruster> thrusters;
        bool direct_body_wrench = false;
        VehicleArchetype archetype = VehicleArchetype::SlenderBodyFin;
        VehicleClass vehicle_class = VehicleClass::UUV;
        std::string vehicle_type;
        std::string source_path;
        FinAllocator::Params allocator;
        MotorModel::Params motor;
        SlenderBodyAUVController::Params gnc;
        ThrusterVehicleController::Params thruster_gnc;
        SurfaceVesselController::Params surface_gnc;
        MultirotorController::Params multirotor_gnc;
        FixedWingController::Params fixedwing_gnc;
        FixedWingAllocator::Params fixedwing_allocator;
        VtolController::Params vtol_gnc;
        VtolAllocator::Params vtol_allocator;
        DifferentialDriveController::Params ground_gnc;
        DifferentialDriveAllocator::Params ground_allocator;

        // Thruster/surface vehicles: per-thruster saturation (N). Fin vehicles leave 0.
        double max_thrust_per_thruster_N = 0.0;

        // Surface vehicles may group several physical engines behind a single
        // HydroX channel (for example VRX WAM-V's two engines per side).
        // This is the effective surge force and yaw lever of one such channel.
        double surface_channel_surge_limit_N = 0.0;
        double surface_channel_lever_arm_m = 0.395;

        // Air vehicles: physical sum of all vertical lift rotors at full command.
        // This must match the UE Aero kT/omega_max model. The allocator mixes
        // physical thrust and emits normalized thrust at the actuator boundary.
        double max_total_lift_N = 0.0;
        double lift_roll_pitch_moment_arm_m = 0.0;
        double lift_yaw_moment_per_thrust_m = 0.0;

        // Resolved control-model inertia; gains in the bundle are already physical.
        double M44_pitch = 0.0;  // Fin AUV pitch inertia incl. added mass (kg·m²)
        double mass_total = 0.0; // surge mass incl. added mass (kg)
    };

} // namespace hydrox
