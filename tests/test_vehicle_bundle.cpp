#include "gnc/control_factory.h"
#include "vehicle_bundle.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace
{
    bool approx(double a, double b, double epsilon = 1e-9)
    {
        return std::abs(a - b) <= epsilon;
    }

    int expect(bool condition, const char *message)
    {
        if (!condition)
        {
            std::fprintf(stderr, "FAIL: %s\n", message);
            return 1;
        }
        return 0;
    }
}

int main()
{
    using hydrox::VehicleArchetype;
    using hydrox::VehicleClass;

    int failures = 0;
    std::string error;

    const auto auv = hydrox::load_vehicle_bundle(
        "profiles/generic-auv-fin/vehicle-bundle.json", &error);
    failures += expect(error.empty() && auv.valid, "generic AUV bundle loads and validates");
    failures += expect(auv.fingerprint != 0,
                       "valid bundle exposes a deployment fingerprint");
    failures += expect(auv.control.archetype == VehicleArchetype::SlenderBodyFin,
                       "generic AUV selects fin control");
    failures += expect(approx(auv.control.allocator.max_thrust_N, 250.0),
                       "generic AUV reads explicit propeller authority");

    const auto rov = hydrox::load_vehicle_bundle(
        "profiles/generic-rov-6dof/vehicle-bundle.json", &error);
    failures += expect(error.empty() && rov.valid, "generic ROV bundle loads and validates");
    failures += expect(rov.control.archetype == VehicleArchetype::Thruster,
                       "generic ROV selects thruster control");
    failures += expect(rov.thrusters.size() == 8,
                       "generic ROV owns an eight-thruster logical layout");
    failures += expect(rov.logical_actuator_count == 8 && rov.allocation_rank == 6,
                       "generic ROV has a full-rank six-DOF allocation layout");
    const auto rov_stack = hydrox::build_control_stack(rov);
    failures += expect(rov_stack.controller != nullptr && rov_stack.allocator != nullptr,
                       "generic ROV layout builds a controller and allocator");

    const auto quad = hydrox::load_vehicle_bundle(
        "profiles/generic-quad-x/vehicle-bundle.json", &error);
    failures += expect(error.empty() && quad.valid, "generic quad bundle loads and validates");
    failures += expect(quad.control.vehicle_class == VehicleClass::UAV_MULTIROTOR,
                       "generic quad declares a multirotor vehicle class");
    failures += expect(approx(quad.control.max_total_lift_N, 34.0),
                       "generic quad reads total lift authority");

    const auto usv = hydrox::load_vehicle_bundle(
        "profiles/generic-usv-twin-prop/vehicle-bundle.json", &error);
    failures += expect(error.empty() && usv.valid, "generic USV bundle loads and validates");
    failures += expect(approx(usv.control.surface_channel_lever_arm_m, 0.55),
                       "generic USV reads channel geometry");
    failures += expect(approx(usv.control.surface_gnc.waypoint_surge_mps, 2.0),
                       "generic USV reads surface controller tuning");

    const auto fixed_wing = hydrox::load_vehicle_bundle(
        "profiles/generic-fixed-wing/vehicle-bundle.json", &error);
    failures += expect(error.empty() && fixed_wing.valid,
                       "generic fixed-wing bundle loads and validates");
    failures += expect(approx(fixed_wing.control.fixedwing_gnc.cruise_speed_mps, 18.0),
                       "generic fixed-wing reads controller tuning");

    const auto vtol = hydrox::load_vehicle_bundle(
        "profiles/generic-vtol-lift-cruise/vehicle-bundle.json", &error);
    failures += expect(error.empty() && vtol.valid, "generic VTOL bundle loads and validates");
    failures += expect(approx(vtol.control.vtol_gnc.cruise_speed_mps, 15.0),
                       "generic VTOL reads controller tuning");

    const auto missing = hydrox::load_vehicle_bundle("profiles/does-not-exist.json", &error);
    failures += expect(!missing.valid && !error.empty(), "missing bundle reports an explicit error");
    failures += expect(missing.fingerprint == 0,
                       "missing bundle cannot masquerade as a fingerprinted profile");

    if (failures == 0)
        std::printf("test_vehicle_bundle: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
