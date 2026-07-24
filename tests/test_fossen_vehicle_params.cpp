// Copyright (c) 2026 OceanX. Author: xuheda
/**
 * test_fossen_vehicle_params.cpp — Contract tests for the shared UE5/HydroX
 * Fossen vehicle-parameter loader.
 */
#include "fossen_vehicle_params.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace
{
    bool approx(double a, double b, double eps = 1e-9)
    {
        return std::abs(a - b) <= eps;
    }

    int expect(bool ok, const char *msg)
    {
        if (!ok)
        {
            std::fprintf(stderr, "FAIL: %s\n", msg);
            return 1;
        }
        return 0;
    }
}

int main()
{
    using hydrox::VehicleArchetype;
    using hydrox::VehicleClass;

    int fails = 0;

    // Vehicle names from CLI/configuration are intentionally forgiving.
    fails += expect(hydrox::canonical_vehicle_type("") == "EcaA9",
                    "empty vehicle type defaults to EcaA9");
    fails += expect(hydrox::canonical_vehicle_type("eca-a9") == "EcaA9",
                    "EcaA9 alias is canonicalized");
    fails += expect(hydrox::canonical_vehicle_type("otter_usv") == "SurfaceVessel",
                    "Otter alias is canonicalized");
    fails += expect(hydrox::canonical_vehicle_type("VRX_WAMV") == "WAMV",
                    "VRX WAM-V selects its dedicated physical profile");
    fails += expect(hydrox::archetype_for("LAUV") == VehicleArchetype::SlenderBodyFin,
                    "LAUV uses fin control");
    fails += expect(hydrox::archetype_for("RexROV2") == VehicleArchetype::Thruster,
                    "RexROV2 uses thruster control");
    fails += expect(hydrox::archetype_for("USV") == VehicleArchetype::Surface,
                    "USV uses surface control");
    fails += expect(hydrox::archetype_for("VRX_WAMV") == VehicleArchetype::Surface,
                    "VRX WAM-V uses surface control");
    fails += expect(hydrox::archetype_for("X500") == VehicleArchetype::Multirotor,
                    "X500 uses multirotor control");
    fails += expect(hydrox::archetype_for("RCCessna") == VehicleArchetype::FixedWing,
                    "RCCessna uses fixed-wing control");
    fails += expect(hydrox::archetype_for("StandardVTOL") == VehicleArchetype::VTOL,
                    "StandardVTOL uses the dedicated VTOL control archetype");

    // Built-in values are a valid fallback if the shared JSON is unavailable.
    {
        const auto eca = hydrox::builtin_fossen_control_params("EcaA9");
        fails += expect(eca.valid, "built-in EcaA9 parameters are valid");
        fails += expect(!eca.loaded_from_json, "built-in EcaA9 is not marked as JSON");
        fails += expect(approx(eca.allocator.S_fin, 0.04155),
                        "built-in EcaA9 fin area");
        fails += expect(approx(eca.allocator.D_prop, eca.motor.D_prop),
                        "built-in propeller diameter is shared by allocator and motor");

        const auto unsupported = hydrox::builtin_fossen_control_params("UnknownVehicle");
        fails += expect(!unsupported.valid, "unknown built-in vehicle is rejected");

        const auto x500 = hydrox::builtin_fossen_control_params("X500");
        fails += expect(x500.valid && x500.vehicle_class == VehicleClass::UAV_MULTIROTOR,
                        "X500 has valid multirotor fallback parameters");
        fails += expect(approx(x500.mass_total, 2.0), "X500 fallback mass matches SDF");
        fails += expect(approx(x500.max_total_lift_N, 34.19432),
                        "X500 allocator thrust matches UE Aero rotor model");

        const auto cessna = hydrox::builtin_fossen_control_params("RCCessna");
        fails += expect(cessna.valid && cessna.vehicle_class == VehicleClass::UAV_FIXED_WING,
                        "RCCessna has valid fixed-wing fallback parameters");
        fails += expect(approx(cessna.mass_total, 1.5), "RCCessna fallback mass matches SDF");

        const auto vtol = hydrox::builtin_fossen_control_params("StandardVTOL");
        fails += expect(vtol.valid && vtol.vehicle_class == VehicleClass::UAV_VTOL,
                        "StandardVTOL has valid VTOL fallback parameters");
        fails += expect(approx(vtol.mass_total, 5.0), "StandardVTOL fallback mass matches SDF");
        fails += expect(approx(vtol.max_total_lift_N, 180.0),
                        "StandardVTOL allocator thrust matches UE Aero lift rotors");
    }

    // CTest runs from the repository root, so these calls must resolve the
    // authoritative files under engine/Content/Fossen.
    std::string error;
    const auto eca = hydrox::load_fossen_control_params("eca-a9", {}, {}, &error);
    fails += expect(error.empty(), "EcaA9 JSON loads without an error");
    fails += expect(eca.valid && eca.loaded_from_json,
                    "EcaA9 loads authoritative JSON parameters");
    fails += expect(eca.source_path.find("eca_a9_params.json") != std::string::npos,
                    "EcaA9 reports its JSON source path");
    fails += expect(approx(eca.allocator.max_thrust_N, 900.0),
                    "EcaA9 JSON overrides maximum thrust");
    fails += expect(approx(eca.allocator.n_max_rpm, 9500.0),
                    "EcaA9 JSON overrides maximum RPM");
    fails += expect(approx(eca.gnc.depth.kp, 0.055),
                    "EcaA9 JSON overrides depth controller");
    fails += expect(eca.yaw_rate_control_loaded_from_json,
                    "EcaA9 JSON yaw-rate controller is detected");
    fails += expect(eca.M44_pitch > 0.0 && eca.mass_total > 0.0,
                    "EcaA9 effective inertia is computed");

    const auto lauv = hydrox::load_fossen_control_params("LAUV", {}, {}, &error);
    fails += expect(error.empty(), "LAUV JSON loads without an error");
    fails += expect(lauv.valid && lauv.loaded_from_json,
                    "LAUV loads authoritative JSON parameters");
    fails += expect(approx(lauv.allocator.delta_max_deg, 25.0),
                    "LAUV JSON overrides fin limit");
    fails += expect(approx(lauv.allocator.u_min, 6.0),
                    "LAUV JSON overrides allocator minimum speed");
    fails += expect(lauv.M44_pitch > 0.0 && lauv.M44_pitch < eca.M44_pitch,
                    "LAUV pitch inertia is positive and below EcaA9");
    fails += expect(lauv.mass_total > 0.0 && lauv.mass_total < eca.mass_total,
                    "LAUV effective surge mass is positive and below EcaA9");

    const auto otter = hydrox::load_fossen_control_params("otter", {}, {}, &error);
    fails += expect(error.empty(), "Otter JSON loads without an error");
    fails += expect(otter.valid && otter.loaded_from_json,
                    "Otter loads authoritative JSON parameters");
    fails += expect(otter.archetype == VehicleArchetype::Surface,
                    "Otter is a surface vehicle");
    fails += expect(otter.vehicle_class == VehicleClass::USV,
                    "Otter vehicle class is USV");
    fails += expect(approx(otter.mass_total, 80.0),
                    "Otter hull and payload mass are combined");
    fails += expect(approx(otter.max_thrust_per_thruster_N, 119.7),
                    "Otter per-motor thrust limit is loaded");
    fails += expect(otter.archetype_control_loaded_from_json,
                    "Otter surface controller loads authoritative tuning");
    fails += expect(approx(otter.surface_gnc.waypoint_surge_mps, 1.5),
                    "Otter waypoint speed is loaded from JSON");

    const auto wamv = hydrox::load_fossen_control_params("VRX_WAMV", {}, {}, &error);
    fails += expect(error.empty(), "WAM-V JSON loads without an error");
    fails += expect(wamv.valid && wamv.loaded_from_json,
                    "WAM-V loads its dedicated physical JSON");
    fails += expect(wamv.source_path.find("wamv_params.json") != std::string::npos,
                    "WAM-V reports its dedicated JSON source path");
    fails += expect(approx(wamv.mass_total, 242.0),
                    "WAM-V includes the four imported engine and propeller assemblies");
    fails += expect(approx(wamv.max_thrust_per_thruster_N, 2352.53),
                    "WAM-V per-engine thrust limit is loaded from VRX");
    fails += expect(approx(wamv.surface_channel_surge_limit_N, 3327.0),
                    "WAM-V virtual control channel preserves X-layout surge authority");
    fails += expect(approx(wamv.surface_channel_lever_arm_m, 2.85),
                    "WAM-V virtual control channel preserves X-layout yaw leverage");

    const auto desistek = hydrox::load_fossen_control_params("DesistekSaga", {}, {}, &error);
    fails += expect(error.empty(), "Desistek JSON loads without an error");
    fails += expect(desistek.valid && desistek.loaded_from_json,
                    "Desistek loads authoritative JSON parameters");
    fails += expect(approx(desistek.max_thrust_per_thruster_N, 30.0),
                    "Desistek thrust cap is loaded");
    fails += expect(desistek.archetype_control_loaded_from_json,
                    "Desistek thruster controller loads authoritative tuning");
    fails += expect(approx(desistek.thruster_gnc.sway.accel_max, 0.0),
                    "Desistek does not request unsupported sway control");

    const auto rexrov2 = hydrox::load_fossen_control_params("RexROV2", {}, {}, &error);
    fails += expect(error.empty(), "RexROV2 JSON loads without an error");
    fails += expect(rexrov2.valid && rexrov2.loaded_from_json,
                    "RexROV2 loads authoritative JSON parameters");
    fails += expect(approx(rexrov2.max_thrust_per_thruster_N, 1800.0),
                    "RexROV2 conservative thrust cap is loaded");
    fails += expect(approx(rexrov2.thruster_gnc.heave.accel_max, 0.32),
                    "RexROV2 heave acceleration limit is loaded");

    const auto x500_json = hydrox::load_fossen_control_params("X500", {}, {}, &error);
    fails += expect(error.empty(), "X500 JSON loads without an error");
    fails += expect(x500_json.valid && x500_json.loaded_from_json,
                    "X500 loads authoritative Aero JSON parameters");
    fails += expect(x500_json.source_path.find("x500_params.json") != std::string::npos,
                    "X500 reports its Aero JSON source path");
    fails += expect(approx(x500_json.max_total_lift_N, 34.19432),
                    "X500 JSON lift cap matches the UE Aero model");
    fails += expect(approx(x500_json.multirotor_gnc.max_tilt_rad, 0.32),
                    "X500 tilt limit is loaded from JSON");

    const auto cessna_json = hydrox::load_fossen_control_params("RCCessna", {}, {}, &error);
    fails += expect(error.empty(), "RCCessna JSON loads without an error");
    fails += expect(cessna_json.valid && cessna_json.loaded_from_json,
                    "RCCessna loads authoritative Aero JSON parameters");
    fails += expect(approx(cessna_json.fixedwing_gnc.roll_limit_rad, 0.40),
                    "RCCessna bank limit is loaded from JSON");

    const auto vtol_json = hydrox::load_fossen_control_params("StandardVTOL", {}, {}, &error);
    fails += expect(error.empty(), "StandardVTOL JSON loads without an error");
    fails += expect(vtol_json.valid && vtol_json.loaded_from_json,
                    "StandardVTOL loads authoritative Aero JSON parameters");
    fails += expect(approx(vtol_json.max_total_lift_N, 180.0),
                    "StandardVTOL JSON lift cap matches the UE Aero model");
    fails += expect(approx(vtol_json.vtol_gnc.cruise_speed_mps, 11.0),
                    "StandardVTOL cruise speed is loaded from JSON");

    // Vehicle-specific JSON yaw-rate gains must not be normalized twice, while
    // pitch and surge gains follow the computed inertia ratios.
    {
        auto gnc = lauv.gnc;
        const auto before = gnc;
        hydrox::apply_inertia_normalized_gains(gnc, lauv);

        const double torque_ratio = lauv.M44_pitch / 25.68;
        const double force_ratio = lauv.mass_total / 71.5;
        fails += expect(approx(gnc.pitch.kp, before.pitch.kp * torque_ratio, 1e-8),
                        "pitch gain is normalized by pitch inertia");
        fails += expect(approx(gnc.surge.kp, before.surge.kp * force_ratio, 1e-8),
                        "surge gain is normalized by effective mass");
        fails += expect(approx(gnc.yaw_rate.kp, before.yaw_rate.kp),
                        "JSON yaw-rate gain is not normalized twice");
    }

    if (fails == 0)
        std::printf("test_fossen_vehicle_params: all checks passed\n");
    return fails == 0 ? 0 : 1;
}
