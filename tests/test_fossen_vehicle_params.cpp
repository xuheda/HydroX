// Copyright (c) 2026 OceanX. Author: xuheda
/** Legacy explicit-parameter loader tests. */
#include "fossen_vehicle_params.h"

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
    failures += expect(hydrox::canonical_vehicle_type("eca-a9") == "EcaA9",
                       "legacy aliases remain supported");
    failures += expect(hydrox::archetype_for("RexROV2") == VehicleArchetype::Thruster,
                       "legacy types retain their control archetype");

    const auto fallback = hydrox::builtin_fossen_control_params("EcaA9");
    failures += expect(fallback.valid && !fallback.loaded_from_json,
                       "built-in legacy fallback remains valid");
    failures += expect(approx(fallback.allocator.S_fin, 0.04155),
                       "legacy fallback retains fin authority");

    const auto quad = hydrox::builtin_fossen_control_params("X500");
    failures += expect(quad.valid && quad.vehicle_class == VehicleClass::UAV_MULTIROTOR,
                       "aircraft fallback retains vehicle class");

    // No implicit simulator workspace lookup is permitted. A legacy file must
    // be supplied by --vehicle-params or --vehicle-params-dir.
    std::string error;
    const auto explicit_only = hydrox::load_fossen_control_params("EcaA9", {}, {}, &error);
    failures += expect(explicit_only.valid && !explicit_only.loaded_from_json,
                       "missing explicit legacy input uses the local fallback");
    failures += expect(!error.empty(), "missing explicit legacy input is reported");

    const auto explicit_file = hydrox::load_fossen_control_params(
        "EcaA9", "tests/fixtures/legacy_auv_params.json", {}, &error);
    failures += expect(error.empty() && explicit_file.valid && explicit_file.loaded_from_json,
                       "explicit legacy file loads without a simulator workspace");
    failures += expect(approx(explicit_file.allocator.max_thrust_N, 180.0),
                       "explicit legacy file overrides thrust authority");
    failures += expect(approx(explicit_file.allocator.n_max_rpm, 3200.0),
                       "explicit legacy file overrides propeller speed");

    const auto cessna = hydrox::load_fossen_control_params(
        "RCCessna", {}, "../engine/Content/Aero", &error);
    failures += expect(error.empty() && cessna.valid && cessna.loaded_from_json,
                       "Cessna profile loads its explicit control tuning");
    failures += expect(approx(cessna.fixedwing_gnc.pitch_trim, 0.21),
                       "Cessna profile overrides the fixed-wing pitch trim");
    failures += expect(approx(cessna.fixedwing_gnc.altitude_ki, 0.004),
                       "Cessna profile enables bounded altitude integral compensation");
    failures += expect(approx(cessna.fixedwing_gnc.max_pitch_command, 0.30),
                       "Cessna profile overrides the fixed-wing pitch command limit");
    failures += expect(approx(cessna.fixedwing_allocator.throttle_trim, 0.42),
                       "Cessna profile overrides the fixed-wing throttle trim");

    const auto wamv = hydrox::load_fossen_control_params(
        "WAMV", {}, "../engine/Content/Fossen", &error);
    failures += expect(error.empty() && wamv.valid && wamv.loaded_from_json,
                       "WAM-V profile loads its explicit surface-control tuning");
    failures += expect(approx(wamv.surface_gnc.surge_kp, 500.0),
                       "WAM-V profile overrides the surface speed-loop gain");
    failures += expect(approx(wamv.surface_gnc.surge_ki, 80.0),
                       "WAM-V profile enables integral speed compensation");

    auto normalized = fallback;
    normalized.M44_pitch = 12.84;
    normalized.mass_total = 35.75;
    auto controller = normalized.gnc;
    const auto before = controller;
    hydrox::apply_inertia_normalized_gains(controller, normalized);
    failures += expect(approx(controller.pitch.kp, before.pitch.kp * 0.5, 1e-8),
                       "pitch gain normalizes with supplied inertia");
    failures += expect(approx(controller.surge.kp, before.surge.kp * 0.5, 1e-8),
                       "surge gain normalizes with supplied mass");

    if (failures == 0)
        std::printf("test_fossen_vehicle_params: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
