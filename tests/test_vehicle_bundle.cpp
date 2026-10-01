#include "../src/bundle_document.h"
#include "bundle_parity_scenario.h"
#include "control_parameter_fields.h"
#include "hydrox/runtime/generic_auv_hitl_profile.h"
#include "vehicle_bundle.h"
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <type_traits>
using namespace hydrox;
namespace
{
int failures = 0;
void check(bool ok, const std::string &what)
{
    if (!ok)
    {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", what.c_str());
    }
}
std::string read(const std::string &path)
{
    std::ifstream f(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(f), {}};
}
std::string encode(const bundle_json::Value &v)
{
    std::ostringstream s;
    s << std::setprecision(17);
    switch (v.kind)
    {
    case bundle_json::Value::Object: {
        s << '{';
        bool first = true;
        for (const auto &p : v.object)
        {
            if (!first)
                s << ',';
            first = false;
            s << std::quoted(p.first) << ':' << encode(p.second);
        }
        s << '}';
        break;
    }
    case bundle_json::Value::Array: {
        s << '[';
        bool first = true;
        for (const auto &x : v.array)
        {
            if (!first)
                s << ',';
            first = false;
            s << encode(x);
        }
        s << ']';
        break;
    }
    case bundle_json::Value::String:
        s << std::quoted(v.string);
        break;
    case bundle_json::Value::Number:
        s << v.number;
        break;
    case bundle_json::Value::Boolean:
        s << "true";
        break;
    default:
        s << "null";
        break;
    }
    return s.str();
}
bool close(double a, double b)
{
    return std::isfinite(a) && std::abs(a - b) <= 1e-7 * std::max(1.0, std::abs(b));
}
} // namespace
int main()
{
    const char *folders[] = {"eca-a9",
                             "lauv",
                             "eca-a9-test",
                             "lauv-test",
                             "desistek-saga",
                             "rexrov2",
                             "bluffbody-rov",
                             "otter",
                             "wamv",
                             "x500",
                             "rc-cessna",
                             "standard-vtol",
                             "generic-auv-fin",
                             "generic-fixed-wing",
                             "generic-quad-x",
                             "generic-rov-6dof",
                             "generic-usv-twin-prop",
                             "generic-vtol-lift-cruise",
                             "r1-rover"};
    for (const char *folder : folders)
    {
        const std::string path = "profiles/" + std::string(folder) + "/vehicle-bundle.json";
        std::string error;
        const std::string json = read(path);
        auto b = load_vehicle_bundle(path, &error);
        check(b.valid && error.empty(), path + ": " + error);
        if (!b.valid)
            continue;
        const std::string golden =
            read("tests/fixtures/bundle_migration/" + std::string(folder) + ".json");
        bundle_json::Document fixture(golden);
        check(fixture.error.empty(), "golden JSON");
        visit_control_parameters(b.control, [&](const char *key, const auto &value) {
            const auto it = fixture.root.object["fields"].object.find(key);
            check(it != fixture.root.object["fields"].object.end(),
                  std::string(folder) + ": golden field " + key);
            if (it == fixture.root.object["fields"].object.end())
                return;
            if constexpr (std::is_arithmetic_v<std::decay_t<decltype(value)>>)
            {
                double expected_parameter = it->second.number;
                // Interpret the immutable historical snapshot, not runtime input.
                // Its negative VTOL overrides used the old shared gain.
                const std::string field(key);
                if (b.control.archetype == VehicleArchetype::VTOL && expected_parameter < 0.0 &&
                    (field == "control_model.allocator.elevator_gain_inv_Nm" ||
                     field == "control_model.allocator.aileron_gain_inv_Nm" ||
                     field == "control_model.allocator.rudder_gain_inv_Nm"))
                    expected_parameter = fixture.root.object.at("fields").object.at(
                        "control_model.allocator.surface_gain_inv_Nm").number;
                check(close(value, expected_parameter), std::string(folder) + ": field " + key);
            }
            else
                for (size_t i = 0; i < value.size(); ++i)
                    check(close(value[i], it->second.array[i].number), key);
            bundle_json::Document malformed(json);
            malformed.at(key)->kind = bundle_json::Value::Null;
            check(!load_vehicle_bundle_json(encode(malformed.root), "invalid", &error).valid,
                  std::string("reject invalid ") + key);
            bundle_json::Document missing(json);
            const std::string k(key);
            const auto dot = k.rfind('.');
            missing.at(k.substr(0, dot))->object.erase(k.substr(dot + 1));
            check(!load_vehicle_bundle_json(encode(missing.root), "missing", &error).valid,
                  std::string("require ") + key);
        });
        const auto output = test::bundle_parity_sequence(build_control_stack(b));
        const auto &expected = fixture.root.object["output"].array;
        check(output.size() == expected.size(), path + ": parity sample count");
        for (size_t i = 0; i < std::min(output.size(), expected.size()); ++i)
            if (!close(output[i], expected[i].number))
            {
                check(false, path + ": control output differs at " + std::to_string(i));
                break;
            }
        // Identity is a label, never a selector for controller gains or layout.
        auto renamed = b;
        renamed.control.vehicle_type = "unregistered-label";
        const auto other = test::bundle_parity_sequence(build_control_stack(renamed));
        check(output == other, path + ": runtime must not infer parameters from names");
        for (const auto &extra :
             {"\"old_control\":{},", "\"notes\":null,", "\"id\":\"duplicate\","})
        {
            std::string invalid = json;
            invalid.insert(invalid.find('{') + 1, extra);
            check(!load_vehicle_bundle_json(invalid, "unknown", &error).valid,
                  path + ": reject extra/duplicate");
        }
        bundle_json::Document old(json);
        old.at("schema_version")->string = "1.0";
        check(!load_vehicle_bundle_json(encode(old.root), "old", &error).valid,
              "schema 1 must fail");
        bundle_json::Document bad_contract(json);
        bad_contract.at("control_contract")->string = "wrong";
        check(!load_vehicle_bundle_json(encode(bad_contract.root), "contract", &error).valid,
              "wrong contract fails");
        if (b.control.archetype != VehicleArchetype::SlenderBodyFin)
        {
            bundle_json::Document inapplicable(json);
            auto &pitch = inapplicable.at("control_model")->object["pitch_inertia_kg_m2"];
            pitch.kind = bundle_json::Value::Number;
            pitch.number = 0.0;
            check(!load_vehicle_bundle_json(encode(inapplicable.root), "inapplicable", &error).valid &&
                      error.find("pitch_inertia_kg_m2") != std::string::npos,
                  path + ": reject fin-only inertia on other archetypes");
        }
        if (b.control.archetype == VehicleArchetype::VTOL)
        {
            for (const auto *axis : {"elevator", "aileron", "rudder"})
            {
                const std::string field =
                    "control_model.allocator." + std::string(axis) + "_gain_inv_Nm";
                bundle_json::Document negative(json);
                negative.at(field)->number = -1.0;
                check(!load_vehicle_bundle_json(encode(negative.root), "negative", &error).valid &&
                          error.find(field) != std::string::npos,
                      path + ": reject negative " + field);
                negative.at(field)->number = 0.0;
                check(load_vehicle_bundle_json(encode(negative.root), "disabled", &error).valid,
                      path + ": allow explicitly disabled " + field);
            }
            bundle_json::Document shared(json);
            auto &gain = shared.at("control_model.allocator")->object["surface_gain_inv_Nm"];
            gain.kind = bundle_json::Value::Number;
            gain.number = 0.18;
            check(!load_vehicle_bundle_json(encode(shared.root), "shared", &error).valid &&
                      error.find("surface_gain_inv_Nm") != std::string::npos,
                  path + ": reject removed shared surface gain");
        }
    }
    std::string error;
    check(!load_vehicle_bundle("does-not-exist.json", &error).valid && !error.empty(),
          "no file fallback");
    check(!load_vehicle_bundle_json("{\"x\":1e309}", "nonfinite", &error).valid,
          "reject nonfinite");
    const auto auv = load_vehicle_bundle(runtime::generic_auv_hitl_profile::kBundlePath);
    check(auv.fingerprint == runtime::generic_auv_hitl_profile::kBundleFingerprint,
          "compiled fingerprint");
    auto invalid_geometry = auv;
    invalid_geometry.control.allocator.fin_angles_deg.fill(0);
    check(!validate_vehicle_bundle(invalid_geometry, &error), "reject degenerate fin geometry");
    auto invalid_motor = auv;
    invalid_motor.control.motor.tau_m = 0;
    check(!validate_vehicle_bundle(invalid_motor, &error), "reject zero motor time constant");
    const auto rov = load_vehicle_bundle("profiles/generic-rov-6dof/vehicle-bundle.json");
    check(rov.control.thrusters.size() == 8 && rov.allocation_rank == 6, "explicit ROV geometry");
    std::printf(
        "VehicleBundle: 19 migration baselines, strict fields and control parity; failures=%d\n",
        failures);
    return failures ? 1 : 0;
}
