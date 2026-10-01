#include "gnc/control_factory.h"
#include "vehicle_bundle.h"
#include "bundle_parity_scenario.h"
#include "hydrox/runtime/hitl_profile_registry.h"
#include "hydrox/safety/safety_profile.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <set>
#include <string>

namespace
{
int expect(bool condition, const char *message)
{
    if (condition)
        return 0;
    std::cerr << "FAIL: " << message << '\n';
    return 1;
}
}

int main()
{
    using namespace hydrox::runtime;
    int failures = 0;
    failures += expect(compiled_hitl_profile_count() == 10,
                       "all ten production profiles are compiled");

    std::set<std::string> ids;
    std::set<uint64_t> fingerprints;
    for (std::size_t index = 0; index < compiled_hitl_profile_count(); ++index)
    {
        HitlProfileIdentity identity{};
        failures += expect(compiled_hitl_profile_identity(index, identity),
                           "registry index resolves");
        if (identity.profile_id == nullptr)
            continue;
        failures += expect(ids.insert(identity.profile_id).second,
                           "profile IDs are unique");
        failures += expect(fingerprints.insert(identity.fingerprint).second,
                           "profile fingerprints are unique");
        failures += expect(identity.fingerprint != 0,
                           "profile fingerprint is non-zero");

        HitlVehicleProfile profile{};
        std::string error;
        failures += expect(load_compiled_hitl_profile(
                               identity.profile_id,
                               identity.fingerprint,
                               0x12340000u + static_cast<uint32_t>(index),
                               profile,
                               &error),
                           error.empty() ? "profile loads" : error.c_str());
        failures += expect(
            std::strcmp(profile.profile_id.data(), identity.profile_id) == 0,
            "loaded profile identity matches registry");
        failures += expect(profile.bundle_fingerprint == identity.fingerprint,
                           "loaded profile fingerprint matches registry");
        failures += expect(profile.mav_type == identity.mav_type,
                           "loaded profile MAV type matches registry");
        failures += expect(profile.selection_nonce ==
                               0x12340000u + static_cast<uint32_t>(index),
                           "selection nonce survives profile construction");
        const auto source = hydrox::load_vehicle_bundle(profile.control.source_path.substr(9), &error);
        failures += expect(source.valid && source.fingerprint == identity.fingerprint,
                           "embedded and source bundle bytes match");
        if (source.valid)
            failures += expect(hydrox::test::bundle_parity_sequence(hydrox::build_control_stack(source)) ==
                               hydrox::test::bundle_parity_sequence(hydrox::build_control_stack(profile.control)),
                               "SITL and compiled HITL layouts and control outputs match");
        if (source.valid)
        {
            // Same runtime fault sequence, independently constructed from file and firmware profiles.
            auto sitl_config = profile.runtime;
            sitl_config.safety_profile = hydrox::safety::safety_profile_for(source.control);
            auto hitl_config = profile.runtime;
            sitl_config.control_feedback_source = hitl_config.control_feedback_source =
                ControlFeedbackSource::TruthDebug; // Isolate authority parity from sensor physics.
            auto sitl_stack = hydrox::build_control_stack(source);
            auto hitl_stack = hydrox::build_control_stack(profile.control);
            HilRuntime sitl(sitl_config, std::move(sitl_stack.controller), std::move(sitl_stack.allocator));
            HilRuntime hitl(hitl_config, std::move(hitl_stack.controller), std::move(hitl_stack.allocator));
            sitl.on_connected(1'000'000);
            hitl.on_connected(1'000'000);
            hydrox::GNCSetpoint target{};
            const bool air = source.control.vehicle_class == hydrox::VehicleClass::UAV_MULTIROTOR ||
                source.control.vehicle_class == hydrox::VehicleClass::UAV_FIXED_WING ||
                source.control.vehicle_class == hydrox::VehicleClass::UAV_VTOL;
            target.depth_ref = target.wp_d = air ? -20.0 : 10.0;
            target.surge_ref = 2.0;
            target.wp_n = 100.0;
            for (uint64_t i = 1; i <= 620; ++i)
            {
                const uint64_t now = 1'000'000 + i * 10'000;
                hydrox::NavigationInput sample{};
                sample.got_imu = true;
                sample.imu.time_usec = now;
                sample.truth_valid = sample.truth.valid = true;
                for (auto &value : sample.truth.eta) value = 0.0;
                for (auto &value : sample.truth.nu) value = 0.0;
                sample.truth.eta[2] = target.depth_ref;
                sample.truth.nu[0] = 2.0;
                failures += expect(sitl.step(sample, now) == StepStatus::OK &&
                                   hitl.step(sample, now) == StepStatus::OK, "shared safety tick");
                if (i < 20 || i == 600)
                {
                    failures += expect(sitl.accept_setpoint(target, hydrox::GNCMode::WAYPOINT_3D, now) &&
                                       hitl.accept_setpoint(target, hydrox::GNCMode::WAYPOINT_3D, now),
                                       "both deployments buffer external references");
                }
                if (i == 601)
                    failures += expect(sitl.safety_status().operator_ack_required &&
                                       hitl.safety_status().operator_ack_required &&
                                       sitl.resume_external_control(now + 1) &&
                                       hitl.resume_external_control(now + 1),
                                       "both deployments require and accept deliberate recovery");
                const auto &a = sitl.last_tick();
                const auto &b = hitl.last_tick();
                failures += expect(a.safety_status.mode == b.safety_status.mode &&
                                   a.safety_status.reason == b.safety_status.reason &&
                                   a.safety_status.operator_ack_required == b.safety_status.operator_ack_required &&
                                   a.actuator_authorized == b.actuator_authorized &&
                                   a.safety_control_source == b.safety_control_source &&
                                   a.actuator.ch == b.actuator.ch,
                                   "SITL and firmware profiles have identical safety decisions and outputs");
                if (i > 20 && i < 600)
                    failures += expect(a.actuator_authorized && b.actuator_authorized,
                                       "onboard fallback must retain valid control output");
            }
            failures += expect(sitl.maintain(8'000'000) == RuntimeEvent::SENSOR_TIMEOUT &&
                               hitl.maintain(8'000'000) == RuntimeEvent::SENSOR_TIMEOUT &&
                               !sitl.last_tick().actuator_authorized && !hitl.last_tick().actuator_authorized,
                               "both deployments inhibit output on sensor loss");
        }
        auto stack = hydrox::build_control_stack(profile.control);
        failures += expect(stack.controller != nullptr && stack.allocator != nullptr,
                           "profile constructs a complete control stack");
    }

    const std::set<std::string> expected = {
        "generic-auv-fin", "EcaA9", "LAUV", "DesistekSaga", "RexROV2",
        "VRX_WAMV", "X500", "RCCessna", "StandardVTOL", "R1Rover"};
    failures += expect(ids == expected, "registry contains every production vehicle");

    HitlVehicleProfile rejected{};
    std::string error;
    failures += expect(!load_compiled_hitl_profile(
                           "X500", 0xDEADBEEFDEADBEEFULL, 1, rejected, &error),
                       "wrong fingerprint is rejected");
    failures += expect(!load_compiled_hitl_profile(
                           "not-a-vehicle", 1, 1, rejected, &error),
                       "unknown profile is rejected");

    if (failures == 0)
        std::cout << "test_hitl_profile_registry: all checks passed\n";
    return failures == 0 ? 0 : 1;
}
