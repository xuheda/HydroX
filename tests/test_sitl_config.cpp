// Copyright (c) 2026 OceanX. Author: xuheda
#include "sitl/sitl_config.h"

#include <cstdio>
#include <algorithm>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
    int expect(bool condition, const char *message)
    {
        if (!condition)
        {
            std::fprintf(stderr, "FAIL: %s\n", message);
            return 1;
        }
        return 0;
    }

    hydrox::sitl::Config parse(std::vector<std::string> arguments)
    {
        if (std::find(arguments.begin(), arguments.end(), "--vehicle-bundle") == arguments.end()) {
            arguments.push_back("--vehicle-bundle");
            arguments.push_back("profiles/eca-a9/vehicle-bundle.json");
        }
        std::vector<char *> argv;
        argv.reserve(arguments.size());
        for (std::string &argument : arguments)
            argv.push_back(argument.data());
        return hydrox::sitl::parse_config(
            static_cast<int>(argv.size()), argv.data());
    }

    bool parse_fails(std::vector<std::string> arguments)
    {
        try
        {
            (void)parse(std::move(arguments));
            return false;
        }
        catch (const std::exception &)
        {
            return true;
        }
    }
}

int main()
{
    using hydrox::AccelMode;
    using hydrox::GNCMode;
    using hydrox::VehicleArchetype;
    using hydrox::VehicleClass;
    using hydrox::runtime::ControlFeedbackSource;
    using hydrox::sitl::ResidualPolicyMode;

    int failures = 0;
    bool missing_rejected = false;
    char exe[] = "hydrox_sitl"; char* no_bundle[] = {exe};
    try { (void)hydrox::sitl::parse_config(1, no_bundle); }
    catch (const std::invalid_argument&) { missing_rejected = true; }
    failures += expect(missing_rejected, "bundle is mandatory");
    failures += expect(parse_fails({"hydrox_sitl", "--vehicle-params", "old.json"}), "old file argument rejected");
    failures += expect(parse_fails({"hydrox_sitl", "--vehicle-params-dir", "old"}), "old directory argument rejected");

    const auto defaults = parse({"hydrox_sitl"});
    failures += expect(defaults.ue5_port == 14600, "default UE5 port");
    failures += expect(defaults.rate_hz == 100, "default control rate");
    failures += expect(defaults.mavlink_system_id == 1,
                       "default MAVLink system id");
    failures += expect(defaults.mission_timeout_s == 0.5,
                       "default SITL command timeout matches HITL");
    failures += expect(
        defaults.residual_policy_mode == ResidualPolicyMode::Disabled,
        "learned residual authority defaults disabled");
    failures += expect(defaults.residual_policy_timeout_ms == 15.0,
                       "loopback policy deadline default");

    failures += expect(defaults.xlog == "auto", "default XLog mode");
    failures += expect(!defaults.publish_truth_state,
                       "truth publishing defaults off");
    failures += expect(!defaults.allow_truth_heading_aid,
                       "truth heading aid defaults off");
    failures += expect(
        defaults.control_feedback_source == ControlFeedbackSource::EstimatedState,
        "control feedback defaults to estimated state");

    const auto configured = parse({
        "hydrox_sitl",
        "--ue5-port", "14605",
        "--ros-domain-id", "17",
        "--vehicle", "vehicle5",
        "--vehicle-type", "LAUV",
        "--vehicle-bundle", "profiles/generic-auv-fin/vehicle-bundle.json",
        "--xlog", "auto",
        "--log-directory", "D:/OceanX/Diagnostics/run42",
        "--run-id", "run42",
        "--rate", "200",
        "--mavlink-signing-key-file", "D:/secure/hil.key",
        "--mavlink-system-id", "37",
        "--mavlink-signing-link-id", "42",
        "--publish-truth-state", "true",
        "--allow-truth-heading-aid", "1",
        "--control-feedback-source", "truth_debug",
        "--residual-policy-mode", "shadow",
        "--residual-policy-host", "127.0.0.1",
        "--residual-policy-port", "14750",
        "--residual-policy-local-port", "14751",
        "--residual-policy-nonce", "20260921",
        "--residual-policy-hz", "10",
        "--residual-policy-timeout-ms", "5",
        "--residual-blend", "0.35",
        "--residual-min-confidence", "0.75",
        "--residual-max-delta", "180,0,90",
        "--residual-max-rate", "600,0,300",
    });
    failures += expect(configured.ue5_port == 14605, "configured UE5 port");
    failures += expect(configured.ros_domain_id == 17, "configured DDS domain");
    failures += expect(configured.vehicle == "vehicle5", "configured vehicle name");
    failures += expect(configured.vehicle_type == "LAUV", "configured vehicle type");
    failures += expect(configured.vehicle_bundle == "profiles/generic-auv-fin/vehicle-bundle.json",
                       "configured vehicle bundle");
    failures += expect(configured.xlog == "auto", "configured XLog mode");
    failures += expect(
        configured.log_directory == "D:/OceanX/Diagnostics/run42" && configured.run_id == "run42",
        "configured automatic diagnostics directory and run id");
    failures += expect(configured.rate_hz == 200, "configured control rate");
    failures += expect(configured.mavlink_signing_key_file == "D:/secure/hil.key",
                       "configured MAVLink signing key file");
    failures += expect(configured.mavlink_system_id == 37,
                       "configured MAVLink system id");
    failures += expect(configured.mavlink_signing_link_id == 42,
                       "configured MAVLink signing link id");
    failures += expect(configured.publish_truth_state, "truth publishing flag");
    failures += expect(configured.allow_truth_heading_aid, "truth heading aid flag");
    failures += expect(
        configured.control_feedback_source == ControlFeedbackSource::TruthDebug,
        "explicit truth debug control feedback");
    failures += expect(
        configured.residual_policy_mode == ResidualPolicyMode::Shadow &&
        configured.residual_policy_port == 14750 &&
        configured.residual_policy_local_port == 14751 &&
        configured.residual_policy_nonce == 20260921 &&
        configured.residual_policy_hz == 10 &&
        configured.residual_max_delta[0] == 180.0 &&
        configured.residual_max_delta[2] == 90.0,
        "configured local residual sidecar contract");

    failures += expect(parse_fails({"hydrox_sitl", "--rate"}),
                       "missing option value is rejected");
    failures += expect(parse_fails({"hydrox_sitl", "--unknown", "value"}),
                       "unknown option is rejected");
    failures += expect(parse_fails({"hydrox_sitl", "--ue5-port", "70000"}),
                       "out-of-range port is rejected");
    failures += expect(parse_fails({"hydrox_sitl", "--ros-domain-id", "233"}),
                       "out-of-range DDS domain is rejected");
    failures += expect(parse_fails({"hydrox_sitl", "--rate", "0"}),
                       "non-positive rate is rejected");
    failures += expect(parse_fails({"hydrox_sitl", "--mission-timeout", "0"}),
                       "disabled command timeout is rejected like HITL");
    failures += expect(parse_fails({"hydrox_sitl", "--mission-timeout", "nan"}),
                       "non-finite command timeout is rejected like HITL");
    failures += expect(parse_fails({"hydrox_sitl", "--time-mode", "hil"}),
                       "removed time-mode option is rejected");
    failures += expect(parse_fails({"hydrox_sitl", "--mavlink-signing-link-id", "256"}),
                       "out-of-range MAVLink signing link id is rejected");
    failures += expect(
        parse_fails({"hydrox_sitl", "--mavlink-system-id", "0"}) &&
            parse_fails({"hydrox_sitl", "--mavlink-system-id", "256"}),
        "out-of-range MAVLink system id is rejected");
    failures += expect(
        parse_fails({"hydrox_sitl", "--control-feedback-source", "automatic"}),
        "unknown control feedback source is rejected");
    failures += expect(
        parse_fails({"hydrox_sitl", "--residual-policy-mode", "automatic"}),
        "unknown residual policy mode is rejected");
    failures += expect(
        parse_fails({
            "hydrox_sitl", "--residual-policy-mode", "shadow",
            "--residual-policy-host", "0.0.0.0",
            "--residual-policy-port", "14750",
            "--residual-policy-local-port", "14751",
            "--residual-policy-nonce", "1"}),
        "non-loopback residual policy host is rejected");
    failures += expect(
        parse_fails({
            "hydrox_sitl", "--residual-policy-mode", "active",
            "--residual-policy-port", "14750",
            "--residual-policy-local-port", "14750",
            "--residual-policy-nonce", "1"}),
        "colliding residual sidecar ports are rejected");
    failures += expect(
        parse_fails({
            "hydrox_sitl", "--residual-policy-mode", "shadow",
            "--residual-policy-port", "14750",
            "--residual-policy-local-port", "14751",
            "--residual-policy-nonce", "0"}),
        "zero residual session nonce is rejected");
    failures += expect(
        parse_fails({
            "hydrox_sitl", "--residual-policy-mode", "shadow",
            "--residual-policy-port", "14750",
            "--residual-policy-local-port", "14751",
            "--residual-policy-nonce", "1",
            "--residual-policy-hz", "30"}),
        "non-divisor residual policy rate is rejected");

    failures += expect(
        hydrox::sitl::gnc_mode_from_string("WAYPOINT_3D") == GNCMode::WAYPOINT_3D,
        "GNC mode parsing");
    failures += expect(
        std::string(hydrox::sitl::gnc_mode_name(GNCMode::DP)) == "DP",
        "GNC mode naming");
    failures += expect(
        hydrox::sitl::accel_mode_from_string("off") == AccelMode::Off,
        "accelerometer mode parsing");
    failures += expect(
        hydrox::sitl::mav_type_for_vehicle_class(VehicleClass::USV) == 11,
        "USV MAVLink type");
    failures += expect(
        std::string(hydrox::sitl::vehicle_archetype_name(VehicleArchetype::Thruster)) ==
            "Thruster",
        "vehicle archetype naming");

    failures += expect(parse_fails({"hydrox_sitl", "--xlog", "off"}), "diagnostics cannot silently be disabled");
    failures += expect(parse_fails({"hydrox_sitl", "--xlog-control", "obsolete.control"}), "removed UI control option rejected");
    if (failures == 0)
        std::printf("test_sitl_config: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
