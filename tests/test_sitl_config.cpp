// Copyright (c) 2026 OceanX. Author: xuheda
#include "sitl/sitl_config.h"

#include <cstdio>
#include <exception>
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

    int failures = 0;

    const auto defaults = parse({"hydrox_sitl"});
    failures += expect(defaults.ue5_port == 14600, "default UE5 port");
    failures += expect(defaults.rate_hz == 100, "default control rate");
    failures += expect(defaults.time_mode == "hil", "default HIL time mode");
    failures += expect(defaults.xlog == "auto", "default XLog mode");

    const auto configured = parse({
        "hydrox_sitl",
        "--ue5-port", "14605",
        "--ros-domain-id", "17",
        "--vehicle", "auv5",
        "--vehicle-type", "LAUV",
        "--rate", "200",
        "--time-mode", "wall",
        "--mavlink-signing-key-file", "D:/secure/hil.key",
        "--mavlink-signing-link-id", "42",
        "--publish-truth-state", "true",
        "--allow-truth-heading-aid", "1",
    });
    failures += expect(configured.ue5_port == 14605, "configured UE5 port");
    failures += expect(configured.ros_domain_id == 17, "configured DDS domain");
    failures += expect(configured.vehicle == "auv5", "configured vehicle name");
    failures += expect(configured.vehicle_type == "LAUV", "configured vehicle type");
    failures += expect(configured.rate_hz == 200, "configured control rate");
    failures += expect(configured.time_mode == "wall", "configured wall time mode");
    failures += expect(configured.mavlink_signing_key_file == "D:/secure/hil.key",
                       "configured MAVLink signing key file");
    failures += expect(configured.mavlink_signing_link_id == 42,
                       "configured MAVLink signing link id");
    failures += expect(configured.publish_truth_state, "truth publishing flag");
    failures += expect(configured.allow_truth_heading_aid, "truth heading aid flag");

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
    failures += expect(parse_fails({"hydrox_sitl", "--time-mode", "auto"}),
                       "unknown time mode is rejected");
    failures += expect(parse_fails({"hydrox_sitl", "--mavlink-signing-link-id", "256"}),
                       "out-of-range MAVLink signing link id is rejected");

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

    if (failures == 0)
        std::printf("test_sitl_config: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
