#include "sitl_config.h"
#include "geodesy.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace hydrox::sitl
{
namespace
{
    bool parse_bool(const std::string &value)
    {
        return value == "1" || value == "true" || value == "TRUE" ||
               value == "on" || value == "ON" ||
               value == "yes" || value == "YES";
    }

    uint16_t parse_port(const std::string &option, const std::string &value)
    {
        const int port = std::stoi(value);
        if (port < 1 || port > 65535)
            throw std::out_of_range(option + " must be in [1, 65535]");
        return static_cast<uint16_t>(port);
    }
}

Config parse_config(int argc, char *argv[])
{
    if ((argc - 1) % 2 != 0)
        throw std::invalid_argument("every SITL option requires a value");

    Config config;
    for (int i = 1; i < argc; i += 2)
    {
        const std::string key = argv[i];
        const std::string value = argv[i + 1];

        if (key == "--ue5-host")
            config.ue5_host = value;
        else if (key == "--ue5-port")
            config.ue5_port = parse_port(key, value);
        else if (key == "--qgc-host")
            config.qgc_host = value;
        else if (key == "--qgc-port")
            config.qgc_port = parse_port(key, value);
        else if (key == "--dds-host")
            config.dds_host = value;
        else if (key == "--dds-port")
            config.dds_port = parse_port(key, value);
        else if (key == "--ros-domain-id")
        {
            const int domain_id = std::stoi(value);
            if (domain_id < 0 || domain_id > 232)
                throw std::out_of_range("--ros-domain-id must be in [0, 232]");
            config.ros_domain_id = static_cast<uint16_t>(domain_id);
        }
        else if (key == "--vehicle")
            config.vehicle = value;
        else if (key == "--vehicle-type")
            config.vehicle_type = value;
        else if (key == "--vehicle-bundle")
            config.vehicle_bundle = value;
        else if (key == "--vehicle-params")
            config.vehicle_params = value;
        else if (key == "--vehicle-params-dir")
            config.vehicle_params_dir = value;
        else if (key == "--ekf-accel")
            config.ekf_accel = value;
        else if (key == "--xlog")
            config.xlog = value;
        else if (key == "--mavlink-signing-key-file")
            config.mavlink_signing_key_file = value;
        else if (key == "--mavlink-signing-link-id")
        {
            const int link_id = std::stoi(value);
            if (link_id < 0 || link_id > 255)
                throw std::out_of_range("--mavlink-signing-link-id must be in [0, 255]");
            config.mavlink_signing_link_id = static_cast<uint8_t>(link_id);
        }
        else if (key == "--parent-pid")
            config.parent_pid = static_cast<uint64_t>(std::stoull(value));
        else if (key == "--publish-truth-state")
            config.publish_truth_state = parse_bool(value);
        else if (key == "--allow-truth-heading-aid")
            config.allow_truth_heading_aid = parse_bool(value);
        else if (key == "--control-feedback-source")
        {
            if (!try_parse_control_feedback_source(
                    value,
                    config.control_feedback_source))
            {
                throw std::invalid_argument(
                    "--control-feedback-source must be 'estimated_state' or 'truth_debug'");
            }
        }
        else if (key == "--rate")
            config.rate_hz = std::stoi(value);
        else if (key == "--mode")
            config.init_mode = value;
        else if (key == "--depth")
            config.init_depth = std::stod(value);
        else if (key == "--heading")
            config.init_heading = std::stod(value);
        else if (key == "--surge")
            config.init_surge = std::stod(value);
        else if (key == "--init-n")
            config.init_n = std::stod(value);
        else if (key == "--init-e")
            config.init_e = std::stod(value);
        else if (key == "--mission-radius")
            config.mission_radius = std::stod(value);
        else if (key == "--mission-timeout")
            config.mission_timeout_s = std::stod(value);
        else if (key == "--gps-projection")
            config.gps_projection = value;
        else if (key == "--gps-origin-lat-deg")
            config.gps_origin_lat_deg = std::stod(value);
        else if (key == "--gps-origin-lon-deg")
            config.gps_origin_lon_deg = std::stod(value);
        else if (key == "--gps-origin-alt-msl-m")
            config.gps_origin_altitude_msl_m = std::stod(value);
        else if (key == "--gps-max-radius-m")
            config.gps_max_radius_m = std::stod(value);
        else
            throw std::invalid_argument("unknown SITL option: " + key);
    }

    if (config.rate_hz <= 0)
        throw std::out_of_range("--rate must be greater than zero");
    if (config.gps_projection != geodesy::kProjectionId)
        throw std::invalid_argument(
            "--gps-projection must be wgs84-aeqd-v1");
    const geodesy::Wgs84AeqdFrame gps_frame{
        config.gps_origin_lat_deg,
        config.gps_origin_lon_deg,
        config.gps_origin_altitude_msl_m,
        config.gps_max_radius_m};
    if (!geodesy::is_valid(gps_frame))
        throw std::out_of_range(
            "GPS origin and --gps-max-radius-m must define a valid WGS-84 AEQD frame");
    return config;
}

GNCMode gnc_mode_from_string(const std::string &value)
{
    if (value == "DEPTH_HOLD")
        return GNCMode::DEPTH_HOLD;
    if (value == "WAYPOINT_3D")
        return GNCMode::WAYPOINT_3D;
    if (value == "DP")
        return GNCMode::DP;
    if (value == "SURFACE")
        return GNCMode::SURFACE;
    return GNCMode::DISABLED;
}

const char *gnc_mode_name(GNCMode mode)
{
    switch (mode)
    {
    case GNCMode::DEPTH_HOLD:
        return "DEPTH_HOLD";
    case GNCMode::WAYPOINT_3D:
        return "WAYPOINT_3D";
    case GNCMode::DP:
        return "DP";
    case GNCMode::SURFACE:
        return "SURFACE";
    case GNCMode::DISABLED:
    default:
        return "DISABLED";
    }
}

AccelMode accel_mode_from_string(const std::string &value)
{
    if (value == "off" || value == "OFF" || value == "0")
        return AccelMode::Off;
    if (value == "on" || value == "ON" || value == "1")
        return AccelMode::On;
    return AccelMode::Auto;
}

const char *accel_mode_name(AccelMode mode)
{
    switch (mode)
    {
    case AccelMode::Off:
        return "off";
    case AccelMode::On:
        return "on";
    case AccelMode::Auto:
    default:
        return "auto";
    }
}

bool try_parse_control_feedback_source(
    const std::string &value,
    ControlFeedbackSource &out_source)
{
    std::string normalized = value;
    std::transform(
        normalized.begin(),
        normalized.end(),
        normalized.begin(),
        [](unsigned char ch)
        {
            return static_cast<char>(std::tolower(ch));
        });
    std::replace(normalized.begin(), normalized.end(), '-', '_');

    if (normalized == "estimated_state" || normalized == "ekf")
    {
        out_source = ControlFeedbackSource::EstimatedState;
        return true;
    }
    if (normalized == "truth_debug")
    {
        out_source = ControlFeedbackSource::TruthDebug;
        return true;
    }
    return false;
}

const char *control_feedback_source_name(ControlFeedbackSource source)
{
    switch (source)
    {
    case ControlFeedbackSource::TruthDebug:
        return "truth_debug";
    case ControlFeedbackSource::EstimatedState:
    default:
        return "estimated_state";
    }
}

uint8_t mav_type_for_vehicle_class(VehicleClass vehicle_class)
{
    switch (vehicle_class)
    {
    case VehicleClass::USV:
        return 11; // MAV_TYPE_SURFACE_BOAT
    case VehicleClass::UAV_MULTIROTOR:
        return 2; // MAV_TYPE_QUADROTOR
    case VehicleClass::UAV_FIXED_WING:
        return 1; // MAV_TYPE_FIXED_WING
    case VehicleClass::UAV_VTOL:
        return 19; // MAV_TYPE_VTOL_DUOROTOR (generic VTOL family identifier)
    case VehicleClass::UUV:
    default:
        return 12; // MAV_TYPE_SUBMARINE
    }
}

const char *vehicle_class_name(VehicleClass vehicle_class)
{
    switch (vehicle_class)
    {
    case VehicleClass::USV:
        return "USV";
    case VehicleClass::UAV_MULTIROTOR:
        return "UAV_MULTIROTOR";
    case VehicleClass::UAV_FIXED_WING:
        return "UAV_FIXED_WING";
    case VehicleClass::UAV_VTOL:
        return "UAV_VTOL";
    case VehicleClass::UUV:
    default:
        return "UUV";
    }
}

const char *vehicle_archetype_name(VehicleArchetype archetype)
{
    switch (archetype)
    {
    case VehicleArchetype::Thruster:
        return "Thruster";
    case VehicleArchetype::Surface:
        return "Surface";
    case VehicleArchetype::Multirotor:
        return "Multirotor";
    case VehicleArchetype::FixedWing:
        return "FixedWing";
    case VehicleArchetype::VTOL:
        return "VTOL";
    case VehicleArchetype::SlenderBodyFin:
    default:
        return "SlenderBodyFin";
    }
}

} // namespace hydrox::sitl
