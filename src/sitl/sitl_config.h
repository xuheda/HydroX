#pragma once

#include "sitl/control_feedback.h"
#include "fossen_vehicle_params.h"
#include "sensor_adapter.h"
#include "types.h"

#include <cstdint>
#include <string>

namespace hydrox::sitl
{
    struct Config
    {
        std::string ue5_host = "127.0.0.1";
        uint16_t ue5_port = 14600;
        std::string qgc_host = "255.255.255.255";
        uint16_t qgc_port = 14550;
        std::string dds_host = "127.0.0.1";
        uint16_t dds_port = 8888;
        uint16_t ros_domain_id = 0;
        std::string vehicle = "vehicle0";
        std::string vehicle_type = "EcaA9";
        std::string vehicle_bundle;
        std::string vehicle_params;
        std::string vehicle_params_dir;
        std::string ekf_accel = "auto";
        std::string xlog = "auto";
        /** Path to a 64-hex-character MAVLink 2 signing key. Empty keeps local HIL unsigned. */
        std::string mavlink_signing_key_file;
        uint8_t mavlink_signing_link_id = 0;
        bool publish_truth_state = false;
        bool allow_truth_heading_aid = false;
        ControlFeedbackSource control_feedback_source =
            ControlFeedbackSource::EstimatedState;
        uint64_t parent_pid = 0;
        int rate_hz = 100;

        std::string init_mode = "DISABLED";
        double init_depth = 5.0;
        double init_heading = 0.0;
        double init_surge = 0.0;
        double init_n = 0.0;
        double init_e = 0.0;
        double mission_radius = 3.0;
        double mission_timeout_s = 2.0;
        std::string gps_projection = "wgs84-aeqd-v1";
        double gps_origin_lat_deg = 0.0;
        double gps_origin_lon_deg = 0.0;
        double gps_origin_altitude_msl_m = 0.0;
        double gps_max_radius_m = 10000.0;
    };

    Config parse_config(int argc, char *argv[]);

    GNCMode gnc_mode_from_string(const std::string &value);
    const char *gnc_mode_name(GNCMode mode);
    AccelMode accel_mode_from_string(const std::string &value);
    const char *accel_mode_name(AccelMode mode);
    bool try_parse_control_feedback_source(
        const std::string &value,
        ControlFeedbackSource &out_source);
    const char *control_feedback_source_name(ControlFeedbackSource source);
    uint8_t mav_type_for_vehicle_class(VehicleClass vehicle_class);
    const char *vehicle_class_name(VehicleClass vehicle_class);
    const char *vehicle_archetype_name(VehicleArchetype archetype);

} // namespace hydrox::sitl
