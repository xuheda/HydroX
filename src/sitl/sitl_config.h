#pragma once

#include "hydrox/runtime/hil_contract.h"
#include "hydrox/runtime/control_feedback.h"
#include "control_parameters.h"
#include "sensor_adapter.h"
#include "types.h"

#include <cstdint>
#include <array>
#include <string>

namespace hydrox::sitl
{
    enum class ResidualPolicyMode : uint8_t
    {
        Disabled = 0,
        Shadow,
        Active,
    };

    struct Config
    {
        std::string ue5_host = "127.0.0.1";
        uint16_t ue5_port = 14600;
        std::string qgc_host = "255.255.255.255";
        uint16_t qgc_port = 14550;
        uint8_t mavlink_system_id = 1;
        std::string dds_host = "127.0.0.1";
        uint16_t dds_port = 8888;
        uint16_t ros_domain_id = 0;
        std::string vehicle = "vehicle0";
        std::string vehicle_type = "EcaA9";
        std::string vehicle_bundle;
        std::string ekf_accel = "auto";
        std::string xlog = "auto";
        std::string log_directory;
        std::string run_id;
        /** Path to a 64-hex-character MAVLink 2 signing key. Empty keeps local HIL unsigned. */
        std::string mavlink_signing_key_file;
        uint8_t mavlink_signing_link_id = 0;
        bool publish_truth_state = false;
        bool allow_truth_heading_aid = false;
        runtime::ControlFeedbackSource control_feedback_source =
            runtime::ControlFeedbackSource::EstimatedState;
        uint64_t parent_pid = 0;
        int rate_hz = 100;

        // Local-only learned residual sidecar. Disabled is the unconditional
        // default; Shadow evaluates and records candidates without authority.
        ResidualPolicyMode residual_policy_mode = ResidualPolicyMode::Disabled;
        std::string residual_policy_host = "127.0.0.1";
        uint16_t residual_policy_port = 0;
        uint16_t residual_policy_local_port = 0;
        uint64_t residual_policy_nonce = 0;
        int residual_policy_hz = 10;
        // Wide enough for the measured Windows loopback Python sidecar tail
        // (about 4 ms after warm-up), while remaining far below the 100 ms
        // decision period. Any miss still fails closed for that decision.
        double residual_policy_timeout_ms = 15.0;
        double residual_blend = 0.35;
        double residual_min_confidence = 0.75;
        std::array<double, 3> residual_max_delta = {180.0, 0.0, 90.0};
        std::array<double, 3> residual_max_rate = {600.0, 0.0, 300.0};

        std::string init_mode = "DISABLED";
        double init_depth = 5.0;
        double init_heading = 0.0;
        double init_surge = 0.0;
        double init_n = 0.0;
        double init_e = 0.0;
        double mission_radius = 3.0;
        double mission_timeout_s =
            static_cast<double>(runtime::kDefaultHilSetpointTimeoutUs) * 1e-6;
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
        runtime::ControlFeedbackSource &out_source);
    const char *control_feedback_source_name(runtime::ControlFeedbackSource source);
    bool try_parse_residual_policy_mode(
        const std::string &value,
        ResidualPolicyMode &out_mode);
    const char *residual_policy_mode_name(ResidualPolicyMode mode);
    uint8_t mav_type_for_vehicle_class(VehicleClass vehicle_class);
    const char *vehicle_class_name(VehicleClass vehicle_class);
    const char *vehicle_archetype_name(VehicleArchetype archetype);

} // namespace hydrox::sitl
