#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace hydrox
{
    struct GNCSetpointDds
    {
        double depth_ref = 5.0;
        double heading_ref = 0.0;
        double surge_ref = 1.0;
        bool use_yaw_rate_ref = false;
        double yaw_rate_ref = 0.0;
        double wp_n = 0.0;
        double wp_e = 0.0;
        double wp_d = 0.0;
        bool use_path_segment = false;
        double path_start_n = 0.0;
        double path_start_e = 0.0;
        double lookahead_m = 0.0;
        double arrival_radius_m = 0.0;
        bool hold_heading = false;
        uint8_t mode = 1;
        bool valid = false;
    };

    namespace dds_cdr
    {
        constexpr std::size_t MAX_SETPOINT_FRAME_ID_CHARS = 128;
        constexpr std::size_t MAX_SETPOINT_MODE_CHARS = 31;

        // Decode one complete oceanx_interfaces/msg/GNCSetpoint CDR sample.
        // Both a ROS 2 DDS-encapsulated sample and a bare Micro XRCE-DDS CDR
        // payload are accepted. The output is assigned only after every field
        // and the complete input length have been validated.
        bool decode_gnc_setpoint(const uint8_t *data,
                                 std::size_t len,
                                 GNCSetpointDds &out,
                                 std::string *error = nullptr);
    } // namespace dds_cdr
} // namespace hydrox
