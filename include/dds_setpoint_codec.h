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
        uint8_t mode = 1;
        bool valid = false;
    };

    namespace dds_cdr
    {
        constexpr std::size_t MAX_SETPOINT_FRAME_ID_CHARS = 128;
        constexpr std::size_t MAX_SETPOINT_MODE_CHARS = 31;

        // Decode one complete oceanx_interfaces/msg/GNCSetpoint CDR sample.
        // The output is assigned only after every field and the complete input
        // length have been validated.
        bool decode_gnc_setpoint(const uint8_t *data,
                                 std::size_t len,
                                 GNCSetpointDds &out,
                                 std::string *error = nullptr);
    } // namespace dds_cdr
} // namespace hydrox
