#include "dds_setpoint_codec.h"
#include "frame_contract.h"
#include "hydrox/runtime/hil_contract.h"

#include <ucdr/microcdr.h>

#include <utility>

namespace hydrox::dds_cdr
{
    namespace
    {
        bool reject(std::string *error, const char *message)
        {
            if (error)
                *error = message;
            return false;
        }

        bool has_room(const ucdrBuffer &reader, std::size_t value_size)
        {
            const std::size_t remaining = ucdr_buffer_remaining(&reader);
            const std::size_t padding = ucdr_buffer_alignment(&reader, value_size);
            return padding <= remaining && value_size <= remaining - padding;
        }

        bool read_int32(ucdrBuffer &reader, int32_t &value)
        {
            value = 0;
            return has_room(reader, sizeof(value)) &&
                   ucdr_deserialize_int32_t(&reader, &value);
        }

        bool read_uint32(ucdrBuffer &reader, uint32_t &value)
        {
            value = 0;
            return has_room(reader, sizeof(value)) &&
                   ucdr_deserialize_uint32_t(&reader, &value);
        }

        bool read_double(ucdrBuffer &reader, double &value)
        {
            value = 0.0;
            return has_room(reader, sizeof(value)) &&
                   ucdr_deserialize_double(&reader, &value);
        }

        bool read_char(ucdrBuffer &reader, char &value)
        {
            value = '\0';
            return has_room(reader, sizeof(value)) &&
                   ucdr_deserialize_char(&reader, &value);
        }

        bool read_bool(ucdrBuffer &reader, bool &value)
        {
            uint8_t encoded = 0;
            if (!has_room(reader, sizeof(encoded)) ||
                !ucdr_deserialize_uint8_t(&reader, &encoded) ||
                encoded > 1)
            {
                value = false;
                return false;
            }

            value = encoded != 0;
            return true;
        }

        bool read_bounded_string(ucdrBuffer &reader,
                                 std::size_t max_chars,
                                 std::string &value)
        {
            uint32_t wire_size = 0;
            if (!read_uint32(reader, wire_size) ||
                wire_size == 0 ||
                wire_size > max_chars + 1)
            {
                return false;
            }

            std::string decoded;
            decoded.reserve(wire_size - 1);
            for (uint32_t i = 0; i < wire_size; ++i)
            {
                char c = '\0';
                if (!read_char(reader, c))
                    return false;

                const bool is_terminator = i + 1 == wire_size;
                if (is_terminator)
                {
                    if (c != '\0')
                        return false;
                }
                else
                {
                    if (c == '\0')
                        return false;
                    decoded.push_back(c);
                }
            }

            value = std::move(decoded);
            return true;
        }

        bool parse_mode(const std::string &mode, uint8_t &encoded)
        {
            if (mode == "DISABLED")
                encoded = 0;
            else if (mode == "DEPTH_HOLD")
                encoded = 1;
            else if (mode == "WAYPOINT_3D")
                encoded = 2;
            else if (mode == "DP")
                encoded = 3;
            else if (mode == "SURFACE")
                encoded = 4;
            else
                return false;
            return true;
        }
    } // namespace

    bool decode_gnc_setpoint(const uint8_t *data,
                             std::size_t len,
                             GNCSetpointDds &out,
                             std::string *error)
    {
        if (!data || len == 0)
            return reject(error, "empty CDR payload");

        // ROS 2 serialized messages carry the four-byte DDS CDR
        // encapsulation header. Micro XRCE-DDS can also provide a bare CDR
        // payload, so keep that representation compatible. Alignment starts
        // after the encapsulation header, not at the beginning of the sample.
        constexpr std::size_t encapsulation_size = 4;
        const bool has_cdr_encapsulation =
            len >= encapsulation_size && data[0] == 0 &&
            (data[1] == 0 || data[1] == 1) &&
            data[2] == 0 && data[3] == 0;

        ucdrBuffer reader;
        if (has_cdr_encapsulation)
        {
            const ucdrEndianness endianness = data[1] == 1
                                                  ? UCDR_LITTLE_ENDIANNESS
                                                  : UCDR_BIG_ENDIANNESS;
            ucdr_init_buffer_origin_offset_endian(
                &reader,
                const_cast<uint8_t *>(data + encapsulation_size),
                len - encapsulation_size,
                0,
                0,
                endianness);
        }
        else
        {
            ucdr_init_buffer(&reader, const_cast<uint8_t *>(data), len);
        }

        int32_t sec = 0;
        uint32_t nanosec = 0;
        std::string frame_id;
        std::string mode;
        GNCSetpointDds decoded;

        if (!read_int32(reader, sec) ||
            !read_uint32(reader, nanosec) ||
            !read_bounded_string(reader, MAX_SETPOINT_FRAME_ID_CHARS, frame_id))
        {
            return reject(error, "invalid or truncated header");
        }
        (void)sec;
        if (!frame_contract::is_map_ned(frame_id))
            return reject(error, "GNCSetpoint header.frame_id must be map_ned");

        if (nanosec >= 1'000'000'000U)
            return reject(error, "header nanosec is out of range");

        if (!read_bounded_string(reader, MAX_SETPOINT_MODE_CHARS, mode))
            return reject(error, "invalid, truncated, or oversized mode");

        if (!read_double(reader, decoded.depth_ref) ||
            !read_double(reader, decoded.heading_ref) ||
            !read_double(reader, decoded.surge_ref) ||
            !read_bool(reader, decoded.use_yaw_rate_ref) ||
            !read_double(reader, decoded.yaw_rate_ref) ||
            !read_double(reader, decoded.wp_n) ||
            !read_double(reader, decoded.wp_e) ||
            !read_double(reader, decoded.wp_d) ||
            !read_bool(reader, decoded.use_path_segment) ||
            !read_double(reader, decoded.path_start_n) ||
            !read_double(reader, decoded.path_start_e) ||
            !read_double(reader, decoded.lookahead_m) ||
            !read_bool(reader, decoded.hold_heading) ||
            !read_double(reader, decoded.arrival_radius_m))
        {
            return reject(error, "invalid or truncated setpoint fields");
        }

        if (!parse_mode(mode, decoded.mode))
            return reject(error, "unknown setpoint mode");

        GNCSetpoint runtime_setpoint;
        runtime_setpoint.depth_ref = decoded.depth_ref;
        runtime_setpoint.heading_ref = decoded.heading_ref;
        runtime_setpoint.surge_ref = decoded.surge_ref;
        runtime_setpoint.use_yaw_rate_ref = decoded.use_yaw_rate_ref;
        runtime_setpoint.yaw_rate_ref = decoded.yaw_rate_ref;
        runtime_setpoint.wp_n = decoded.wp_n;
        runtime_setpoint.wp_e = decoded.wp_e;
        runtime_setpoint.wp_d = decoded.wp_d;
        runtime_setpoint.use_path_segment = decoded.use_path_segment;
        runtime_setpoint.path_start_n = decoded.path_start_n;
        runtime_setpoint.path_start_e = decoded.path_start_e;
        runtime_setpoint.lookahead_m = decoded.lookahead_m;
        runtime_setpoint.arrival_radius_m = decoded.arrival_radius_m;
        runtime_setpoint.hold_heading = decoded.hold_heading;
        if (!runtime::valid_gnc_setpoint(
                runtime_setpoint,
                static_cast<GNCMode>(decoded.mode)))
            return reject(error, "setpoint is outside the shared HIL contract");

        if (ucdr_buffer_has_error(&reader))
            return reject(error, "CDR payload has an invalid trailing section");

        // XRCE DATA submessages are aligned to a four-byte boundary. When
        // the final CDR field is narrower than that boundary, the callback
        // length includes up to three zero fill bytes after the sample.
        // Accept exactly that transport padding while rejecting arbitrary
        // trailing data and non-zero bytes.
        const std::size_t remaining = ucdr_buffer_remaining(&reader);
        bool valid_xrce_padding = remaining == 0;
        if (remaining != 0 && remaining <= 3 && (len % 4) == 0)
        {
            valid_xrce_padding = true;
            for (std::size_t index = 0; index < remaining; ++index)
                valid_xrce_padding = valid_xrce_padding && reader.iterator[index] == 0;
        }
        if (!valid_xrce_padding)
            return reject(error, "CDR payload has an invalid trailing section");

        decoded.valid = true;
        out = decoded;
        if (error)
            error->clear();
        return true;
    }
} // namespace hydrox::dds_cdr
