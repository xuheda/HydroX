/**
 * mavlink_hil.cpp — MAVLink HIL message encoding and decoding implementation
 */
#include "mavlink_hil.h"
#include <cinttypes>
#include <cstring>
#include <algorithm>
#include <cstdio>
#include <limits>

namespace hydrox
{

    // CRC_EXTRA table (consistent with PX4 / QGC)
    static uint8_t crc_extra_table(uint32_t msg_id)
    {
        switch (msg_id)
        {
        case MSGID_HEARTBEAT:
            return 50;
        case MSGID_SYS_STATUS:
            return 124;
        case MSGID_ATTITUDE:
            return 39;
        case MSGID_LOCAL_POSITION_NED:
            return 185;
        case MSGID_GLOBAL_POSITION_INT:
            return 104;
        case MSGID_VFR_HUD:
            return 20;
        case MSGID_STATUSTEXT:
            return 83;
        case MSGID_COMMAND_LONG:
            return 152;
        case MSGID_HIL_SENSOR:
            return 108;
        case MSGID_HIL_GPS:
            return 124;
        case MSGID_HIL_STATE_QUAT:
            return 4;
        case MSGID_HIL_ACTUATOR_CONTROLS:
            return 47;
        case MSGID_HIL_DVL:
            return 82; // Tracking-aware 26-byte schema, agreed by both sides
        case MSGID_HIL_TRUTH_STATE:
            return 78; // Custom debug truth, agreed by both sides
        case MSGID_HIL_PASSIVE_SONAR:
            return 79; // Custom OceanX sensor, agreed by both sides
        case MSGID_HIL_ACOUSTIC_NEIGHBORS:
            return 80; // Custom OceanX sensor, agreed by both sides
        case MSGID_HIL_RANGEFINDER_SCAN:
            return 81; // Custom OceanX sensor, agreed by both sides
        case MSGID_HIL_WHEEL_ODOMETRY:
            return 83; // 33-byte differential-drive odometry schema
        case MSGID_HIL_PROFILE:
            return 84; // 62-byte profile selection/readiness handshake
        case MSGID_HIL_SESSION_CONFIG:
            return 85; // 120-byte versioned per-run configuration
        case MSGID_HIL_SESSION_STATUS:
            return 86; // 49-byte configuration acknowledgement
        case MSGID_HIL_FC_STATE:
            return 87; // 255-byte flight-controller estimate telemetry
        case MSGID_HIL_CONTROL_TRACE:
            return 88; // 248-byte per-control-tick replay trace
        default:
            return 0;
        }
    }

    // CRC-16/MCRF4XX
    uint16_t MavlinkHIL::_crc16(const uint8_t *data, size_t len, uint16_t crc)
    {
        for (size_t i = 0; i < len; ++i)
        {
            uint8_t tmp = data[i] ^ (crc & 0xFF);
            tmp ^= (tmp << 4) & 0xFF;
            crc = ((crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8) ^ (static_cast<uint16_t>(tmp) << 3) ^ (static_cast<uint16_t>(tmp) >> 4)) & 0xFFFF;
        }
        return crc;
    }

    uint8_t MavlinkHIL::_crc_extra(uint32_t msg_id)
    {
        return crc_extra_table(msg_id);
    }

    // Constructor
    MavlinkHIL::MavlinkHIL(
        uint8_t sysid,
        uint8_t compid,
        const MavlinkSigningConfig &signing)
        : _sysid(sysid), _compid(compid), _signing(signing) {}

    // Receive: byte stream -> frame
    std::vector<MavFrame> MavlinkHIL::feed(const uint8_t *data, size_t len)
    {
        std::vector<MavFrame> result;
        feed_each(
            data,
            len,
            &result,
            [](void *context, const MavFrame &frame)
            {
                static_cast<std::vector<MavFrame> *>(context)->push_back(frame);
            });
        return result;
    }

    void MavlinkHIL::feed_each(const uint8_t *data, size_t len,
                               void *context, FrameVisitor visitor)
    {
        if (data == nullptr || visitor == nullptr)
            return;

        for (size_t i = 0; i < len; ++i)
        {
            if (!_buf.push_back(data[i]))
            {
                // A valid MAVLink packet fits in the fixed buffer. Overflow
                // therefore means corrupt input; resynchronise at this byte.
                _buf.clear();
                if (data[i] == 0xFD || data[i] == 0xFE)
                    (void)_buf.push_back(data[i]);
            }

            while (true)
            {
                auto frame = _parse_one();
                if (!frame)
                    break;
                visitor(context, *frame);
            }
        }
    }

    std::optional<MavFrame> MavlinkHIL::_parse_one()
    {
        // HydroX runtime traffic is MAVLink 2. PX4's official uploader uses
        // MAVLink 1 COMMAND_LONG frames to request a bootloader reboot, so
        // accept both wire headers and keep the same CRC-extra validation.
        while (!_buf.empty() && _buf[0] != 0xFD && _buf[0] != 0xFE)
            _buf.erase(_buf.begin());

        if (_buf.empty())
            return std::nullopt;
        const bool mavlink2 = _buf[0] == 0xFD;
        const size_t header_len = mavlink2 ? 10u : 6u;
        if (_buf.size() < header_len + 2u)
            return std::nullopt;

        const uint8_t payload_len = _buf[1];
        uint8_t incompat_flags = 0;
        if (mavlink2)
        {
            incompat_flags = _buf[2];
            if ((incompat_flags & ~MAVLINK_IFLAG_SIGNED) != 0)
            {
                // Unknown incompatibility flags must be discarded by
                // MAVLink 2 receivers.
                _buf.erase(_buf.begin());
                return _parse_one();
            }
        }
        const bool signed_frame =
            mavlink2 && (incompat_flags & MAVLINK_IFLAG_SIGNED) != 0;
        const size_t base_frame_len = header_len + payload_len + 2u;
        const size_t frame_len = base_frame_len +
            (signed_frame ? MAVLINK_SIGNATURE_BLOCK_LEN : 0u);

        if (_buf.size() < frame_len)
            return std::nullopt;

        const uint32_t msg_id = mavlink2
            ? static_cast<uint32_t>(_buf[7]) |
                  (static_cast<uint32_t>(_buf[8]) << 8) |
                  (static_cast<uint32_t>(_buf[9]) << 16)
            : static_cast<uint32_t>(_buf[5]);

        // CRC verification (starting from payload_len byte, to the end of payload)
        const size_t crc_offset = header_len + payload_len;
        uint16_t crc_rcv = static_cast<uint16_t>(_buf[crc_offset]) |
                           (static_cast<uint16_t>(_buf[crc_offset + 1]) << 8);

        // CRC covers the header after magic plus the complete payload.
        uint16_t crc_calc = _crc16(
            _buf.data() + 1, (header_len - 1u) + payload_len);
        uint8_t extra = crc_extra_table(msg_id);
        crc_calc = _crc16(&extra, 1, crc_calc);

        if (crc_rcv != crc_calc)
        {
            static int bad_crc_count = 0;
            if (++bad_crc_count <= 10)
                std::fprintf(stderr,
                             "[MavlinkHIL] bad CRC #%d msg=%" PRIu32 " len=%u rcv=0x%04x calc=0x%04x\n",
                             bad_crc_count, msg_id, payload_len, crc_rcv, crc_calc);
            _buf.erase(_buf.begin(), _buf.begin() + static_cast<int>(frame_len));
            return _parse_one(); // Discard bad frame, continue parsing subsequent frames in the same buffer
        }

        const uint8_t sysid = _buf[mavlink2 ? 5 : 3];
        const uint8_t compid = _buf[mavlink2 ? 6 : 4];
        bool signing_valid = true;
        if (signed_frame)
        {
            const uint8_t *signature = _buf.data() + base_frame_len;
            signing_valid = _validate_signed_frame(
                _buf.data(), base_frame_len, sysid, compid, signature);
        }
        else if (_signing.require_incoming)
        {
            signing_valid = false;
        }
        if (!signing_valid)
        {
            _buf.erase(_buf.begin(), _buf.begin() + static_cast<int>(frame_len));
            return _parse_one();
        }

        MavFrame frame;
        frame.msg_id = msg_id;
        frame.sysid = sysid;
        frame.compid = compid;
        frame.mavlink_version = mavlink2 ? 2 : 1;
        frame.signed_frame = signed_frame;
        (void)frame.payload.assign(_buf.begin() + header_len,
                                   _buf.begin() + header_len + payload_len);
        _buf.erase(_buf.begin(), _buf.begin() + static_cast<int>(frame_len));
        return frame;
    }

    bool MavlinkHIL::_validate_signed_frame(
        const uint8_t *packet,
        size_t packet_len_without_signature,
        uint8_t sysid,
        uint8_t compid,
        const uint8_t *signature)
    {
        if (!_signing.valid() || packet == nullptr ||
            packet_len_without_signature < 12 || signature == nullptr)
        {
            return false;
        }

        const uint8_t link_id = signature[0];
        if (link_id != _signing.link_id)
            return false;

        uint64_t timestamp = 0;
        for (int i = 0; i < 6; ++i)
            timestamp |= static_cast<uint64_t>(signature[1 + i]) << (8 * i);

        std::array<uint8_t, 32> digest{};
        // MAVLink signs header+payload+CRC, excluding the 0xFD magic byte.
        if (!mavlink_signing_digest(
                _signing.secret_key,
                packet + 1,
                packet_len_without_signature - 1,
                link_id,
                timestamp,
                digest) ||
            !mavlink_signature_equal_48(digest.data(), signature + 7))
        {
            return false;
        }

        const uint32_t stream_id =
            (static_cast<uint32_t>(sysid) << 16) |
            (static_cast<uint32_t>(compid) << 8) |
            static_cast<uint32_t>(link_id);
        ReplayStream *previous = nullptr;
        ReplayStream *unused = nullptr;
        for (ReplayStream &stream : _last_rx_timestamps)
        {
            if (stream.used && stream.id == stream_id)
                previous = &stream;
            else if (!stream.used && unused == nullptr)
                unused = &stream;
        }
        if (previous != nullptr && timestamp <= previous->timestamp)
            return false;

        constexpr uint64_t kReplayStartupWindowTicks = 6000000ULL; // 60 seconds
        const uint64_t local_timestamp = std::max(
            mavlink_signing_timestamp_10us(), _tx_signing_timestamp);
        if (previous == nullptr &&
            timestamp + kReplayStartupWindowTicks < local_timestamp)
        {
            return false;
        }

        ReplayStream *target = previous != nullptr ? previous : unused;
        if (target == nullptr)
            return false;
        target->id = stream_id;
        target->timestamp = timestamp;
        target->used = true;
        _tx_signing_timestamp = std::max(_tx_signing_timestamp, timestamp);
        return true;
    }

    // Parse various messages

    // Little-endian read helpers
    static inline void le_read(const uint8_t *p, uint64_t &v) { memcpy(&v, p, 8); }
    static inline void le_read(const uint8_t *p, int32_t &v) { memcpy(&v, p, 4); }
    static inline void le_read(const uint8_t *p, uint32_t &v) { memcpy(&v, p, 4); }
    static inline void le_read(const uint8_t *p, uint16_t &v) { memcpy(&v, p, 2); }
    static inline void le_read(const uint8_t *p, float &v) { memcpy(&v, p, 4); }
    static inline void le_read(const uint8_t *p, double &v) { memcpy(&v, p, 8); }
    static inline void le_read(const uint8_t *p, int16_t &v) { memcpy(&v, p, 2); }
    static inline void le_read(const uint8_t *p, uint8_t &v) { v = *p; }

    HeartbeatMsg MavlinkHIL::parse_heartbeat(const MavFrame &f) const
    {
        // HEARTBEAT: custom_mode(4), type, autopilot, base_mode,
        // system_status, mavlink_version.
        HeartbeatMsg m;
        if (f.payload.size() < 9)
            return m;
        const uint8_t *ptr = f.payload.data();
        le_read(ptr + 4, m.type);
        le_read(ptr + 5, m.autopilot);
        le_read(ptr + 6, m.base_mode);
        le_read(ptr + 7, m.system_status);
        le_read(ptr + 8, m.mavlink_version);
        m.valid = true;
        return m;
    }

    CommandLongMsg MavlinkHIL::parse_command_long(const MavFrame &f) const
    {
        CommandLongMsg message;
        if (f.msg_id != MSGID_COMMAND_LONG || f.payload.size() != 33)
            return message;
        const uint8_t *payload = f.payload.data();
        for (std::size_t index = 0; index < message.params.size(); ++index)
        {
            le_read(payload + index * sizeof(float), message.params[index]);
            if (!std::isfinite(message.params[index]))
                return message;
        }
        le_read(payload + 28, message.command);
        le_read(payload + 30, message.target_system);
        le_read(payload + 31, message.target_component);
        le_read(payload + 32, message.confirmation);
        message.valid = true;
        return message;
    }

    HilSensorMsg MavlinkHIL::parse_hil_sensor(const MavFrame &f) const
    {
        // HIL_SENSOR: time_usec(8) + 13×float(52) + fields_updated(4) = 64 bytes
        const auto &p = f.payload;
        HilSensorMsg m;
        if (p.size() < 64)
            return m;
        const uint8_t *ptr = p.data();
        le_read(ptr, m.time_usec);
        le_read(ptr + 8, m.xacc);
        le_read(ptr + 12, m.yacc);
        le_read(ptr + 16, m.zacc);
        le_read(ptr + 20, m.xgyro);
        le_read(ptr + 24, m.ygyro);
        le_read(ptr + 28, m.zgyro);
        le_read(ptr + 32, m.xmag);
        le_read(ptr + 36, m.ymag);
        le_read(ptr + 40, m.zmag);
        le_read(ptr + 44, m.abs_pressure);
        le_read(ptr + 48, m.diff_pressure);
        le_read(ptr + 52, m.pressure_alt);
        // [56]: temperature (unused)
        le_read(ptr + 60, m.fields_updated);
        return m;
    }

    HilGpsMsg MavlinkHIL::parse_hil_gps(const MavFrame &f) const
    {
        // HIL_GPS: time_usec(8) fix(1) lat(4) lon(4) alt(4) eph(2) epv(2)
        //          vel(2) vn(2) ve(2) vd(2) cog(2) sats(1) = 36 bytes
        const auto &p = f.payload;
        HilGpsMsg m;
        if (p.size() < 36)
            return m;
        const uint8_t *ptr = p.data();
        le_read(ptr, m.time_usec);
        le_read(ptr + 8, m.fix_type);
        le_read(ptr + 9, m.lat);
        le_read(ptr + 13, m.lon);
        le_read(ptr + 17, m.alt);
        le_read(ptr + 21, m.eph);
        le_read(ptr + 23, m.epv);
        le_read(ptr + 25, m.vel);
        le_read(ptr + 27, m.vn);
        le_read(ptr + 29, m.ve);
        le_read(ptr + 31, m.vd);
        le_read(ptr + 33, m.cog);
        le_read(ptr + 35, m.satellites_visible);
        return m;
    }

    HilDvlMsg MavlinkHIL::parse_hil_dvl(const MavFrame &f) const
    {
        // Custom HIL_DVL: time_usec(8), vx/vy/vz(12), altitude_m(4),
        // tracking_mode(1), flags(1) = 26 bytes. This is intentionally an
        // exact-length contract: legacy 25-byte payloads are invalid.
        const auto &p = f.payload;
        HilDvlMsg m;
        if (p.size() != HIL_DVL_PAYLOAD_LEN)
            return m;
        const uint8_t *ptr = p.data();
        le_read(ptr, m.time_usec);
        le_read(ptr + 8, m.vx);
        le_read(ptr + 12, m.vy);
        le_read(ptr + 16, m.vz);
        le_read(ptr + 20, m.altitude_m);
        uint8_t TrackingMode = 0;
        le_read(ptr + 24, TrackingMode);
        m.tracking_mode = static_cast<DvlTrackingMode>(TrackingMode);
        le_read(ptr + 25, m.flags);

        if (!m.is_bottom_track() && !m.is_water_track())
        {
            m.tracking_mode = DvlTrackingMode::Unavailable;
            m.flags = 0;
            m.altitude_m = std::numeric_limits<float>::quiet_NaN();
        }
        else if (m.is_water_track())
        {
            // Water-track has no bottom range. Normalize malformed sender
            // values so downstream code can never observe a fake zero height.
            m.flags &= static_cast<uint8_t>(~HIL_DVL_FLAG_ALTITUDE_VALID);
            m.altitude_m = std::numeric_limits<float>::quiet_NaN();
        }
        else if (!std::isfinite(m.altitude_m))
        {
            m.flags &= static_cast<uint8_t>(~HIL_DVL_FLAG_ALTITUDE_VALID);
        }
        return m;
    }

    HilWheelOdometryMsg MavlinkHIL::parse_hil_wheel_odometry(const MavFrame &f) const
    {
        const auto &p = f.payload;
        HilWheelOdometryMsg m;
        if (p.size() != HIL_WHEEL_ODOMETRY_PAYLOAD_LEN)
            return m;
        const uint8_t *ptr = p.data();
        le_read(ptr, m.time_usec);
        le_read(ptr + 8, m.right_radps);
        le_read(ptr + 12, m.left_radps);
        le_read(ptr + 16, m.forward_mps);
        le_read(ptr + 20, m.yaw_rate_radps);
        le_read(ptr + 24, m.velocity_variance);
        le_read(ptr + 28, m.yaw_rate_variance);
        le_read(ptr + 32, m.flags);
        if (!m.velocity_valid())
            m.flags &= static_cast<uint8_t>(~HIL_WHEEL_ODOMETRY_FLAG_VELOCITY_VALID);
        if (!m.wheel_speeds_valid())
            m.flags &= static_cast<uint8_t>(~HIL_WHEEL_ODOMETRY_FLAG_WHEEL_SPEEDS_VALID);
        if (!std::isfinite(m.yaw_rate_radps) ||
            !std::isfinite(m.yaw_rate_variance) || m.yaw_rate_variance <= 0.0f)
            m.flags &= static_cast<uint8_t>(~HIL_WHEEL_ODOMETRY_FLAG_YAW_RATE_VALID);
        return m;
    }

    HilProfileMsg MavlinkHIL::parse_hil_profile(const MavFrame &f) const
    {
        HilProfileMsg message;
        if (f.payload.size() != HIL_PROFILE_PAYLOAD_LEN)
            return message;
        const uint8_t *payload = f.payload.data();
        le_read(payload, message.fingerprint);
        le_read(payload + 8, message.nonce);
        message.operation = static_cast<HilProfileOperation>(payload[12]);
        message.mav_type = payload[13];
        std::memcpy(
            message.profile_id.data(), payload + 14, message.profile_id.size());
        const bool terminated =
            std::memchr(message.profile_id.data(), '\0',
                        message.profile_id.size()) != nullptr;
        const bool known_operation =
            message.operation == HilProfileOperation::Select ||
            message.operation == HilProfileOperation::Ready ||
            message.operation == HilProfileOperation::Rejected;
        message.valid = terminated && message.profile_id[0] != '\0' &&
                        message.fingerprint != 0 && message.nonce != 0 &&
                        known_operation;
        return message;
    }

    HilSessionConfigMsg MavlinkHIL::parse_hil_session_config(
        const MavFrame &f) const
    {
        HilSessionConfigMsg message;
        if (f.msg_id != MSGID_HIL_SESSION_CONFIG ||
            f.payload.size() != HIL_SESSION_CONFIG_PAYLOAD_LEN)
            return message;
        const uint8_t *payload = f.payload.data();
        le_read(payload, message.profile_fingerprint);
        le_read(payload + 8, message.nonce);
        std::memcpy(message.digest.data(), payload + 12,
                    message.digest.size());
        if (!runtime::decode_hil_session_config(
                payload + 12 + message.digest.size(),
                runtime::kHilSessionConfigPayloadSize,
                message.config))
            return message;
        message.valid = message.profile_fingerprint != 0 &&
                        message.nonce != 0;
        message.digest_valid = message.valid &&
            runtime::hil_session_digest_equal(
                message.digest,
                runtime::hil_session_digest(
                    message.profile_fingerprint, message.config));
        return message;
    }

    HilSessionStatusMsg MavlinkHIL::parse_hil_session_status(
        const MavFrame &f) const
    {
        HilSessionStatusMsg message;
        if (f.msg_id != MSGID_HIL_SESSION_STATUS ||
            f.payload.size() != HIL_SESSION_STATUS_PAYLOAD_LEN)
            return message;
        const uint8_t *payload = f.payload.data();
        le_read(payload, message.profile_fingerprint);
        le_read(payload + 8, message.nonce);
        std::memcpy(message.digest.data(), payload + 12,
                    message.digest.size());
        message.operation =
            static_cast<HilSessionStatusOperation>(payload[44]);
        uint16_t reason = 0;
        uint16_t field = 0;
        le_read(payload + 45, reason);
        le_read(payload + 47, field);
        message.reason = static_cast<runtime::HilSessionRejectReason>(reason);
        message.field = static_cast<runtime::HilSessionField>(field);
        const bool known_operation =
            message.operation == HilSessionStatusOperation::Applying ||
            message.operation == HilSessionStatusOperation::Ready ||
            message.operation == HilSessionStatusOperation::Rejected;
        const bool consistent_reason =
            (message.operation == HilSessionStatusOperation::Rejected) ==
            (message.reason != runtime::HilSessionRejectReason::None);
        const bool known_reason =
            reason <= static_cast<uint16_t>(
                runtime::HilSessionRejectReason::InternalError);
        const bool known_field =
            field <= static_cast<uint16_t>(
                runtime::HilSessionField::Reserved);
        message.valid = message.profile_fingerprint != 0 &&
                        message.nonce != 0 &&
                        !runtime::hil_session_digest_is_zero(message.digest) &&
                        known_operation && consistent_reason &&
                        known_reason && known_field;
        return message;
    }

    HilTruthStateMsg MavlinkHIL::parse_hil_truth_state(const MavFrame &f) const
    {
        // Custom HIL_TRUTH_STATE: time_usec(8) + eta[6](48) + nu[6](48) = 104 bytes.
        const auto &p = f.payload;
        HilTruthStateMsg m;
        if (p.size() < 104)
            return m;
        const uint8_t *ptr = p.data();
        le_read(ptr, m.time_usec);
        for (int i = 0; i < 6; ++i)
        {
            le_read(ptr + 8 + i * 8, m.eta[i]);
            le_read(ptr + 56 + i * 8, m.nu[i]);
        }
        m.valid = true;
        return m;
    }

    HilFcStateMsg MavlinkHIL::parse_hil_fc_state(const MavFrame &f) const
    {
        HilFcStateMsg message;
        if (f.msg_id != MSGID_HIL_FC_STATE ||
            f.payload.size() != HIL_FC_STATE_PAYLOAD_LEN)
            return message;

        const uint8_t *data = f.payload.data();
        le_read(data, message.time_usec);
        for (std::size_t index = 0; index < 6; ++index)
        {
            le_read(data + 8 + index * sizeof(double), message.eta[index]);
            le_read(data + 56 + index * sizeof(double), message.nu[index]);
        }
        le_read(data + 104, message.depth_m);
        for (std::size_t index = 0; index < 8; ++index)
            le_read(data + 112 + index * sizeof(float), message.normalized[index]);
        for (std::size_t index = 0; index < 3; ++index)
        {
            le_read(data + 144 + index * sizeof(float), message.dvl_vel[index]);
            le_read(data + 156 + index * sizeof(float), message.acc[index]);
            le_read(data + 168 + index * sizeof(float), message.gyro[index]);
            le_read(data + 192 + index * sizeof(float), message.gps_vel_ned[index]);
        }
        le_read(data + 180, message.gps_lat);
        le_read(data + 184, message.gps_lon);
        le_read(data + 188, message.gps_alt);
        le_read(data + 204, message.motor_rpm_actual);
        le_read(data + 208, message.motor_thrust_N);
        le_read(data + 212, message.motor_power_W);
        le_read(data + 216, message.motor_current_A);
        le_read(data + 220, message.power_total_W);
        le_read(data + 224, message.energy_Wh);
        le_read(data + 228, message.battery_soc);
        le_read(data + 232, message.voltage_terminal);
        le_read(data + 236, message.runtime_remaining_s);
        le_read(data + 240, message.commanded_rpm);
        le_read(data + 244, message.tick);
        le_read(data + 248, message.flags);
        message.mode = data[250];
        message.mission_state = data[251];
        message.actuator_channel_count = data[252];
        message.gps_fix = data[253];
        message.gps_satellites = data[254];

        constexpr uint16_t known_flags =
            HIL_FC_STATE_FLAG_DVL_VALID |
            HIL_FC_STATE_FLAG_EKF_INITIALIZED |
            HIL_FC_STATE_FLAG_ARMED |
            HIL_FC_STATE_FLAG_ACTUATOR_AUTHORIZED |
            HIL_FC_STATE_FLAG_EXTERNAL_SETPOINT;
        const auto finite_doubles = [](const double *values, std::size_t count)
        {
            for (std::size_t index = 0; index < count; ++index)
                if (!std::isfinite(values[index])) return false;
            return true;
        };
        const auto finite_floats = [](const float *values, std::size_t count)
        {
            for (std::size_t index = 0; index < count; ++index)
                if (!std::isfinite(values[index])) return false;
            return true;
        };
        message.valid = message.time_usec > 0 && message.tick > 0 &&
            message.mode <= 4 && message.mission_state <= 3 &&
            message.actuator_channel_count <= 8 &&
            (message.flags & ~known_flags) == 0 &&
            finite_doubles(message.eta, 6) &&
            finite_doubles(message.nu, 6) &&
            std::isfinite(message.depth_m) &&
            finite_floats(message.normalized, 8) &&
            finite_floats(message.dvl_vel, 3) &&
            finite_floats(message.acc, 3) &&
            finite_floats(message.gyro, 3) &&
            finite_floats(message.gps_vel_ned, 3) &&
            std::isfinite(message.motor_rpm_actual) &&
            std::isfinite(message.motor_thrust_N) &&
            std::isfinite(message.motor_power_W) &&
            std::isfinite(message.motor_current_A) &&
            std::isfinite(message.power_total_W) &&
            std::isfinite(message.energy_Wh) &&
            std::isfinite(message.battery_soc) &&
            std::isfinite(message.voltage_terminal) &&
            std::isfinite(message.runtime_remaining_s) &&
            std::isfinite(message.commanded_rpm);
        return message;
    }

    HilControlTraceMsg MavlinkHIL::parse_hil_control_trace(
        const MavFrame &f) const
    {
        HilControlTraceMsg message;
        if (f.msg_id != MSGID_HIL_CONTROL_TRACE ||
            f.payload.size() != HIL_CONTROL_TRACE_PAYLOAD_LEN)
            return message;

        const uint8_t *data = f.payload.data();
        le_read(data, message.time_usec);
        le_read(data + 8, message.tick);
        le_read(data + 12, message.dt_s);
        for (std::size_t index = 0; index < 6; ++index)
        {
            le_read(data + 20 + index * sizeof(double), message.eta[index]);
            le_read(data + 68 + index * sizeof(double), message.nu[index]);
            le_read(data + 212 + index * sizeof(float), message.wrench[index]);
        }
        le_read(data + 116, message.depth_m);
        le_read(data + 124, message.depth_ref);
        le_read(data + 132, message.heading_ref);
        le_read(data + 140, message.surge_ref);
        le_read(data + 148, message.yaw_rate_ref);
        le_read(data + 156, message.wp_n);
        le_read(data + 164, message.wp_e);
        le_read(data + 172, message.wp_d);
        le_read(data + 180, message.path_start_n);
        le_read(data + 188, message.path_start_e);
        le_read(data + 196, message.lookahead_m);
        le_read(data + 204, message.arrival_radius_m);
        le_read(data + 236, message.setpoint_age_s);
        le_read(data + 240, message.flags);
        message.mode = data[242];
        message.mission_state = data[243];
        message.actuator_channel_count = data[244];

        constexpr uint16_t known_flags =
            HIL_CONTROL_TRACE_FLAG_USE_YAW_RATE |
            HIL_CONTROL_TRACE_FLAG_USE_PATH_SEGMENT |
            HIL_CONTROL_TRACE_FLAG_HOLD_HEADING |
            HIL_CONTROL_TRACE_FLAG_CONTROLLER_RESET |
            HIL_CONTROL_TRACE_FLAG_ACTUATOR_AUTHORIZED |
            HIL_CONTROL_TRACE_FLAG_USED_TRUTH |
            HIL_CONTROL_TRACE_FLAG_EXTERNAL_SETPOINT |
            HIL_CONTROL_TRACE_FLAG_DVL_VALID |
            HIL_CONTROL_TRACE_FLAG_EKF_INITIALIZED;
        const auto finite_doubles = [](const double *values, std::size_t count)
        {
            for (std::size_t index = 0; index < count; ++index)
                if (!std::isfinite(values[index])) return false;
            return true;
        };
        const auto finite_floats = [](const float *values, std::size_t count)
        {
            for (std::size_t index = 0; index < count; ++index)
                if (!std::isfinite(values[index])) return false;
            return true;
        };
        const double setpoint_values[] = {
            message.depth_ref, message.heading_ref, message.surge_ref,
            message.yaw_rate_ref, message.wp_n, message.wp_e, message.wp_d,
            message.path_start_n, message.path_start_e,
            message.lookahead_m, message.arrival_radius_m};
        message.valid = message.time_usec > 0 && message.tick > 0 &&
            message.dt_s > 0.0 && message.dt_s <= 1.0 &&
            message.mode <= 4 && message.mission_state <= 3 &&
            message.actuator_channel_count <= 8 &&
            (message.flags & ~known_flags) == 0 &&
            data[245] == 0 && data[246] == 0 && data[247] == 0 &&
            finite_doubles(message.eta, 6) &&
            finite_doubles(message.nu, 6) &&
            std::isfinite(message.depth_m) &&
            finite_doubles(setpoint_values, 11) &&
            finite_floats(message.wrench, 6) &&
            std::isfinite(message.setpoint_age_s) &&
            message.setpoint_age_s >= -1.0f;
        return message;
    }

    HilActuatorControlsMsg MavlinkHIL::parse_hil_actuator_controls(
        const MavFrame &f) const
    {
        // MAVLink HIL_ACTUATOR_CONTROLS wire layout is fixed at 81 bytes.
        const auto &p = f.payload;
        HilActuatorControlsMsg message;
        if (f.msg_id != MSGID_HIL_ACTUATOR_CONTROLS || p.size() < 81)
            return message;
        const uint8_t *data = p.data();
        le_read(data, message.time_usec);
        for (std::size_t index = 0; index < message.controls.size(); ++index)
            le_read(data + 8 + index * sizeof(float), message.controls[index]);
        le_read(data + 72, message.flags);
        le_read(data + 80, message.mode);
        message.valid = std::all_of(
            message.controls.begin(), message.controls.end(),
            [](float value) { return std::isfinite(value); });
        return message;
    }

    HilPassiveSonarBearingMsg MavlinkHIL::parse_hil_passive_sonar(const MavFrame &f) const
    {
        // Custom HIL_PASSIVE_SONAR:
        // time_usec(8) valid(1) target_slot(4) azimuth(4) elevation(4) = 21 bytes.
        const auto &p = f.payload;
        HilPassiveSonarBearingMsg m;
        if (p.size() < 21)
            return m;
        const uint8_t *ptr = p.data();
        uint8_t valid = 0;
        le_read(ptr, m.time_usec);
        le_read(ptr + 8, valid);
        le_read(ptr + 9, m.target_slot);
        le_read(ptr + 13, m.azimuth_rad);
        le_read(ptr + 17, m.elevation_rad);
        m.valid = (valid != 0);
        return m;
    }

    HilAcousticNeighborsMsg MavlinkHIL::parse_hil_acoustic_neighbors(const MavFrame &f) const
    {
        // Custom HIL_ACOUSTIC_NEIGHBORS:
        // header: time_usec(8) receiver_id(4) count(1)
        // contact: sender_id(4), range(4), delay(4), depth(4), position(12),
        //          velocity(12), yaw(4), payload[3](12) = 56 bytes.
        constexpr size_t header_bytes = 13;
        constexpr size_t contact_bytes = 56;
        const auto &p = f.payload;
        HilAcousticNeighborsMsg m;
        if (p.size() < header_bytes)
            return m;

        const uint8_t *ptr = p.data();
        uint8_t count = 0;
        le_read(ptr, m.time_usec);
        le_read(ptr + 8, m.receiver_id);
        le_read(ptr + 12, count);
        const size_t available =
            (p.size() - header_bytes) / contact_bytes;
        const size_t n = std::min<size_t>(
            std::min<size_t>(count, available), HIL_MAX_ACOUSTIC_CONTACTS);
        m.contacts.reserve(n);
        for (size_t i = 0; i < n; ++i)
        {
            const uint8_t *c = p.data() + header_bytes + i * contact_bytes;
            HilAcousticContactMsg contact;
            le_read(c + 0, contact.sender_id);
            le_read(c + 4, contact.range_m);
            le_read(c + 8, contact.propagation_delay_s);
            le_read(c + 12, contact.depth_m);
            for (int k = 0; k < 3; ++k)
                le_read(c + 16 + k * 4, contact.position_ned[k]);
            for (int k = 0; k < 3; ++k)
                le_read(c + 28 + k * 4, contact.velocity_ned[k]);
            le_read(c + 40, contact.yaw_ned_rad);
            for (int k = 0; k < 3; ++k)
                le_read(c + 44 + k * 4, contact.payload[k]);
            contact.valid = true;
            (void)m.contacts.push_back(contact);
        }
        return m;
    }

    HilRangeFinderScanMsg MavlinkHIL::parse_hil_rangefinder_scan(const MavFrame &f) const
    {
        // Custom HIL_RANGEFINDER_SCAN:
        // time_usec(8) ray_count(4) max_range_m(4) range_m[ray_count](4 each).
        constexpr size_t header_bytes = 16;
        const auto &p = f.payload;
        HilRangeFinderScanMsg m;
        if (p.size() < header_bytes)
            return m;

        const uint8_t *ptr = p.data();
        le_read(ptr, m.time_usec);
        le_read(ptr + 8, m.ray_count);
        le_read(ptr + 12, m.max_range_m);
        const size_t available = (p.size() - header_bytes) / sizeof(float);
        const size_t n = std::min<size_t>(
            std::min<size_t>(m.ray_count, available),
            HIL_MAX_RANGEFINDER_RAYS);
        m.ranges_m.reserve(n);
        for (size_t i = 0; i < n; ++i)
        {
            float range_m = 0.0f;
            le_read(p.data() + header_bytes + i * sizeof(float), range_m);
            (void)m.ranges_m.push_back(range_m);
        }
        m.ray_count = static_cast<uint32_t>(m.ranges_m.size());
        m.valid = m.time_usec > 0 && m.max_range_m > 0.0f && !m.ranges_m.empty();
        return m;
    }

    // Encode and send messages
    std::vector<uint8_t> MavlinkHIL::_encode_frame(uint32_t msg_id,
                                                   const uint8_t *payload,
                                                   size_t payload_len) const
    {
        MavlinkPacket packet;
        if (!_encode_frame(packet, msg_id, payload, payload_len))
            return {};
        return std::vector<uint8_t>(
            packet.bytes.begin(), packet.bytes.begin() + packet.length);
    }

    bool MavlinkHIL::_encode_frame(MavlinkPacket &packet,
                                   uint32_t msg_id,
                                   const uint8_t *payload,
                                   size_t payload_len) const
    {
        packet.length = 0;
        if (payload_len > MAVLINK_MAX_PAYLOAD_LEN ||
            (payload == nullptr && payload_len != 0))
        {
            return false;
        }

        const bool sign_outgoing = _signing.valid() && _signing.sign_outgoing;
        const size_t expected_length = 10 + payload_len + 2 +
            (sign_outgoing ? MAVLINK_SIGNATURE_BLOCK_LEN : 0);
        if (expected_length > packet.bytes.size())
            return false;

        auto append = [&](uint8_t value)
        {
            packet.bytes[packet.length++] = value;
        };
        append(0xFD);
        append(static_cast<uint8_t>(payload_len));
        append(sign_outgoing ? MAVLINK_IFLAG_SIGNED : 0);
        append(0);
        append(_seq++);
        append(_sysid);
        append(_compid);
        append(static_cast<uint8_t>(msg_id & 0xFF));
        append(static_cast<uint8_t>((msg_id >> 8) & 0xFF));
        append(static_cast<uint8_t>((msg_id >> 16) & 0xFF));
        if (payload_len > 0)
        {
            std::memcpy(packet.bytes.data() + packet.length,
                        payload, payload_len);
            packet.length += payload_len;
        }

        // CRC: From frame[1] (LEN) to the end of payload
        uint16_t crc = _crc16(packet.data() + 1, 9 + payload_len);
        uint8_t ex = crc_extra_table(msg_id);
        crc = _crc16(&ex, 1, crc);
        append(static_cast<uint8_t>(crc & 0xFF));
        append(static_cast<uint8_t>(crc >> 8));

        if (sign_outgoing)
        {
            const uint64_t now = mavlink_signing_timestamp_10us();
            const uint64_t timestamp = std::max(
                now,
                _tx_signing_timestamp == std::numeric_limits<uint64_t>::max()
                    ? _tx_signing_timestamp
                    : _tx_signing_timestamp + 1);
            _tx_signing_timestamp = timestamp;

            std::array<uint8_t, 32> digest{};
            if (!mavlink_signing_digest(
                    _signing.secret_key,
                    packet.data() + 1,
                    packet.size() - 1,
                    _signing.link_id,
                    timestamp,
                    digest))
            {
                packet.length = 0;
                return false;
            }
            append(_signing.link_id);
            for (int i = 0; i < 6; ++i)
                append(static_cast<uint8_t>((timestamp >> (8 * i)) & 0xFFu));
            for (int i = 0; i < 6; ++i)
                append(digest[static_cast<size_t>(i)]);
        }
        return packet.length == expected_length;
    }

    std::vector<uint8_t> MavlinkHIL::encode_hil_actuator_controls(
        const std::array<float, 8> &controls,
        uint64_t time_usec,
        uint8_t mode,
        uint64_t flags) const
    {
        // HIL_ACTUATOR_CONTROLS (93), MAVLink generated wire layout:
        // [0..7]   time_usec (u64)
        // [8..71]  controls[0..15] (f32×16)
        // [72..79] flags (u64)
        // [80]     mode (u8)
        uint8_t buf[81] = {};
        memcpy(buf, &time_usec, 8);
        for (int i = 0; i < 8; ++i)
            memcpy(buf + 8 + i * 4, &controls[i], 4);
        memcpy(buf + 72, &flags, 8);
        buf[80] = mode;
        return _encode_frame(MSGID_HIL_ACTUATOR_CONTROLS, buf, 81);
    }

    bool MavlinkHIL::encode_hil_actuator_controls(
        MavlinkPacket &packet,
        const std::array<float, 8> &controls,
        uint64_t time_usec,
        uint8_t mode,
        uint64_t flags) const
    {
        uint8_t buf[81] = {};
        std::memcpy(buf, &time_usec, 8);
        for (int i = 0; i < 8; ++i)
            std::memcpy(buf + 8 + i * 4, &controls[i], 4);
        std::memcpy(buf + 72, &flags, 8);
        buf[80] = mode;
        return _encode_frame(
            packet, MSGID_HIL_ACTUATOR_CONTROLS, buf, sizeof(buf));
    }

    bool MavlinkHIL::encode_hil_profile(
        MavlinkPacket &packet, const HilProfileMsg &message) const
    {
        packet.length = 0;
        const bool terminated =
            std::memchr(message.profile_id.data(), '\0',
                        message.profile_id.size()) != nullptr;
        const bool known_operation =
            message.operation == HilProfileOperation::Select ||
            message.operation == HilProfileOperation::Ready ||
            message.operation == HilProfileOperation::Rejected;
        if (!message.valid || !terminated || message.profile_id[0] == '\0' ||
            message.fingerprint == 0 || message.nonce == 0 || !known_operation)
            return false;
        uint8_t payload[HIL_PROFILE_PAYLOAD_LEN] = {};
        std::memcpy(payload, &message.fingerprint, sizeof(message.fingerprint));
        std::memcpy(payload + 8, &message.nonce, sizeof(message.nonce));
        payload[12] = static_cast<uint8_t>(message.operation);
        payload[13] = message.mav_type;
        std::memcpy(payload + 14, message.profile_id.data(), message.profile_id.size());
        return _encode_frame(packet, MSGID_HIL_PROFILE, payload, sizeof(payload));
    }

    bool MavlinkHIL::encode_hil_session_config(
        MavlinkPacket &packet, const HilSessionConfigMsg &message) const
    {
        packet.length = 0;
        runtime::HilSessionField field = runtime::HilSessionField::None;
        const runtime::HilSessionDigest expected =
            runtime::hil_session_digest(
                message.profile_fingerprint, message.config);
        if (!message.valid || message.profile_fingerprint == 0 ||
            message.nonce == 0 ||
            !runtime::validate_hil_session_config(message.config, field) ||
            !runtime::hil_session_digest_equal(message.digest, expected))
            return false;

        uint8_t payload[HIL_SESSION_CONFIG_PAYLOAD_LEN] = {};
        std::memcpy(payload, &message.profile_fingerprint,
                    sizeof(message.profile_fingerprint));
        std::memcpy(payload + 8, &message.nonce, sizeof(message.nonce));
        std::memcpy(payload + 12, message.digest.data(), message.digest.size());
        const runtime::HilSessionConfigPayload config_payload =
            runtime::encode_hil_session_config(message.config);
        std::memcpy(payload + 12 + message.digest.size(),
                    config_payload.data(), config_payload.size());
        return _encode_frame(
            packet, MSGID_HIL_SESSION_CONFIG, payload, sizeof(payload));
    }

    bool MavlinkHIL::encode_hil_session_status(
        MavlinkPacket &packet, const HilSessionStatusMsg &message) const
    {
        packet.length = 0;
        const bool known_operation =
            message.operation == HilSessionStatusOperation::Applying ||
            message.operation == HilSessionStatusOperation::Ready ||
            message.operation == HilSessionStatusOperation::Rejected;
        const bool consistent_reason =
            (message.operation == HilSessionStatusOperation::Rejected) ==
            (message.reason != runtime::HilSessionRejectReason::None);
        const bool known_reason =
            static_cast<uint16_t>(message.reason) <=
            static_cast<uint16_t>(
                runtime::HilSessionRejectReason::InternalError);
        const bool known_field =
            static_cast<uint16_t>(message.field) <=
            static_cast<uint16_t>(runtime::HilSessionField::Reserved);
        if (!message.valid || message.profile_fingerprint == 0 ||
            message.nonce == 0 ||
            runtime::hil_session_digest_is_zero(message.digest) ||
            !known_operation || !consistent_reason ||
            !known_reason || !known_field)
            return false;

        uint8_t payload[HIL_SESSION_STATUS_PAYLOAD_LEN] = {};
        std::memcpy(payload, &message.profile_fingerprint,
                    sizeof(message.profile_fingerprint));
        std::memcpy(payload + 8, &message.nonce, sizeof(message.nonce));
        std::memcpy(payload + 12, message.digest.data(), message.digest.size());
        payload[44] = static_cast<uint8_t>(message.operation);
        const uint16_t reason = static_cast<uint16_t>(message.reason);
        const uint16_t field = static_cast<uint16_t>(message.field);
        std::memcpy(payload + 45, &reason, sizeof(reason));
        std::memcpy(payload + 47, &field, sizeof(field));
        return _encode_frame(
            packet, MSGID_HIL_SESSION_STATUS, payload, sizeof(payload));
    }

    bool MavlinkHIL::encode_hil_fc_state(
        MavlinkPacket &packet, const HilFcStateMsg &message) const
    {
        packet.length = 0;
        constexpr uint16_t known_flags =
            HIL_FC_STATE_FLAG_DVL_VALID |
            HIL_FC_STATE_FLAG_EKF_INITIALIZED |
            HIL_FC_STATE_FLAG_ARMED |
            HIL_FC_STATE_FLAG_ACTUATOR_AUTHORIZED |
            HIL_FC_STATE_FLAG_EXTERNAL_SETPOINT;
        if (!message.valid || message.time_usec == 0 || message.tick == 0 ||
            message.mode > 4 || message.mission_state > 3 ||
            message.actuator_channel_count > 8 ||
            (message.flags & ~known_flags) != 0)
            return false;

        uint8_t payload[HIL_FC_STATE_PAYLOAD_LEN] = {};
        std::memcpy(payload, &message.time_usec, sizeof(message.time_usec));
        for (std::size_t index = 0; index < 6; ++index)
        {
            std::memcpy(payload + 8 + index * sizeof(double),
                        &message.eta[index], sizeof(double));
            std::memcpy(payload + 56 + index * sizeof(double),
                        &message.nu[index], sizeof(double));
        }
        std::memcpy(payload + 104, &message.depth_m, sizeof(message.depth_m));
        for (std::size_t index = 0; index < 8; ++index)
            std::memcpy(payload + 112 + index * sizeof(float),
                        &message.normalized[index], sizeof(float));
        for (std::size_t index = 0; index < 3; ++index)
        {
            std::memcpy(payload + 144 + index * sizeof(float),
                        &message.dvl_vel[index], sizeof(float));
            std::memcpy(payload + 156 + index * sizeof(float),
                        &message.acc[index], sizeof(float));
            std::memcpy(payload + 168 + index * sizeof(float),
                        &message.gyro[index], sizeof(float));
            std::memcpy(payload + 192 + index * sizeof(float),
                        &message.gps_vel_ned[index], sizeof(float));
        }
        std::memcpy(payload + 180, &message.gps_lat, sizeof(message.gps_lat));
        std::memcpy(payload + 184, &message.gps_lon, sizeof(message.gps_lon));
        std::memcpy(payload + 188, &message.gps_alt, sizeof(message.gps_alt));
        std::memcpy(payload + 204, &message.motor_rpm_actual, sizeof(float));
        std::memcpy(payload + 208, &message.motor_thrust_N, sizeof(float));
        std::memcpy(payload + 212, &message.motor_power_W, sizeof(float));
        std::memcpy(payload + 216, &message.motor_current_A, sizeof(float));
        std::memcpy(payload + 220, &message.power_total_W, sizeof(float));
        std::memcpy(payload + 224, &message.energy_Wh, sizeof(float));
        std::memcpy(payload + 228, &message.battery_soc, sizeof(float));
        std::memcpy(payload + 232, &message.voltage_terminal, sizeof(float));
        std::memcpy(payload + 236, &message.runtime_remaining_s, sizeof(float));
        std::memcpy(payload + 240, &message.commanded_rpm, sizeof(float));
        std::memcpy(payload + 244, &message.tick, sizeof(message.tick));
        std::memcpy(payload + 248, &message.flags, sizeof(message.flags));
        payload[250] = message.mode;
        payload[251] = message.mission_state;
        payload[252] = message.actuator_channel_count;
        payload[253] = message.gps_fix;
        payload[254] = message.gps_satellites;
        return _encode_frame(
            packet, MSGID_HIL_FC_STATE, payload, sizeof(payload));
    }

    bool MavlinkHIL::encode_hil_control_trace(
        MavlinkPacket &packet, const HilControlTraceMsg &message) const
    {
        packet.length = 0;
        constexpr uint16_t known_flags =
            HIL_CONTROL_TRACE_FLAG_USE_YAW_RATE |
            HIL_CONTROL_TRACE_FLAG_USE_PATH_SEGMENT |
            HIL_CONTROL_TRACE_FLAG_HOLD_HEADING |
            HIL_CONTROL_TRACE_FLAG_CONTROLLER_RESET |
            HIL_CONTROL_TRACE_FLAG_ACTUATOR_AUTHORIZED |
            HIL_CONTROL_TRACE_FLAG_USED_TRUTH |
            HIL_CONTROL_TRACE_FLAG_EXTERNAL_SETPOINT |
            HIL_CONTROL_TRACE_FLAG_DVL_VALID |
            HIL_CONTROL_TRACE_FLAG_EKF_INITIALIZED;
        const auto finite_doubles = [](const double *values, std::size_t count)
        {
            for (std::size_t index = 0; index < count; ++index)
                if (!std::isfinite(values[index])) return false;
            return true;
        };
        const auto finite_floats = [](const float *values, std::size_t count)
        {
            for (std::size_t index = 0; index < count; ++index)
                if (!std::isfinite(values[index])) return false;
            return true;
        };
        const double setpoint_values[] = {
            message.depth_ref, message.heading_ref, message.surge_ref,
            message.yaw_rate_ref, message.wp_n, message.wp_e, message.wp_d,
            message.path_start_n, message.path_start_e,
            message.lookahead_m, message.arrival_radius_m};
        if (!message.valid || message.time_usec == 0 || message.tick == 0 ||
            !std::isfinite(message.dt_s) ||
            message.dt_s <= 0.0 || message.dt_s > 1.0 ||
            message.mode > 4 || message.mission_state > 3 ||
            message.actuator_channel_count > 8 ||
            (message.flags & ~known_flags) != 0 ||
            !finite_doubles(message.eta, 6) ||
            !finite_doubles(message.nu, 6) ||
            !std::isfinite(message.depth_m) ||
            !finite_doubles(setpoint_values, 11) ||
            !finite_floats(message.wrench, 6) ||
            !std::isfinite(message.setpoint_age_s) ||
            message.setpoint_age_s < -1.0f)
            return false;

        uint8_t payload[HIL_CONTROL_TRACE_PAYLOAD_LEN] = {};
        std::memcpy(payload, &message.time_usec, sizeof(message.time_usec));
        std::memcpy(payload + 8, &message.tick, sizeof(message.tick));
        std::memcpy(payload + 12, &message.dt_s, sizeof(message.dt_s));
        for (std::size_t index = 0; index < 6; ++index)
        {
            std::memcpy(payload + 20 + index * sizeof(double),
                        &message.eta[index], sizeof(double));
            std::memcpy(payload + 68 + index * sizeof(double),
                        &message.nu[index], sizeof(double));
            std::memcpy(payload + 212 + index * sizeof(float),
                        &message.wrench[index], sizeof(float));
        }
        std::memcpy(payload + 116, &message.depth_m, sizeof(double));
        std::memcpy(payload + 124, &message.depth_ref, sizeof(double));
        std::memcpy(payload + 132, &message.heading_ref, sizeof(double));
        std::memcpy(payload + 140, &message.surge_ref, sizeof(double));
        std::memcpy(payload + 148, &message.yaw_rate_ref, sizeof(double));
        std::memcpy(payload + 156, &message.wp_n, sizeof(double));
        std::memcpy(payload + 164, &message.wp_e, sizeof(double));
        std::memcpy(payload + 172, &message.wp_d, sizeof(double));
        std::memcpy(payload + 180, &message.path_start_n, sizeof(double));
        std::memcpy(payload + 188, &message.path_start_e, sizeof(double));
        std::memcpy(payload + 196, &message.lookahead_m, sizeof(double));
        std::memcpy(payload + 204, &message.arrival_radius_m, sizeof(double));
        std::memcpy(payload + 236, &message.setpoint_age_s, sizeof(float));
        std::memcpy(payload + 240, &message.flags, sizeof(message.flags));
        payload[242] = message.mode;
        payload[243] = message.mission_state;
        payload[244] = message.actuator_channel_count;
        return _encode_frame(
            packet, MSGID_HIL_CONTROL_TRACE, payload, sizeof(payload));
    }

    std::vector<uint8_t> MavlinkHIL::encode_heartbeat(uint8_t mav_type) const
    {
        // HEARTBEAT (0): custom_mode(4) type(1) autopilot(1) base_mode(1)
        //                system_status(1) mavlink_version(1) = 9 bytes
        uint8_t buf[9] = {};
        // custom_mode = 0
        buf[4] = mav_type;
        buf[5] = 0;  // MAV_AUTOPILOT_GENERIC
        buf[6] = 0;  // base_mode
        buf[7] = 4;  // MAV_STATE_ACTIVE
        buf[8] = 3;  // mavlink_version
        return _encode_frame(MSGID_HEARTBEAT, buf, 9);
    }

    bool MavlinkHIL::encode_heartbeat(
        MavlinkPacket &packet, uint8_t mav_type) const
    {
        uint8_t buf[9] = {};
        buf[4] = mav_type;
        buf[5] = 0;
        buf[6] = 0;
        buf[7] = 4;
        buf[8] = 3;
        return _encode_frame(packet, MSGID_HEARTBEAT, buf, sizeof(buf));
    }

} // namespace hydrox
