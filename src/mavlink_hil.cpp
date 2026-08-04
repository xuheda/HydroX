/**
 * mavlink_hil.cpp — MAVLink HIL message encoding and decoding implementation
 */
#include "mavlink_hil.h"
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
                // A valid MAVLink 2 packet fits in the fixed buffer. Overflow
                // therefore means corrupt input; resynchronise at this byte.
                _buf.clear();
                if (data[i] == 0xFD)
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
        // Find MAVLink2 frame header 0xFD
        while (!_buf.empty() && _buf[0] != 0xFD)
            _buf.erase(_buf.begin());

        if (_buf.size() < 12)
            return std::nullopt; // Minimal frame size is 12 bytes

        uint8_t payload_len = _buf[1];
        const uint8_t incompat_flags = _buf[2];
        if ((incompat_flags & ~MAVLINK_IFLAG_SIGNED) != 0)
        {
            // Unknown incompatibility flags must be discarded by MAVLink 2
            // receivers. Drop this magic byte and resynchronise safely.
            _buf.erase(_buf.begin());
            return _parse_one();
        }
        const bool signed_frame = (incompat_flags & MAVLINK_IFLAG_SIGNED) != 0;
        const size_t base_frame_len = 10u + payload_len + 2u;
        const size_t frame_len = base_frame_len +
            (signed_frame ? MAVLINK_SIGNATURE_BLOCK_LEN : 0u);

        if (_buf.size() < frame_len)
            return std::nullopt;

        // msg_id: 3-byte little-endian (offsets 7, 8, 9)
        uint32_t msg_id = static_cast<uint32_t>(_buf[7]) | (static_cast<uint32_t>(_buf[8]) << 8) | (static_cast<uint32_t>(_buf[9]) << 16);

        // CRC verification (starting from payload_len byte, to the end of payload)
        uint16_t crc_rcv = static_cast<uint16_t>(_buf[10 + payload_len]) | (static_cast<uint16_t>(_buf[10 + payload_len + 1]) << 8);

        // CRC coverage: frame[1] to frame[9+payload_len] (excluding header 0xFD and CRC)
        uint16_t crc_calc = _crc16(_buf.data() + 1, 9 + payload_len);
        uint8_t extra = crc_extra_table(msg_id);
        crc_calc = _crc16(&extra, 1, crc_calc);

        if (crc_rcv != crc_calc)
        {
            static int bad_crc_count = 0;
            if (++bad_crc_count <= 10)
                std::fprintf(stderr,
                             "[MavlinkHIL] bad CRC #%d msg=%u len=%u rcv=0x%04x calc=0x%04x\n",
                             bad_crc_count, msg_id, payload_len, crc_rcv, crc_calc);
            _buf.erase(_buf.begin(), _buf.begin() + static_cast<int>(frame_len));
            return _parse_one(); // Discard bad frame, continue parsing subsequent frames in the same buffer
        }

        const uint8_t sysid = _buf[5];
        const uint8_t compid = _buf[6];
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
        frame.signed_frame = signed_frame;
        (void)frame.payload.assign(_buf.begin() + 10,
                                   _buf.begin() + 10 + payload_len);
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
        // [48]: diff_pressure, [52]: pressure_alt, [56]: temperature (unused)
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

    std::vector<uint8_t> MavlinkHIL::encode_attitude(
        float roll, float pitch, float yaw,
        float rollspeed, float pitchspeed, float yawspeed,
        uint32_t time_boot_ms) const
    {
        // ATTITUDE (30): time_boot_ms(4) roll(4) pitch(4) yaw(4)
        //                rollspeed(4) pitchspeed(4) yawspeed(4) = 28 bytes
        uint8_t buf[28] = {};
        memcpy(buf + 0, &time_boot_ms, 4);
        memcpy(buf + 4, &roll, 4);
        memcpy(buf + 8, &pitch, 4);
        memcpy(buf + 12, &yaw, 4);
        memcpy(buf + 16, &rollspeed, 4);
        memcpy(buf + 20, &pitchspeed, 4);
        memcpy(buf + 24, &yawspeed, 4);
        return _encode_frame(MSGID_ATTITUDE, buf, 28);
    }

    std::vector<uint8_t> MavlinkHIL::encode_local_position_ned(
        float x, float y, float z,
        float vx, float vy, float vz,
        uint32_t time_boot_ms) const
    {
        // LOCAL_POSITION_NED (32): time_boot_ms(4) x(4) y(4) z(4)
        //                          vx(4) vy(4) vz(4) = 28 bytes
        uint8_t buf[28] = {};
        memcpy(buf + 0, &time_boot_ms, 4);
        memcpy(buf + 4, &x, 4);
        memcpy(buf + 8, &y, 4);
        memcpy(buf + 12, &z, 4);
        memcpy(buf + 16, &vx, 4);
        memcpy(buf + 20, &vy, 4);
        memcpy(buf + 24, &vz, 4);
        return _encode_frame(MSGID_LOCAL_POSITION_NED, buf, 28);
    }

    std::vector<uint8_t> MavlinkHIL::encode_vfr_hud(
        float airspeed, float groundspeed, float alt, float climb,
        int16_t heading_deg, uint16_t throttle_pct) const
    {
        // VFR_HUD (74): airspeed(4) groundspeed(4) alt(4) climb(4)
        //               heading(2) throttle(2) = 20 bytes
        uint8_t buf[20] = {};
        memcpy(buf + 0, &airspeed, 4);
        memcpy(buf + 4, &groundspeed, 4);
        memcpy(buf + 8, &alt, 4);
        memcpy(buf + 12, &climb, 4);
        memcpy(buf + 16, &heading_deg, 2);
        memcpy(buf + 18, &throttle_pct, 2);
        return _encode_frame(MSGID_VFR_HUD, buf, 20);
    }

    std::vector<uint8_t> MavlinkHIL::encode_sys_status(
        uint32_t sensors_present, uint32_t sensors_enabled,
        uint32_t sensors_health, uint16_t load_permille,
        int16_t battery_pct) const
    {
        // SYS_STATUS (1): sensors_present(4) sensors_enabled(4) sensors_health(4)
        // load(2) voltage_battery(2) current_battery(2) battery_remaining(1)
        // drop_rate_comm(2) errors_comm(2) errors_count1..4(8) = 31 bytes
        uint8_t buf[31] = {};
        memcpy(buf + 0, &sensors_present, 4);
        memcpy(buf + 4, &sensors_enabled, 4);
        memcpy(buf + 8, &sensors_health, 4);
        memcpy(buf + 12, &load_permille, 2); // CPU load permille
        // voltage/current left as 0 (no battery)
        memcpy(buf + 30, &battery_pct, 1); // -1 = unknown
        return _encode_frame(MSGID_SYS_STATUS, buf, 31);
    }

    std::vector<uint8_t> MavlinkHIL::encode_global_position_int(
        int32_t lat_deg7, int32_t lon_deg7,
        int32_t alt_mm, int32_t relative_alt_mm,
        int16_t vx_cms, int16_t vy_cms, int16_t vz_cms,
        uint16_t hdg_cdeg, uint32_t time_boot_ms) const
    {
        // GLOBAL_POSITION_INT (33): time_boot_ms(4) lat(4) lon(4) alt(4)
        // relative_alt(4) vx(2) vy(2) vz(2) hdg(2) = 28 bytes
        uint8_t buf[28] = {};
        memcpy(buf + 0, &time_boot_ms, 4);
        memcpy(buf + 4, &lat_deg7, 4);
        memcpy(buf + 8, &lon_deg7, 4);
        memcpy(buf + 12, &alt_mm, 4);
        memcpy(buf + 16, &relative_alt_mm, 4);
        memcpy(buf + 20, &vx_cms, 2);
        memcpy(buf + 22, &vy_cms, 2);
        memcpy(buf + 24, &vz_cms, 2);
        memcpy(buf + 26, &hdg_cdeg, 2);
        return _encode_frame(MSGID_GLOBAL_POSITION_INT, buf, 28);
    }

    std::vector<uint8_t> MavlinkHIL::encode_statustext(
        uint8_t severity, const char *text) const
    {
        // STATUSTEXT (253): severity(1) + text[50] = 51 bytes
        uint8_t buf[51] = {};
        buf[0] = severity;
        const size_t len = std::min(strlen(text), static_cast<size_t>(50));
        memcpy(buf + 1, text, len);
        return _encode_frame(MSGID_STATUSTEXT, buf, 51);
    }

} // namespace hydrox
