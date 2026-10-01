#pragma once
/**
 * mavlink_hil.h — MAVLink HIL message encoding and decoding
 *
 * Supported messages:
 *   Input (HIL Bridge -> fc):
 *     HIL_SENSOR        (107)   IMU + depth sensor
 *     HIL_GPS           (113)   GPS
 *     HIL_DVL           (11060) DVL body velocity (custom)
 *     HIL_TRUTH_STATE   (11061) Simulator truth state (custom debug)
 *     HEARTBEAT         (0)
 *
 *   Output (fc -> HIL Bridge):
 *     HIL_ACTUATOR_CONTROLS (93) Normalized simulator actuator outputs
 *     HIL_FC_STATE      (11069) Estimated state and controller telemetry
 *     HIL_CONTROL_TRACE (11070) Per-tick deterministic control replay input/output
 *     HEARTBEAT         (0)     1Hz Heartbeat
 *
 * Does not depend on pymavlink / mavlink C library, pure handwritten minimal implementation.
 * CRC: CRC-16/MCRF4XX + CRC_EXTRA (consistent with PX4).
 */
#include "mavlink_signing.h"
#include "hydrox/runtime/fixed_vector.h"
#include "hydrox/runtime/hil_session_config.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace hydrox
{

    // Message ID
    constexpr uint32_t MSGID_HEARTBEAT = 0;
    constexpr uint32_t MSGID_SYS_STATUS = 1;           // QGC health indicator
    constexpr uint32_t MSGID_ATTITUDE = 30;            // QGC telemetry
    constexpr uint32_t MSGID_LOCAL_POSITION_NED = 32;  // QGC telemetry
    constexpr uint32_t MSGID_GLOBAL_POSITION_INT = 33; // QGC map positioning
    constexpr uint32_t MSGID_VFR_HUD = 74;             // QGC telemetry
    constexpr uint32_t MSGID_COMMAND_LONG = 76;        // Standard command
    constexpr uint32_t MSGID_STATUSTEXT = 253;         // QGC notification bar
    constexpr uint32_t MSGID_HIL_SENSOR = 107;
    constexpr uint32_t MSGID_HIL_GPS = 113;
    constexpr uint32_t MSGID_HIL_STATE_QUAT = 115;
    constexpr uint32_t MSGID_HIL_ACTUATOR_CONTROLS = 93;
    constexpr uint32_t MSGID_HIL_DVL = 11060; // Custom
    constexpr uint32_t MSGID_HIL_TRUTH_STATE = 11061; // Custom debug truth
    constexpr uint32_t MSGID_HIL_PASSIVE_SONAR = 11062; // Custom OceanX sensor
    constexpr uint32_t MSGID_HIL_ACOUSTIC_NEIGHBORS = 11063; // Custom OceanX sensor
    constexpr uint32_t MSGID_HIL_RANGEFINDER_SCAN = 11064; // Custom OceanX sensor
    constexpr uint32_t MSGID_HIL_WHEEL_ODOMETRY = 11065; // Custom UGV wheel odometry
    constexpr uint32_t MSGID_HIL_PROFILE = 11066; // Profile selection handshake
    constexpr uint32_t MSGID_HIL_SESSION_CONFIG = 11067; // Per-run configuration
    constexpr uint32_t MSGID_HIL_SESSION_STATUS = 11068; // Configuration acknowledgement
    constexpr uint32_t MSGID_HIL_FC_STATE = 11069; // Estimated state telemetry
    constexpr uint32_t MSGID_HIL_CONTROL_TRACE = 11070; // Per-control-tick trace
    constexpr size_t MAVLINK_MAX_PAYLOAD_LEN = 255;
    constexpr size_t MAVLINK_MAX_PACKET_LEN =
        10 + MAVLINK_MAX_PAYLOAD_LEN + 2 + MAVLINK_SIGNATURE_BLOCK_LEN;
    constexpr size_t HIL_MAX_ACOUSTIC_CONTACTS = 4;
    constexpr size_t HIL_MAX_RANGEFINDER_RAYS = 48;

    /** In-place HIL_DVL (11060) schema. There is no legacy payload fallback. */
    constexpr size_t HIL_DVL_PAYLOAD_LEN = 26;
    constexpr size_t HIL_WHEEL_ODOMETRY_PAYLOAD_LEN = 33;
    constexpr size_t HIL_PROFILE_ID_LEN = 48;
    constexpr size_t HIL_PROFILE_PAYLOAD_LEN = 62;
    constexpr size_t HIL_SESSION_CONFIG_PAYLOAD_LEN =
        8 + 4 + runtime::kHilSessionDigestSize +
        runtime::kHilSessionConfigPayloadSize;
    constexpr size_t HIL_SESSION_STATUS_PAYLOAD_LEN =
        8 + 4 + runtime::kHilSessionDigestSize + 1 + 2 + 2;
    constexpr size_t HIL_FC_STATE_PAYLOAD_LEN = 255;
    constexpr size_t HIL_CONTROL_TRACE_PAYLOAD_LEN = 248;
    constexpr uint16_t HIL_FC_STATE_FLAG_DVL_VALID = 1u << 0;
    constexpr uint16_t HIL_FC_STATE_FLAG_EKF_INITIALIZED = 1u << 1;
    constexpr uint16_t HIL_FC_STATE_FLAG_ARMED = 1u << 2;
    constexpr uint16_t HIL_FC_STATE_FLAG_ACTUATOR_AUTHORIZED = 1u << 3;
    constexpr uint16_t HIL_FC_STATE_FLAG_EXTERNAL_SETPOINT = 1u << 4;
    constexpr uint16_t HIL_CONTROL_TRACE_FLAG_USE_YAW_RATE = 1u << 0;
    constexpr uint16_t HIL_CONTROL_TRACE_FLAG_USE_PATH_SEGMENT = 1u << 1;
    constexpr uint16_t HIL_CONTROL_TRACE_FLAG_HOLD_HEADING = 1u << 2;
    constexpr uint16_t HIL_CONTROL_TRACE_FLAG_CONTROLLER_RESET = 1u << 3;
    constexpr uint16_t HIL_CONTROL_TRACE_FLAG_ACTUATOR_AUTHORIZED = 1u << 4;
    constexpr uint16_t HIL_CONTROL_TRACE_FLAG_USED_TRUTH = 1u << 5;
    constexpr uint16_t HIL_CONTROL_TRACE_FLAG_EXTERNAL_SETPOINT = 1u << 6;
    constexpr uint16_t HIL_CONTROL_TRACE_FLAG_DVL_VALID = 1u << 7;
    constexpr uint16_t HIL_CONTROL_TRACE_FLAG_EKF_INITIALIZED = 1u << 8;
    constexpr uint8_t HIL_WHEEL_ODOMETRY_FLAG_VELOCITY_VALID = 1u << 0;
    constexpr uint8_t HIL_WHEEL_ODOMETRY_FLAG_WHEEL_SPEEDS_VALID = 1u << 1;
    constexpr uint8_t HIL_WHEEL_ODOMETRY_FLAG_YAW_RATE_VALID = 1u << 2;
    constexpr uint8_t HIL_DVL_FLAG_VELOCITY_VALID = 1u << 0;
    constexpr uint8_t HIL_DVL_FLAG_ALTITUDE_VALID = 1u << 1;

    enum class DvlTrackingMode : uint8_t
    {
        Unavailable = 0,
        BottomTrack = 1,
        WaterTrack = 2,
    };
    enum class HilProfileOperation : uint8_t
    {
        Select = 1,
        Ready = 2,
        Rejected = 3,
    };

    enum class HilSessionStatusOperation : uint8_t
    {
        Applying = 1,
        Ready = 2,
        Rejected = 3,
    };


    constexpr uint8_t MAV_STATE_STANDBY = 3;
    constexpr uint8_t MAV_STATE_ACTIVE = 4;
    constexpr uint16_t MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246;

    // Parsed message structures
    struct HeartbeatMsg
    {
        uint8_t type = 0;
        uint8_t autopilot = 0;
        uint8_t base_mode = 0;
        uint8_t system_status = 0;
        uint8_t mavlink_version = 0;
        bool valid = false;
    };

    struct CommandLongMsg
    {
        std::array<float, 7> params{};
        uint16_t command = 0;
        uint8_t target_system = 0;
        uint8_t target_component = 0;
        uint8_t confirmation = 0;
        bool valid = false;
    };

    struct HilSensorMsg
    {
        uint64_t time_usec = 0;
        float xacc = 0, yacc = 0, zacc = 0;    // Specific force (m/s²)
        float xgyro = 0, ygyro = 0, zgyro = 0; // Angular velocity (rad/s)
        float xmag = 0, ymag = 0, zmag = 0;    // Magnetic field (body frame, uT)
        float abs_pressure = 0;                // Absolute pressure (Pa)
        float diff_pressure = 0;               // OceanX internal HIL: Pa, not MAVLink hPa
        float pressure_alt = 0;                // Pressure altitude MSL (m)
        uint32_t fields_updated = 0;
    };

    struct HilGpsMsg
    {
        uint64_t time_usec = 0;
        uint8_t fix_type = 0;           // 0=None, 3=3D
        int32_t lat = 0;                // deg x 1e7
        int32_t lon = 0;                // deg x 1e7
        int32_t alt = 0;                // mm above mean sea level (MAVLink common.xml)
        uint16_t eph = 0xFFFFu;         // horizontal accuracy, cm; 0xFFFF unknown
        uint16_t epv = 0xFFFFu;         // vertical accuracy, cm; 0xFFFF unknown
        uint16_t vel = 0xFFFFu;         // ground speed, cm/s; 0xFFFF unknown
        int16_t vn = 0, ve = 0, vd = 0; // cm/s
        uint16_t cog = 0xFFFFu;         // course over ground, cdeg; 0xFFFF unknown
        uint8_t satellites_visible = 0;
    };

    struct HilDvlMsg
    {
        uint64_t time_usec = 0;
        float vx = 0, vy = 0, vz = 0; // Body frame velocity (m/s)
        float altitude_m = std::numeric_limits<float>::quiet_NaN();
        DvlTrackingMode tracking_mode = DvlTrackingMode::Unavailable;
        uint8_t flags = 0;

        bool is_bottom_track() const
        {
            return tracking_mode == DvlTrackingMode::BottomTrack;
        }

        bool is_water_track() const
        {
            return tracking_mode == DvlTrackingMode::WaterTrack;
        }

        bool velocity_valid() const
        {
            return (flags & HIL_DVL_FLAG_VELOCITY_VALID) != 0 &&
                   (is_bottom_track() || is_water_track()) &&
                   std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vz);
        }

        bool altitude_valid() const
        {
            return is_bottom_track() &&
                   (flags & HIL_DVL_FLAG_ALTITUDE_VALID) != 0 &&
                   std::isfinite(altitude_m);
        }
    };

    struct HilWheelOdometryMsg
    {
        uint64_t time_usec = 0;
        float right_radps = 0.0f;
        float left_radps = 0.0f;
        float forward_mps = 0.0f;
        float yaw_rate_radps = 0.0f;
        float velocity_variance = 0.0f;
        float yaw_rate_variance = 0.0f;
        uint8_t flags = 0;

        bool velocity_valid() const
        {
            return (flags & HIL_WHEEL_ODOMETRY_FLAG_VELOCITY_VALID) != 0 &&
                   std::isfinite(forward_mps) &&
                   std::isfinite(velocity_variance) && velocity_variance > 0.0f;
        }
        bool wheel_speeds_valid() const
        {
            return (flags & HIL_WHEEL_ODOMETRY_FLAG_WHEEL_SPEEDS_VALID) != 0 &&
                   std::isfinite(right_radps) && std::isfinite(left_radps);
        }
    };

    struct HilProfileMsg
    {
        uint64_t fingerprint = 0;
        uint32_t nonce = 0;
        HilProfileOperation operation = HilProfileOperation::Rejected;
        uint8_t mav_type = 0;
        std::array<char, HIL_PROFILE_ID_LEN> profile_id{};
        bool valid = false;
    };

    struct HilSessionConfigMsg
    {
        uint64_t profile_fingerprint = 0;
        uint32_t nonce = 0;
        runtime::HilSessionDigest digest{};
        runtime::HilSessionConfigV1 config{};
        bool digest_valid = false;
        bool valid = false;
    };

    struct HilSessionStatusMsg
    {
        uint64_t profile_fingerprint = 0;
        uint32_t nonce = 0;
        runtime::HilSessionDigest digest{};
        HilSessionStatusOperation operation =
            HilSessionStatusOperation::Rejected;
        runtime::HilSessionRejectReason reason =
            runtime::HilSessionRejectReason::None;
        runtime::HilSessionField field = runtime::HilSessionField::None;
        bool valid = false;
    };

    struct HilTruthStateMsg
    {
        uint64_t time_usec = 0;
        double eta[6] = {}; // [N, E, D, roll, pitch, yaw]
        double nu[6] = {};  // [u, v, w, p, q, r]
        bool valid = false;
    };

    /**
     * Flight-controller-owned state sent from FMU to the host HITL bridge.
     * This is estimator/controller telemetry, never simulator truth.
     */
    struct HilFcStateMsg
    {
        uint64_t time_usec = 0;
        double eta[6] = {};
        double nu[6] = {};
        double depth_m = 0.0;
        float normalized[8] = {};
        float dvl_vel[3] = {};
        float acc[3] = {};
        float gyro[3] = {};
        int32_t gps_lat = 0;
        int32_t gps_lon = 0;
        int32_t gps_alt = 0;
        float gps_vel_ned[3] = {};
        float motor_rpm_actual = 0.0f;
        float motor_thrust_N = 0.0f;
        float motor_power_W = 0.0f;
        float motor_current_A = 0.0f;
        float power_total_W = 0.0f;
        float energy_Wh = 0.0f;
        float battery_soc = 1.0f;
        float voltage_terminal = 0.0f;
        float runtime_remaining_s = 0.0f;
        float commanded_rpm = 0.0f;
        uint32_t tick = 0;
        uint16_t flags = 0;
        uint8_t mode = 0;
        uint8_t mission_state = 0;
        uint8_t actuator_channel_count = 0;
        uint8_t gps_fix = 0;
        uint8_t gps_satellites = 0;
        bool valid = false;
    };

    /**
     * One flight-controller control tick. Inputs stay double precision so an
     * offline controller/allocator replay does not accumulate telemetry
     * quantization error. Wrench is diagnostic float output; actuator output
     * remains the standard HIL_ACTUATOR_CONTROLS packet with the same time.
     */
    struct HilControlTraceMsg
    {
        uint64_t time_usec = 0;
        uint32_t tick = 0;
        double dt_s = 0.0;
        double eta[6] = {};
        double nu[6] = {};
        double depth_m = 0.0;
        double depth_ref = 0.0;
        double heading_ref = 0.0;
        double surge_ref = 0.0;
        double yaw_rate_ref = 0.0;
        double wp_n = 0.0;
        double wp_e = 0.0;
        double wp_d = 0.0;
        double path_start_n = 0.0;
        double path_start_e = 0.0;
        double lookahead_m = 0.0;
        double arrival_radius_m = 0.0;
        float wrench[6] = {};
        float setpoint_age_s = -1.0f;
        uint16_t flags = 0;
        uint8_t mode = 0;
        uint8_t mission_state = 0;
        uint8_t actuator_channel_count = 0;
        bool valid = false;
    };

    struct HilPassiveSonarBearingMsg
    {
        uint64_t time_usec = 0;
        bool valid = false;
        uint32_t target_slot = 0;
        float azimuth_rad = 0.0f;
        float elevation_rad = 0.0f;
    };

    struct HilAcousticContactMsg
    {
        uint32_t sender_id = 0;
        float range_m = 0.0f;
        float propagation_delay_s = 0.0f;
        float azimuth_rad = 0.0f;
        float elevation_rad = 0.0f;
        float depth_m = 0.0f;
        float position_ned[3] = {};
        float velocity_ned[3] = {};
        float yaw_ned_rad = 0.0f;
        float payload[3] = {};
        bool valid = false;
    };

    struct HilAcousticNeighborsMsg
    {
        uint64_t time_usec = 0;
        uint32_t receiver_id = 0;
        runtime::FixedVector<HilAcousticContactMsg,
                             HIL_MAX_ACOUSTIC_CONTACTS> contacts;
    };

    struct HilRangeFinderScanMsg
    {
        uint64_t time_usec = 0;
        uint32_t ray_count = 0;
        float max_range_m = 0.0f;
        runtime::FixedVector<float, HIL_MAX_RANGEFINDER_RAYS> ranges_m;
        bool valid = false;
    };

    struct HilActuatorControlsMsg
    {
        uint64_t time_usec = 0;
        std::array<float, 16> controls = {}; // Normalized [-1, 1]
        uint8_t mode = 0;
        uint64_t flags = 0;
        bool valid = false;
    };

    // Parsed frame (generic, contains msg_id + payload)
    struct MavFrame
    {
        uint32_t msg_id = 0;
        uint8_t sysid = 0;
        uint8_t compid = 0;
        uint8_t mavlink_version = 0;
        bool signed_frame = false;
        runtime::FixedVector<uint8_t, MAVLINK_MAX_PAYLOAD_LEN> payload;
    };

    struct MavlinkPacket
    {
        std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
        size_t length = 0;

        const uint8_t *data() const noexcept { return bytes.data(); }
        uint8_t *data() noexcept { return bytes.data(); }
        size_t size() const noexcept { return length; }
        bool empty() const noexcept { return length == 0; }
    };

    // Encoder and Decoder
    class MavlinkHIL
    {
    public:
        explicit MavlinkHIL(
            uint8_t sysid = 1,
            uint8_t compid = 1,
            const MavlinkSigningConfig &signing = {});

        // Receive: byte stream -> frame list
        /** Feed bytes, returns completely parsed frames (could be empty). */
        std::vector<MavFrame> feed(const uint8_t *data, size_t len);

        using FrameVisitor = void (*)(void *context, const MavFrame &frame);
        /** Heap-free receive API for embedded supervisors. */
        void feed_each(const uint8_t *data, size_t len,
                       void *context, FrameVisitor visitor);

        /** Parse various messages from frame payload */
        HeartbeatMsg parse_heartbeat(const MavFrame &f) const;
        CommandLongMsg parse_command_long(const MavFrame &f) const;
        HilSensorMsg parse_hil_sensor(const MavFrame &f) const;
        HilGpsMsg parse_hil_gps(const MavFrame &f) const;
        HilDvlMsg parse_hil_dvl(const MavFrame &f) const;
        HilWheelOdometryMsg parse_hil_wheel_odometry(const MavFrame &f) const;
        HilTruthStateMsg parse_hil_truth_state(const MavFrame &f) const;
        HilFcStateMsg parse_hil_fc_state(const MavFrame &f) const;
        HilControlTraceMsg parse_hil_control_trace(const MavFrame &f) const;
        HilActuatorControlsMsg parse_hil_actuator_controls(const MavFrame &f) const;
        HilPassiveSonarBearingMsg parse_hil_passive_sonar(const MavFrame &f) const;
        HilAcousticNeighborsMsg parse_hil_acoustic_neighbors(const MavFrame &f) const;
        HilRangeFinderScanMsg parse_hil_rangefinder_scan(const MavFrame &f) const;
        HilProfileMsg parse_hil_profile(const MavFrame &f) const;
        HilSessionConfigMsg parse_hil_session_config(const MavFrame &f) const;
        HilSessionStatusMsg parse_hil_session_status(const MavFrame &f) const;

        // Send: message -> byte frame
        /** HIL_ACTUATOR_CONTROLS (93) */
        std::vector<uint8_t> encode_hil_actuator_controls(
            const std::array<float, 8> &controls,
            uint64_t time_usec = 0,
            uint8_t mode = 0,
            uint64_t flags = 0) const;
        bool encode_hil_actuator_controls(
            MavlinkPacket &packet,
            const std::array<float, 8> &controls,
            uint64_t time_usec = 0,
            uint8_t mode = 0,
            uint64_t flags = 0) const;

        /** HEARTBEAT (0) */
        std::vector<uint8_t> encode_heartbeat(uint8_t mav_type = 12) const;
        /** HIL_PROFILE (11066) */
        bool encode_hil_profile(
            MavlinkPacket &packet, const HilProfileMsg &message) const;
        /** HIL_SESSION_CONFIG (11067) */
        bool encode_hil_session_config(
            MavlinkPacket &packet, const HilSessionConfigMsg &message) const;
        /** HIL_SESSION_STATUS (11068) */
        bool encode_hil_session_status(
            MavlinkPacket &packet, const HilSessionStatusMsg &message) const;
        /** HIL_FC_STATE (11069) */
        bool encode_hil_fc_state(
            MavlinkPacket &packet, const HilFcStateMsg &message) const;
        /** HIL_CONTROL_TRACE (11070) */
        bool encode_hil_control_trace(
            MavlinkPacket &packet, const HilControlTraceMsg &message) const;

        bool encode_heartbeat(MavlinkPacket &packet,
                              uint8_t mav_type = 12) const;

    private:
        uint8_t _sysid;
        uint8_t _compid;
        mutable uint8_t _seq = 0;
        MavlinkSigningConfig _signing;
        mutable uint64_t _tx_signing_timestamp = 0;
        struct ReplayStream
        {
            uint32_t id = 0;
            uint64_t timestamp = 0;
            bool used = false;
        };
        std::array<ReplayStream, 8> _last_rx_timestamps{};

        runtime::FixedVector<uint8_t, MAVLINK_MAX_PACKET_LEN> _buf;

        std::optional<MavFrame> _parse_one();
        std::vector<uint8_t> _encode_frame(uint32_t msg_id,
                                           const uint8_t *payload,
                                           size_t payload_len) const;
        bool _encode_frame(MavlinkPacket &packet,
                           uint32_t msg_id,
                           const uint8_t *payload,
                           size_t payload_len) const;
        bool _validate_signed_frame(
            const uint8_t *packet,
            size_t packet_len_without_signature,
            uint8_t sysid,
            uint8_t compid,
            const uint8_t *signature);

        static uint16_t _crc16(const uint8_t *data, size_t len,
                               uint16_t crc = 0xFFFF);
        static uint8_t _crc_extra(uint32_t msg_id);
    };

    // Utility

    /** Absolute pressure (Pa) -> depth (m, positive downwards) */
    inline double pressure_to_depth(float abs_pressure_pa)
    {
        constexpr double rho = 1028.0;
        constexpr double g = 9.80665;
        constexpr double p0 = 101325.0;
        double d = (static_cast<double>(abs_pressure_pa) - p0) / (rho * g);
        return d > 0.0 ? d : 0.0;
    }

} // namespace hydrox
