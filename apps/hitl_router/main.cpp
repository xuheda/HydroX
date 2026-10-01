#include "hydrox/platform/host/host_clock.h"
#include "hydrox/platform/host/host_serial_byte_stream.h"
#include "hydrox/platform/host/host_sleeper.h"
#include "hydrox/runtime/fixed_frame_sender.h"
#include "hydrox/runtime/mavlink_deframer.h"
#include "hitl/hitl_xlog.h"
#include "mavlink_hil.h"
#include "sitl/sitl_platform.h"
#include "tcp_transport.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>

namespace
{
    std::atomic<bool> running{true};

    struct Config
    {
        std::string ue_host = "127.0.0.1";
        uint16_t ue_port = 14600;
        std::string serial_device;
        uint32_t baud_rate = 921600;
        std::string label = "vehicle";
        uint32_t status_interval_ms = 250;
        std::string status_file;
        std::string profile_id;
        uint64_t profile_fingerprint = 0;
        std::string xlog = "auto";
        std::string log_directory;
        std::string run_id;
        std::string stop_file;
        std::string truth_udp_host = "127.0.0.1";
        uint16_t truth_udp_port = 0;
        std::string state_udp_host = "127.0.0.1";
        uint16_t state_udp_port = 0;
        uint32_t parent_pid = 0;
        hydrox::runtime::HilSessionConfigValues session_values{};
        hydrox::runtime::HilSessionConfigV1 session_config{};
        hydrox::runtime::HilSessionDigest session_digest{};
        uint32_t session_fields_seen = 0;
    };

    enum SessionOptionBit : uint32_t
    {
        SessionVersion = 1u << 0,
        SessionPeriod = 1u << 1,
        SessionMaxSensorDt = 1u << 2,
        SessionSensorTimeout = 1u << 3,
        SessionSetpointTimeout = 1u << 4,
        SessionInitialN = 1u << 5,
        SessionInitialE = 1u << 6,
        SessionInitialDown = 1u << 7,
        SessionInitialHeading = 1u << 8,
        SessionInitialSurge = 1u << 9,
        SessionGpsLat = 1u << 10,
        SessionGpsLon = 1u << 11,
        SessionGpsAlt = 1u << 12,
        SessionGpsRadius = 1u << 13,
        SessionMissionRadius = 1u << 14,
        SessionRequiredSensors = 1u << 15,
        SessionAccelMode = 1u << 16,
        SessionFeedbackSource = 1u << 17,
        SessionTruthHeadingAid = 1u << 18,
    };
    constexpr uint32_t kRequiredSessionFields =
        (1u << 19) - 1u;

    void signal_handler(int)
    {
        running = false;
    }

    bool parse_unsigned(const char *text, uint64_t maximum, uint64_t &value)
    {
        if (text == nullptr || *text == '\0')
            return false;
        char *end = nullptr;
        const unsigned long long parsed = std::strtoull(text, &end, 10);
        if (end == text || *end != '\0' || parsed > maximum)
            return false;
        value = static_cast<uint64_t>(parsed);
        return true;
    }

    bool parse_fingerprint(const char *text, uint64_t &fingerprint)
    {
        if (text == nullptr || std::strlen(text) != 16)
            return false;
        for (const char *cursor = text; *cursor != '\0'; ++cursor)
        {
            const bool decimal = *cursor >= '0' && *cursor <= '9';
            const bool lower = *cursor >= 'a' && *cursor <= 'f';
            const bool upper = *cursor >= 'A' && *cursor <= 'F';
            if (!decimal && !lower && !upper)
                return false;
        }
        char *end = nullptr;
        const unsigned long long parsed = std::strtoull(text, &end, 16);
        if (end == text || *end != '\0' || parsed == 0)
            return false;
        fingerprint = static_cast<uint64_t>(parsed);
        return true;
    }

    bool parse_finite_double(const char *text, double &value)
    {
        if (text == nullptr || *text == '\0')
            return false;
        char *end = nullptr;
        const double parsed = std::strtod(text, &end);
        if (end == text || *end != '\0' || !std::isfinite(parsed))
            return false;
        value = parsed;
        return true;
    }

    bool parse_bool(const char *text, bool &value)
    {
        if (text == nullptr)
            return false;
        if (std::strcmp(text, "true") == 0 ||
            std::strcmp(text, "1") == 0)
        {
            value = true;
            return true;
        }
        if (std::strcmp(text, "false") == 0 ||
            std::strcmp(text, "0") == 0)
        {
            value = false;
            return true;
        }
        return false;
    }

    bool parse_config(int argc, char *argv[], Config &config)
    {
        for (int i = 1; i < argc; ++i)
        {
            const char *key = argv[i];
            if (std::strcmp(key, "--help") == 0)
                return false;
            if (i + 1 >= argc)
                return false;
            const char *value = argv[++i];
            if (std::strcmp(key, "--ue-host") == 0)
                config.ue_host = value;
            else if (std::strcmp(key, "--serial") == 0)
                config.serial_device = value;
            else if (std::strcmp(key, "--label") == 0)
                config.label = value;
            else if (std::strcmp(key, "--status-file") == 0)
                config.status_file = value;
            else if (std::strcmp(key, "--profile-id") == 0)
                config.profile_id = value;
            else if (std::strcmp(key, "--xlog") == 0)
                config.xlog = value;
            else if (std::strcmp(key, "--log-directory") == 0)
                config.log_directory = value;
            else if (std::strcmp(key, "--run-id") == 0)
                config.run_id = value;
            else if (std::strcmp(key, "--stop-file") == 0)
                config.stop_file = value;
            else if (std::strcmp(key, "--truth-udp-host") == 0)
                config.truth_udp_host = value;
            else if (std::strcmp(key, "--state-udp-host") == 0)
                config.state_udp_host = value;
            else if (std::strcmp(key, "--parent-pid") == 0)
            {
                uint64_t parsed = 0;
                if (!parse_unsigned(value, UINT32_MAX, parsed))
                    return false;
                config.parent_pid = static_cast<uint32_t>(parsed);
            }
            else if (std::strcmp(key, "--profile-fingerprint") == 0)
            {
                if (!parse_fingerprint(value, config.profile_fingerprint))
                    return false;
            }
            else if (std::strcmp(key, "--session-version") == 0)
            {
                uint64_t parsed = 0;
                if (!parse_unsigned(value, 65535, parsed) ||
                    parsed != hydrox::runtime::kHilSessionConfigVersion)
                    return false;
                config.session_fields_seen |= SessionVersion;
            }
            else if (std::strcmp(key, "--session-period-us") == 0)
            {
                uint64_t parsed = 0;
                if (!parse_unsigned(value, UINT32_MAX, parsed))
                    return false;
                config.session_values.nominal_period_us =
                    static_cast<uint32_t>(parsed);
                config.session_fields_seen |= SessionPeriod;
            }
            else if (std::strcmp(key, "--session-max-sensor-dt-us") == 0)
            {
                uint64_t parsed = 0;
                if (!parse_unsigned(value, UINT32_MAX, parsed))
                    return false;
                config.session_values.max_sensor_dt_us =
                    static_cast<uint32_t>(parsed);
                config.session_fields_seen |= SessionMaxSensorDt;
            }
            else if (std::strcmp(key, "--session-sensor-timeout-us") == 0)
            {
                uint64_t parsed = 0;
                if (!parse_unsigned(value, UINT32_MAX, parsed))
                    return false;
                config.session_values.sensor_timeout_us =
                    static_cast<uint32_t>(parsed);
                config.session_fields_seen |= SessionSensorTimeout;
            }
            else if (std::strcmp(key, "--session-setpoint-timeout-us") == 0)
            {
                uint64_t parsed = 0;
                if (!parse_unsigned(value, UINT32_MAX, parsed))
                    return false;
                config.session_values.setpoint_timeout_us =
                    static_cast<uint32_t>(parsed);
                config.session_fields_seen |= SessionSetpointTimeout;
            }
            else if (std::strcmp(key, "--session-initial-n-m") == 0)
            {
                if (!parse_finite_double(
                        value, config.session_values.initial_n_m))
                    return false;
                config.session_fields_seen |= SessionInitialN;
            }
            else if (std::strcmp(key, "--session-initial-e-m") == 0)
            {
                if (!parse_finite_double(
                        value, config.session_values.initial_e_m))
                    return false;
                config.session_fields_seen |= SessionInitialE;
            }
            else if (std::strcmp(key, "--session-initial-down-m") == 0)
            {
                if (!parse_finite_double(
                        value, config.session_values.initial_down_m))
                    return false;
                config.session_fields_seen |= SessionInitialDown;
            }
            else if (std::strcmp(key, "--session-initial-heading-rad") == 0)
            {
                if (!parse_finite_double(
                        value, config.session_values.initial_heading_rad))
                    return false;
                config.session_fields_seen |= SessionInitialHeading;
            }
            else if (std::strcmp(key, "--session-initial-surge-mps") == 0)
            {
                if (!parse_finite_double(
                        value, config.session_values.initial_surge_mps))
                    return false;
                config.session_fields_seen |= SessionInitialSurge;
            }
            else if (std::strcmp(key, "--session-gps-lat-deg") == 0)
            {
                if (!parse_finite_double(
                        value, config.session_values.gps_origin_lat_deg))
                    return false;
                config.session_fields_seen |= SessionGpsLat;
            }
            else if (std::strcmp(key, "--session-gps-lon-deg") == 0)
            {
                if (!parse_finite_double(
                        value, config.session_values.gps_origin_lon_deg))
                    return false;
                config.session_fields_seen |= SessionGpsLon;
            }
            else if (std::strcmp(key, "--session-gps-alt-msl-m") == 0)
            {
                if (!parse_finite_double(
                        value,
                        config.session_values.gps_origin_altitude_msl_m))
                    return false;
                config.session_fields_seen |= SessionGpsAlt;
            }
            else if (std::strcmp(key, "--session-gps-max-radius-m") == 0)
            {
                if (!parse_finite_double(
                        value, config.session_values.gps_max_radius_m))
                    return false;
                config.session_fields_seen |= SessionGpsRadius;
            }
            else if (std::strcmp(key, "--session-mission-radius-m") == 0)
            {
                if (!parse_finite_double(
                        value, config.session_values.mission_radius_m))
                    return false;
                config.session_fields_seen |= SessionMissionRadius;
            }
            else if (std::strcmp(key, "--session-required-sensor-mask") == 0)
            {
                uint64_t parsed = 0;
                if (!parse_unsigned(value, UINT32_MAX, parsed))
                    return false;
                config.session_values.required_sensor_mask =
                    static_cast<uint32_t>(parsed);
                config.session_fields_seen |= SessionRequiredSensors;
            }
            else if (std::strcmp(key, "--session-accel-mode") == 0)
            {
                if (std::strcmp(value, "off") == 0)
                    config.session_values.accel_mode =
                        hydrox::runtime::HilSessionAccelMode::Off;
                else if (std::strcmp(value, "auto") == 0)
                    config.session_values.accel_mode =
                        hydrox::runtime::HilSessionAccelMode::Auto;
                else if (std::strcmp(value, "on") == 0)
                    config.session_values.accel_mode =
                        hydrox::runtime::HilSessionAccelMode::On;
                else
                    return false;
                config.session_fields_seen |= SessionAccelMode;
            }
            else if (std::strcmp(key, "--session-feedback-source") == 0)
            {
                if (std::strcmp(value, "estimated_state") == 0)
                    config.session_values.feedback_source =
                        hydrox::runtime::HilSessionFeedbackSource::EstimatedState;
                else if (std::strcmp(value, "truth_debug") == 0)
                    config.session_values.feedback_source =
                        hydrox::runtime::HilSessionFeedbackSource::TruthDebug;
                else
                    return false;
                config.session_fields_seen |= SessionFeedbackSource;
            }
            else if (std::strcmp(key, "--session-truth-heading-aid") == 0)
            {
                if (!parse_bool(
                        value,
                        config.session_values.allow_truth_heading_aid))
                    return false;
                config.session_fields_seen |= SessionTruthHeadingAid;
            }
            else if (std::strcmp(key, "--ue-port") == 0)
            {
                uint64_t parsed = 0;
                if (!parse_unsigned(value, 65535, parsed) || parsed == 0)
                    return false;
                config.ue_port = static_cast<uint16_t>(parsed);
            }
            else if (std::strcmp(key, "--truth-udp-port") == 0)
            {
                uint64_t parsed = 0;
                if (!parse_unsigned(value, 65535, parsed) || parsed == 0)
                    return false;
                config.truth_udp_port = static_cast<uint16_t>(parsed);
            }
            else if (std::strcmp(key, "--state-udp-port") == 0)
            {
                uint64_t parsed = 0;
                if (!parse_unsigned(value, 65535, parsed) || parsed == 0)
                    return false;
                config.state_udp_port = static_cast<uint16_t>(parsed);
            }
            else if (std::strcmp(key, "--baud") == 0)
            {
                uint64_t parsed = 0;
                if (!parse_unsigned(value, 4'000'000, parsed) || parsed == 0)
                    return false;
                config.baud_rate = static_cast<uint32_t>(parsed);
            }
            else if (std::strcmp(key, "--status-interval-ms") == 0)
            {
                uint64_t parsed = 0;
                if (!parse_unsigned(value, 600'000, parsed))
                    return false;
                config.status_interval_ms = static_cast<uint32_t>(parsed);
            }
            else
            {
                return false;
            }
        }
        hydrox::runtime::HilSessionField field =
            hydrox::runtime::HilSessionField::None;
        const bool session_valid =
            config.session_fields_seen == kRequiredSessionFields &&
            hydrox::runtime::resolve_hil_session_config(
                config.session_values, config.session_config, field);
        if (session_valid)
            config.session_digest = hydrox::runtime::hil_session_digest(
                config.profile_fingerprint, config.session_config);
        return config.xlog != "off" && config.xlog != "OFF" && config.xlog != "0" &&
               !config.xlog.empty() && !config.serial_device.empty() && !config.label.empty() &&
               !config.profile_id.empty() &&
               config.profile_id.size() < hydrox::HIL_PROFILE_ID_LEN &&
               config.profile_fingerprint != 0 && session_valid;
    }

    void usage(const char *program)
    {
        std::fprintf(
            stderr,
            "Usage: %s --serial <COM7|/dev/ttyACM0> "
            "[--baud 921600] [--ue-host 127.0.0.1] [--ue-port 14600] "
            "[--label vehicle] [--status-interval-ms 250] "
            "[--parent-pid <pid>] "
            "[--status-file <path.json>] "
            "[--xlog <off|auto|path.xlog>] "
            "[--log-directory <path>] [--run-id <id>] "
            "[--stop-file <path.request>] "
            "[--truth-udp-host 127.0.0.1] [--truth-udp-port 14610] "
            "[--state-udp-host 127.0.0.1] [--state-udp-port 14700] "
            "--profile-id <exact-id> "
            "--profile-fingerprint <16-hex-digits> "
            "--session-version 1 --session-period-us <us> "
            "--session-max-sensor-dt-us <us> "
            "--session-sensor-timeout-us <us> "
            "--session-setpoint-timeout-us <us> "
            "--session-initial-n-m <m> --session-initial-e-m <m> "
            "--session-initial-down-m <m> "
            "--session-initial-heading-rad <rad> "
            "--session-initial-surge-mps <m/s> "
            "--session-gps-lat-deg <deg> --session-gps-lon-deg <deg> "
            "--session-gps-alt-msl-m <m> "
            "--session-gps-max-radius-m <m> "
            "--session-mission-radius-m <m> "
            "--session-required-sensor-mask <uint32> "
            "--session-accel-mode <off|auto|on> "
            "--session-feedback-source <estimated_state|truth_debug> "
            "--session-truth-heading-aid <true|false>\n",
            program != nullptr ? program : "hydrox_hitl_router");
    }

    bool stop_requested(const Config &config)
    {
        if (!running)
            return true;
        if (config.stop_file.empty())
            return false;
        std::error_code error;
        return std::filesystem::exists(config.stop_file, error) && !error;
    }

    std::string json_escape(const std::string &value)
    {
        std::string escaped;
        escaped.reserve(value.size() + 8);
        for (const unsigned char ch : value)
        {
            switch (ch)
            {
            case '\\': escaped += "\\\\"; break;
            case '"': escaped += "\\\""; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default:
                if (ch >= 0x20)
                    escaped.push_back(static_cast<char>(ch));
                break;
            }
        }
        return escaped;
    }

    int64_t unix_time_ms()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    bool write_status(
        const Config &config,
        const char *state,
        bool serial_open,
        bool ue_connected,
        uint64_t generation,
        uint64_t ue_to_board_frames,
        uint64_t ue_to_board_bytes,
        uint64_t board_to_ue_frames,
        uint64_t board_to_ue_bytes)
    {
        char fingerprint[17] = {};
        std::snprintf(
            fingerprint, sizeof(fingerprint), "%016llX",
            static_cast<unsigned long long>(config.profile_fingerprint));
        char session_digest[65] = {};
        for (std::size_t index = 0;
             index < config.session_digest.size();
             ++index)
        {
            std::snprintf(
                session_digest + index * 2,
                sizeof(session_digest) - index * 2,
                "%02x",
                static_cast<unsigned int>(config.session_digest[index]));
        }
        const bool profile_ready =
            state != nullptr &&
            (std::strcmp(state, "active") == 0 ||
             std::strcmp(state, "waiting_for_ue") == 0 ||
             std::strcmp(state, "applying_session") == 0);
        const bool session_ready =
            state != nullptr &&
            (std::strcmp(state, "active") == 0 ||
             std::strcmp(state, "waiting_for_ue") == 0);

        std::ostringstream json;
        json << "{\"schema_version\":2"
             << ",\"label\":\"" << json_escape(config.label) << "\""
             << ",\"serial_device\":\"" << json_escape(config.serial_device) << "\""
             << ",\"baud_rate\":" << config.baud_rate
             << ",\"ue_host\":\"" << json_escape(config.ue_host) << "\""
             << ",\"ue_port\":" << config.ue_port
             << ",\"profile_id\":\"" << json_escape(config.profile_id) << "\""
             << ",\"profile_fingerprint\":\"" << fingerprint << "\""
             << ",\"profile_ready\":" << (profile_ready ? "true" : "false")
             << ",\"session_config_version\":" << config.session_config.schema_version
             << ",\"session_config_sha256\":\"" << session_digest << "\""
             << ",\"session_ready\":" << (session_ready ? "true" : "false")
             << ",\"state\":\"" << json_escape(state != nullptr ? state : "unknown") << "\""
             << ",\"serial_open\":" << (serial_open ? "true" : "false")
             << ",\"ue_connected\":" << (ue_connected ? "true" : "false")
             << ",\"generation\":" << generation
             << ",\"ue_to_board_frames\":" << ue_to_board_frames
             << ",\"ue_to_board_bytes\":" << ue_to_board_bytes
             << ",\"board_to_ue_frames\":" << board_to_ue_frames
             << ",\"board_to_ue_bytes\":" << board_to_ue_bytes
             << ",\"updated_unix_ms\":" << unix_time_ms()
             << "}";
        const std::string payload = json.str();

        // This framed stdout line is the live IPC channel consumed by OceanX.
        std::printf("@@OCEANX_STATUS %s\n", payload.c_str());
        std::fflush(stdout);
        if (config.status_file.empty())
            return true;

        const std::string temporary = config.status_file + ".tmp";
        std::ofstream output(temporary, std::ios::out | std::ios::trunc);
        if (!output.is_open())
            return false;
        output << payload << "\n";
        output.close();
        if (!output)
            return false;

        return hydrox::sitl::replace_file_atomically(
            temporary, config.status_file);
    }

    std::string safe_file_component(std::string value)
    {
        for (char &character : value)
        {
            const bool safe =
                (character >= 'A' && character <= 'Z') ||
                (character >= 'a' && character <= 'z') ||
                (character >= '0' && character <= '9') ||
                character == '_' || character == '-';
            if (!safe)
                character = '_';
        }
        return value.empty() ? "vehicle" : value;
    }

    enum class ProfileHandshakeResult : uint8_t
    {
        Ready = 0,
        Rejected,
        TimedOut,
        IoError,
        Interrupted,
    };

    uint32_t next_profile_nonce()
    {
        static std::atomic<uint32_t> counter{1};
        const uint64_t ticks = static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count());
        uint32_t nonce = static_cast<uint32_t>(ticks ^ (ticks >> 32)) ^
                         counter.fetch_add(1, std::memory_order_relaxed);
        return nonce != 0 ? nonce : 1;
    }

    ProfileHandshakeResult negotiate_profile(
        const Config &config,
        hydrox::platform::ByteStream &serial,
        hydrox::platform::Clock &clock,
        hydrox::platform::Sleeper &sleeper,
        uint32_t nonce)
    {
        constexpr uint64_t kHandshakeTimeoutUs = 5'000'000;
        constexpr uint64_t kRequestIntervalUs = 250'000;
        hydrox::HilProfileMsg selection;
        selection.fingerprint = config.profile_fingerprint;
        selection.nonce = nonce;
        selection.operation = hydrox::HilProfileOperation::Select;
        std::memcpy(
            selection.profile_id.data(),
            config.profile_id.c_str(),
            config.profile_id.size() + 1);
        selection.valid = true;

        hydrox::MavlinkHIL codec(255, 190);
        hydrox::MavlinkPacket request;
        if (!codec.encode_hil_profile(request, selection))
            return ProfileHandshakeResult::IoError;

        hydrox::runtime::FixedFrameSender sender;
        std::array<uint8_t, 512> bytes{};
        const uint64_t started_at = clock.now_us();
        const uint64_t deadline = started_at + kHandshakeTimeoutUs;
        uint64_t next_request_us = started_at;
        while (!stop_requested(config) && serial.is_open() &&
               clock.now_us() < deadline)
        {
            const uint64_t now_us = clock.now_us();
            const auto flush_status = sender.flush(serial, now_us);
            if (flush_status == hydrox::runtime::FixedFrameSendStatus::FATAL ||
                flush_status == hydrox::runtime::FixedFrameSendStatus::TIMED_OUT)
                return ProfileHandshakeResult::IoError;

            if (now_us >= next_request_us && sender.pending_bytes() == 0)
            {
                const auto send_status = sender.write_frame(
                    serial, request.data(), request.size(), now_us);
                if (send_status == hydrox::runtime::FixedFrameSendStatus::FATAL ||
                    send_status == hydrox::runtime::FixedFrameSendStatus::TIMED_OUT ||
                    send_status == hydrox::runtime::FixedFrameSendStatus::OVERSIZE)
                    return ProfileHandshakeResult::IoError;
                next_request_us = now_us + kRequestIntervalUs;
            }

            const auto read = serial.read(bytes.data(), bytes.size());
            if (read.status == hydrox::platform::IoStatus::Ok)
            {
                if (read.size == 0 || read.size > bytes.size())
                    return ProfileHandshakeResult::IoError;
                const auto frames = codec.feed(bytes.data(), read.size);
                for (const auto &frame : frames)
                {
                    if (frame.msg_id != hydrox::MSGID_HIL_PROFILE)
                        continue;
                    const auto response = codec.parse_hil_profile(frame);
                    if (!response.valid || response.nonce != nonce ||
                        response.fingerprint != config.profile_fingerprint ||
                        std::strcmp(response.profile_id.data(),
                                    config.profile_id.c_str()) != 0)
                        continue;
                    if (response.operation == hydrox::HilProfileOperation::Ready)
                        return ProfileHandshakeResult::Ready;
                    if (response.operation == hydrox::HilProfileOperation::Rejected)
                        return ProfileHandshakeResult::Rejected;
                }
            }
            else if (read.status != hydrox::platform::IoStatus::WouldBlock)
            {
                return ProfileHandshakeResult::IoError;
            }
            sleeper.sleep_for_us(1'000);
        }
        if (stop_requested(config))
            return ProfileHandshakeResult::Interrupted;
        return serial.is_open() ? ProfileHandshakeResult::TimedOut
                                : ProfileHandshakeResult::IoError;
    }

    enum class SessionHandshakeResult : uint8_t
    {
        Ready = 0,
        Rejected,
        TimedOut,
        IoError,
        Interrupted,
    };

    SessionHandshakeResult negotiate_session(
        const Config &config,
        hydrox::platform::ByteStream &serial,
        hydrox::platform::Clock &clock,
        hydrox::platform::Sleeper &sleeper,
        uint32_t nonce,
        hydrox::runtime::HilSessionRejectReason &reject_reason,
        hydrox::runtime::HilSessionField &reject_field)
    {
        constexpr uint64_t kHandshakeTimeoutUs = 5'000'000;
        constexpr uint64_t kRequestIntervalUs = 250'000;
        reject_reason = hydrox::runtime::HilSessionRejectReason::None;
        reject_field = hydrox::runtime::HilSessionField::None;

        hydrox::HilSessionConfigMsg selection;
        selection.profile_fingerprint = config.profile_fingerprint;
        selection.nonce = nonce;
        selection.digest = config.session_digest;
        selection.config = config.session_config;
        selection.digest_valid = true;
        selection.valid = true;

        hydrox::MavlinkHIL codec(255, 190);
        hydrox::MavlinkPacket request;
        if (!codec.encode_hil_session_config(request, selection))
            return SessionHandshakeResult::IoError;

        hydrox::runtime::FixedFrameSender sender;
        std::array<uint8_t, 512> bytes{};
        const uint64_t started_at = clock.now_us();
        const uint64_t deadline = started_at + kHandshakeTimeoutUs;
        uint64_t next_request_us = started_at;
        while (!stop_requested(config) && serial.is_open() &&
               clock.now_us() < deadline)
        {
            const uint64_t now_us = clock.now_us();
            const auto flush_status = sender.flush(serial, now_us);
            if (flush_status == hydrox::runtime::FixedFrameSendStatus::FATAL ||
                flush_status ==
                    hydrox::runtime::FixedFrameSendStatus::TIMED_OUT)
                return SessionHandshakeResult::IoError;

            if (now_us >= next_request_us && sender.pending_bytes() == 0)
            {
                const auto send_status = sender.write_frame(
                    serial, request.data(), request.size(), now_us);
                if (send_status ==
                        hydrox::runtime::FixedFrameSendStatus::FATAL ||
                    send_status ==
                        hydrox::runtime::FixedFrameSendStatus::TIMED_OUT ||
                    send_status ==
                        hydrox::runtime::FixedFrameSendStatus::OVERSIZE)
                    return SessionHandshakeResult::IoError;
                next_request_us = now_us + kRequestIntervalUs;
            }

            const auto read = serial.read(bytes.data(), bytes.size());
            if (read.status == hydrox::platform::IoStatus::Ok)
            {
                if (read.size == 0 || read.size > bytes.size())
                    return SessionHandshakeResult::IoError;
                const auto frames = codec.feed(bytes.data(), read.size);
                for (const auto &frame : frames)
                {
                    if (frame.msg_id != hydrox::MSGID_HIL_SESSION_STATUS)
                        continue;
                    const auto response =
                        codec.parse_hil_session_status(frame);
                    if (!response.valid || response.nonce != nonce ||
                        response.profile_fingerprint !=
                            config.profile_fingerprint ||
                        !hydrox::runtime::hil_session_digest_equal(
                            response.digest, config.session_digest))
                        continue;
                    if (response.operation ==
                        hydrox::HilSessionStatusOperation::Ready)
                        return SessionHandshakeResult::Ready;
                    if (response.operation ==
                        hydrox::HilSessionStatusOperation::Rejected)
                    {
                        reject_reason = response.reason;
                        reject_field = response.field;
                        return SessionHandshakeResult::Rejected;
                    }
                }
            }
            else if (read.status != hydrox::platform::IoStatus::WouldBlock)
            {
                return SessionHandshakeResult::IoError;
            }
            sleeper.sleep_for_us(1'000);
        }
        if (stop_requested(config))
            return SessionHandshakeResult::Interrupted;
        return serial.is_open() ? SessionHandshakeResult::TimedOut
                                : SessionHandshakeResult::IoError;
    }
}

int main(int argc, char *argv[])
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::setvbuf(stderr, nullptr, _IONBF, 0);

    Config config;
    if (!parse_config(argc, argv, config))
    {
        usage(argc > 0 ? argv[0] : nullptr);
        return 2;
    }

    hydrox::sitl::ParentProcessGuard parent_guard(config.parent_pid);
    if (!parent_guard.arm())
    {
        std::fprintf(stderr, "Failed to monitor parent PID %u\n",
                     static_cast<unsigned>(config.parent_pid));
        return 5;
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    hydrox::platform::host::HostClock clock;
    hydrox::platform::host::HostSleeper sleeper;
    hydrox::platform::host::HostSerialByteStream serial(
        config.serial_device, config.baud_rate);
    hydrox::TcpTransport ue(config.ue_host, config.ue_port, true);
    hydrox::hitl::XLogRecorder xlog_recorder;
    bool capture_failed = false;
    std::array<uint8_t, 1024> buffer{};
    hydrox::sitl::NetworkRuntime network;
    std::unique_ptr<hydrox::sitl::UdpSender> truth_mirror;
    std::unique_ptr<hydrox::sitl::UdpSender> state_mirror;
    if (config.truth_udp_port != 0)
    {
        if (!network.ready())
        {
            std::fprintf(stderr, "Failed to initialize UDP network runtime\n");
            return 4;
        }
        truth_mirror = std::make_unique<hydrox::sitl::UdpSender>(
            config.truth_udp_host, config.truth_udp_port, false);
        if (!truth_mirror->is_open())
        {
            std::fprintf(stderr, "Failed to open HIL truth UDP mirror\n");
            return 4;
        }
    }

    if (config.state_udp_port != 0)
    {
        if (!network.ready())
        {
            std::fprintf(stderr, "Failed to initialize UDP network runtime\n");
            return 4;
        }
        state_mirror = std::make_unique<hydrox::sitl::UdpSender>(
            config.state_udp_host, config.state_udp_port, false);
        if (!state_mirror->is_open())
        {
            std::fprintf(stderr, "Failed to open FC state UDP mirror\n");
            return 4;
        }
    }

    std::printf(
        "[HITL Router:%s] UE=%s:%u serial=%s@%u profile=%s/%016llX status_interval_ms=%u truth_udp=%s:%u state_udp=%s:%u\n",
        config.label.c_str(),
        config.ue_host.c_str(),
        static_cast<unsigned>(config.ue_port),
        config.serial_device.c_str(),
        static_cast<unsigned>(config.baud_rate),
        config.profile_id.c_str(),
        static_cast<unsigned long long>(config.profile_fingerprint),
        static_cast<unsigned>(config.status_interval_ms),
        config.truth_udp_host.c_str(),
        static_cast<unsigned>(config.truth_udp_port),
        config.state_udp_host.c_str(),
        static_cast<unsigned>(config.state_udp_port));

    uint64_t ue_to_board_frames = 0;
    uint64_t ue_to_board_bytes = 0;
    uint64_t board_to_ue_frames = 0;
    uint64_t board_to_ue_bytes = 0;
    uint64_t truth_mirror_frames = 0;
    uint64_t state_mirror_frames = 0;
    uint64_t generation = 0;

    while (!stop_requested(config))
    {
        if (!serial.open())
        {
            write_status(config, "serial_unavailable", false, false, generation,
                         ue_to_board_frames, ue_to_board_bytes,
                         board_to_ue_frames, board_to_ue_bytes);
            std::fprintf(stderr, "[HITL Router:%s] serial open failed; retrying\n",
                         config.label.c_str());
            sleeper.sleep_for_us(1'000'000);
            continue;
        }

        write_status(config, "selecting_profile", true, false, generation,
                     ue_to_board_frames, ue_to_board_bytes,
                     board_to_ue_frames, board_to_ue_bytes);
        const uint32_t profile_nonce = next_profile_nonce();
        const ProfileHandshakeResult profile_result = negotiate_profile(
            config, serial, clock, sleeper, profile_nonce);
        if (profile_result != ProfileHandshakeResult::Ready)
        {
            const char *state =
                profile_result == ProfileHandshakeResult::Rejected
                    ? "profile_rejected"
                    : profile_result == ProfileHandshakeResult::TimedOut
                          ? "profile_timeout"
                          : "profile_io_error";
            write_status(config, state, serial.is_open(), false, generation,
                         ue_to_board_frames, ue_to_board_bytes,
                         board_to_ue_frames, board_to_ue_bytes);
            serial.close();
            if (profile_result == ProfileHandshakeResult::Interrupted)
                break;
            std::fprintf(
                stderr,
                "[HITL Router:%s] profile handshake %s for %s/%016llX\n",
                config.label.c_str(), state, config.profile_id.c_str(),
                static_cast<unsigned long long>(config.profile_fingerprint));
            if (profile_result == ProfileHandshakeResult::Rejected)
                return 3;
            sleeper.sleep_for_us(1'000'000);
            continue;
        }
        std::printf(
            "[HITL Router:%s] profile READY id=%s fingerprint=%016llX nonce=%08X\n",
            config.label.c_str(), config.profile_id.c_str(),
            static_cast<unsigned long long>(config.profile_fingerprint),
            static_cast<unsigned>(profile_nonce));

        write_status(config, "applying_session", true, false, generation,
                     ue_to_board_frames, ue_to_board_bytes,
                     board_to_ue_frames, board_to_ue_bytes);
        hydrox::runtime::HilSessionRejectReason session_reject_reason =
            hydrox::runtime::HilSessionRejectReason::None;
        hydrox::runtime::HilSessionField session_reject_field =
            hydrox::runtime::HilSessionField::None;
        const SessionHandshakeResult session_result = negotiate_session(
            config,
            serial,
            clock,
            sleeper,
            profile_nonce,
            session_reject_reason,
            session_reject_field);
        if (session_result != SessionHandshakeResult::Ready)
        {
            const char *state =
                session_result == SessionHandshakeResult::Rejected
                    ? "session_rejected"
                    : session_result == SessionHandshakeResult::TimedOut
                          ? "session_timeout"
                          : "session_io_error";
            write_status(config, state, serial.is_open(), false, generation,
                         ue_to_board_frames, ue_to_board_bytes,
                         board_to_ue_frames, board_to_ue_bytes);
            serial.close();
            if (session_result == SessionHandshakeResult::Interrupted)
                break;
            std::fprintf(
                stderr,
                "[HITL Router:%s] session handshake %s reason=%u field=%u\n",
                config.label.c_str(),
                state,
                static_cast<unsigned int>(session_reject_reason),
                static_cast<unsigned int>(session_reject_field));
            if (session_result == SessionHandshakeResult::Rejected)
                return 5;
            sleeper.sleep_for_us(1'000'000);
            continue;
        }
        std::printf(
            "[HITL Router:%s] session READY version=%u nonce=%08X\n",
            config.label.c_str(),
            static_cast<unsigned int>(config.session_config.schema_version),
            static_cast<unsigned int>(profile_nonce));

        if (!xlog_recorder.is_open() && !capture_failed)
        {
            std::string xlog_path = config.xlog;
            if (xlog_path == "auto" || xlog_path == "AUTO")
            {
                std::string safe_label = config.label;
                for (char &character : safe_label)
                {
                    const bool safe =
                        (character >= 'A' && character <= 'Z') ||
                        (character >= 'a' && character <= 'z') ||
                        (character >= '0' && character <= '9') ||
                        character == '_' || character == '-';
                    if (!safe)
                        character = '_';
                }
                xlog_path = (std::filesystem::path(config.log_directory.empty() ? "log" : config.log_directory) /
                    ("xlog_hitl_" + safe_label + "_" + std::to_string(hydrox::xlog::unix_time_ns_now()) + ".xlog")).string();
            }
            hydrox::hitl::XLogConfig xlog_config;
            xlog_config.path = xlog_path;
            xlog_config.vehicle = config.label;
            xlog_config.recording_session_id = config.run_id;
            xlog_config.profile_id = config.profile_id;
            xlog_config.profile_fingerprint = config.profile_fingerprint;
            xlog_config.session_config = config.session_config;
            xlog_config.session_digest = config.session_digest;
            std::string xlog_error;
            if (!xlog_recorder.open(xlog_config, &xlog_error))
            {
                std::fprintf(
                    stderr, "[HITL Router:%s] XLog open failed: %s\n",
                    config.label.c_str(), xlog_error.c_str());
                capture_failed = true; // Logging failure must not sever the hardware control link.
            }
            std::printf("[HITL Router:%s] XLog=%s\n",
                        config.label.c_str(), xlog_recorder.path().c_str());
        }

        bool ue_ready = false;
        write_status(config, "waiting_for_ue", true, false, generation,
                     ue_to_board_frames, ue_to_board_bytes,
                     board_to_ue_frames, board_to_ue_bytes);
        while (!stop_requested(config) && serial.is_open() && !ue_ready)
        {
            if (ue.connect())
            {
                ue_ready = true;
                break;
            }

            write_status(config, "waiting_for_ue", true, false, generation,
                         ue_to_board_frames, ue_to_board_bytes,
                         board_to_ue_frames, board_to_ue_bytes);
            const hydrox::platform::IoResult pending = serial.read(
                buffer.data(), buffer.size());
            if (pending.status == hydrox::platform::IoStatus::Error ||
                pending.status == hydrox::platform::IoStatus::Closed)
            {
                serial.close();
                break;
            }
            std::fprintf(stderr, "[HITL Router:%s] profile ready; UE connect failed; retrying\n",
                         config.label.c_str());
            sleeper.sleep_for_us(1'000'000);
        }
        if (stop_requested(config))
            break;
        if (!ue_ready)
        {
            serial.close();
            continue;
        }

        hydrox::runtime::MavlinkDeframer from_ue;
        hydrox::runtime::MavlinkDeframer from_board;
        hydrox::MavlinkHIL truth_decoder;
        hydrox::MavlinkHIL board_decoder;
        hydrox::HilTruthStateMsg latest_truth{};
        hydrox::HilDvlMsg latest_dvl{};
        hydrox::HilActuatorControlsMsg latest_actuator{};
        hydrox::runtime::FixedFrameSender serial_sender;
        bool link_ok = true;
        ++generation;
        uint64_t next_status_us = clock.now_us() +
            static_cast<uint64_t>(config.status_interval_ms) * 1000ULL;
        std::printf("[HITL Router:%s] bridge active generation=%llu\n",
                    config.label.c_str(),
                    static_cast<unsigned long long>(generation));
        write_status(config, "active", true, true, generation,
                     ue_to_board_frames, ue_to_board_bytes,
                     board_to_ue_frames, board_to_ue_bytes);

        while (!stop_requested(config) && link_ok &&
               ue.is_connected() && serial.is_open())
        {
            bool progressed = false;
            const auto now_us = clock.now_us();
            const auto flush_status = serial_sender.flush(serial, now_us);
            if (flush_status == hydrox::runtime::FixedFrameSendStatus::FATAL ||
                flush_status == hydrox::runtime::FixedFrameSendStatus::TIMED_OUT)
            {
                link_ok = false;
                break;
            }

            const int readable = ue.wait_readable(1);
            if (readable < 0)
            {
                link_ok = false;
                break;
            }
            if (readable > 0)
            {
                const int bytes = ue.read(buffer.data(), buffer.size());
                if (bytes < 0)
                {
                    link_ok = false;
                    break;
                }
                if (bytes > 0)
                {
                    progressed = true;
                    from_ue.feed(
                        buffer.data(), static_cast<std::size_t>(bytes),
                        [&](const uint8_t *frame, std::size_t size)
                        {
                            ++ue_to_board_frames;
                            ue_to_board_bytes += static_cast<uint64_t>(size);
                            if (truth_mirror)
                            {
                                const auto decoded = truth_decoder.feed(
                                    frame, size);
                                for (const auto &message : decoded)
                                {
                                    if (message.msg_id == hydrox::MSGID_HIL_DVL)
                                    {
                                        const auto dvl =
                                            truth_decoder.parse_hil_dvl(message);
                                        if (dvl.velocity_valid())
                                            latest_dvl = dvl;
                                        continue;
                                    }
                                    if (message.msg_id ==
                                        hydrox::MSGID_HIL_TRUTH_STATE)
                                    {
                                        const auto truth =
                                            truth_decoder.parse_hil_truth_state(message);
                                        if (truth.valid)
                                            latest_truth = truth;
                                        if (truth_mirror->send(frame, size))
                                            ++truth_mirror_frames;
                                        break;
                                    }
                                }
                            }
                            const auto status = serial_sender.write_frame(
                                serial, frame, size, clock.now_us());
                            if (status ==
                                    hydrox::runtime::FixedFrameSendStatus::FATAL ||
                                status ==
                                    hydrox::runtime::FixedFrameSendStatus::TIMED_OUT ||
                                status ==
                                    hydrox::runtime::FixedFrameSendStatus::OVERSIZE)
                            {
                                link_ok = false;
                            }
                        });
                }
            }

            const hydrox::platform::IoResult serial_read = serial.read(
                buffer.data(), buffer.size());
            if (serial_read.status == hydrox::platform::IoStatus::Ok)
            {
                progressed = true;
                from_board.feed(
                    buffer.data(), serial_read.size,
                    [&](const uint8_t *frame, std::size_t size)
                    {
                        ++board_to_ue_frames;
                        board_to_ue_bytes += static_cast<uint64_t>(size);
                        const auto decoded = board_decoder.feed(frame, size);
                        for (const auto &message : decoded)
                        {
                            if (message.msg_id ==
                                hydrox::MSGID_HIL_ACTUATOR_CONTROLS)
                            {
                                const auto actuator =
                                    board_decoder.parse_hil_actuator_controls(message);
                                if (actuator.valid)
                                    latest_actuator = actuator;
                            }
                            else if (message.msg_id ==
                                     hydrox::MSGID_HIL_FC_STATE && state_mirror)
                            {
                                const auto state =
                                    board_decoder.parse_hil_fc_state(message);
                                if (state.valid && state_mirror->send(frame, size))
                                    ++state_mirror_frames;
                            }
                            else if (message.msg_id ==
                                     hydrox::MSGID_HIL_CONTROL_TRACE)
                            {
                                const auto trace =
                                    board_decoder.parse_hil_control_trace(message);
                                if (trace.valid && xlog_recorder.is_open())
                                {
                                    const hydrox::HilActuatorControlsMsg *matching =
                                        latest_actuator.valid &&
                                        latest_actuator.time_usec == trace.time_usec
                                            ? &latest_actuator : nullptr;
                                    if (!xlog_recorder.record(trace, matching))
                                    {
                                        std::fprintf(
                                            stderr,
                                            "[HITL Router:%s] XLog write failed: %s\n",
                                            config.label.c_str(),
                                            xlog_recorder.last_error().c_str());
                                        capture_failed = true;
                                    }
                                }
                            }
                        }
                        if (!ue.write(frame, size))
                            link_ok = false;
                    });
            }
            else if (serial_read.status !=
                     hydrox::platform::IoStatus::WouldBlock)
            {
                link_ok = false;
            }

            if (config.status_interval_ms > 0 && now_us >= next_status_us)
            {
                std::printf(
                    "[HITL Router:%s] status generation=%llu ue_to_board_frames=%llu ue_to_board_bytes=%llu board_to_ue_frames=%llu board_to_ue_bytes=%llu truth_mirror_frames=%llu state_mirror_frames=%llu\n",
                    config.label.c_str(),
                    static_cast<unsigned long long>(generation),
                    static_cast<unsigned long long>(ue_to_board_frames),
                    static_cast<unsigned long long>(ue_to_board_bytes),
                    static_cast<unsigned long long>(board_to_ue_frames),
                    static_cast<unsigned long long>(board_to_ue_bytes),
                    static_cast<unsigned long long>(truth_mirror_frames),
                    static_cast<unsigned long long>(state_mirror_frames));
                if (latest_truth.valid || latest_actuator.valid)
                {
                    std::printf(
                        "[HITL Router:%s] dynamics truth=(N %.2f E %.2f D %.2f yaw %.3f u %.3f r %.3f) dvl=(mode=%u flags=0x%02X vx %.3f vy %.3f vz %.3f) actuator=(%.3f %.3f %.3f %.3f prop %.3f mode=0x%02X)\n",
                        config.label.c_str(), latest_truth.eta[0],
                        latest_truth.eta[1], latest_truth.eta[2],
                        latest_truth.eta[5], latest_truth.nu[0],
                        latest_truth.nu[5],
                        static_cast<unsigned>(latest_dvl.tracking_mode),
                        static_cast<unsigned>(latest_dvl.flags),
                        latest_dvl.vx, latest_dvl.vy, latest_dvl.vz,
                        latest_actuator.controls[0],
                        latest_actuator.controls[1],
                        latest_actuator.controls[2],
                        latest_actuator.controls[3],
                        latest_actuator.controls[4],
                        static_cast<unsigned>(latest_actuator.mode));
                }
                write_status(config, "active", true, true, generation,
                             ue_to_board_frames, ue_to_board_bytes,
                             board_to_ue_frames, board_to_ue_bytes);
                next_status_us = now_us +
                    static_cast<uint64_t>(config.status_interval_ms) * 1000ULL;
            }

            if (!progressed)
                sleeper.sleep_for_us(1'000);
        }

        ue.disconnect();
        serial.close();
        write_status(config, "reconnecting", false, false, generation,
                     ue_to_board_frames, ue_to_board_bytes,
                     board_to_ue_frames, board_to_ue_bytes);
        std::fprintf(stderr, "[HITL Router:%s] link lost; reconnecting\n",
                     config.label.c_str());
        if (!stop_requested(config))
            sleeper.sleep_for_us(500'000);
    }
    write_status(config, "stopped", false, false, generation,
                 ue_to_board_frames, ue_to_board_bytes,
                 board_to_ue_frames, board_to_ue_bytes);
    if (xlog_recorder.is_open())
    {
        const auto stats = xlog_recorder.stats();
        xlog_recorder.close();
        std::printf(
            "[HITL Router:%s] XLog closed traces=%llu with_actuator=%llu "
            "without_actuator=%llu missing_trace_ticks=%llu\n",
            config.label.c_str(),
            static_cast<unsigned long long>(stats.traces),
            static_cast<unsigned long long>(stats.traces_with_actuator),
            static_cast<unsigned long long>(stats.traces_without_actuator),
            static_cast<unsigned long long>(stats.missing_trace_ticks));
    }
    xlog_recorder.close();
    return capture_failed || !xlog_recorder.last_error().empty() ? 6 : 0;
}
