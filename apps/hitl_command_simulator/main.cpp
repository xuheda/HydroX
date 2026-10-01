#include "hydrox/platform/host/host_clock.h"
#include "hydrox/platform/host/host_serial_byte_stream.h"
#include "hydrox/platform/host/host_sleeper.h"
#include "hydrox/runtime/ecaa9_diagnostic_mission.h"
#include "hydrox/runtime/fixed_frame_sender.h"
#include "hydrox/runtime/hil_contract.h"
#include "hydrox/runtime/hitl_command_codec.h"
#include "mavlink_hil.h"
#include "sitl/dds_worker.h"
#include "sitl/sitl_platform.h"

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
#include <limits>
#include <memory>
#include <sstream>
#include <string>

namespace
{
constexpr uint32_t kDefaultBaud = 921600;
constexpr double kDefaultRateHz = 20.0;
constexpr double kMinRateHz = 5.0;
constexpr double kMaxRateHz = 100.0;
constexpr uint32_t kNeutralFramesOnStop = 5;
constexpr double kPi = 3.14159265358979323846;

std::atomic<bool> g_running{true};

struct Config
{
    Config()
    {
        setpoint.lookahead_m = 8.0;
        setpoint.arrival_radius_m = 2.0;
    }

    std::string serial_device;
    std::string status_file;
    uint32_t baud = kDefaultBaud;
    double rate_hz = kDefaultRateHz;
    double duration_s = 0.0;
    hydrox::GNCMode mode = hydrox::GNCMode::DISABLED;
    hydrox::GNCSetpoint setpoint{};
    bool hardware_flow_control = true;
    bool arm_command = false;
    bool validate_only = false;
    bool rate_explicit = false;
    std::string mission;
    std::string truth_udp_bind = "127.0.0.1";
    uint16_t truth_udp_port = 14610;
    uint32_t state_timeout_ms = 500;
    uint32_t parent_pid = 0;
    std::string vehicle;
    std::string dds_host = "127.0.0.1";
    uint16_t dds_port = 8888;
    uint16_t ros_domain_id = 0;
    uint32_t dds_client_key = 1;
    std::string state_udp_bind = "127.0.0.1";
    uint16_t state_udp_port = 0;

    bool ros_bridge_enabled() const noexcept
    {
        return !vehicle.empty();
    }
};

const char *mission_state_name(uint8_t mission_state) noexcept
{
    switch (mission_state)
    {
    case 0: return "IDLE";
    case 1: return "RUNNING";
    case 2: return "COMPLETE";
    case 3: return "FAILED";
    default: return "UNKNOWN";
    }
}

const char *mode_name(hydrox::GNCMode mode) noexcept
{
    switch (mode)
    {
    case hydrox::GNCMode::DISABLED: return "DISABLED";
    case hydrox::GNCMode::DEPTH_HOLD: return "DEPTH_HOLD";
    case hydrox::GNCMode::WAYPOINT_3D: return "WAYPOINT_3D";
    case hydrox::GNCMode::DP: return "DP";
    case hydrox::GNCMode::SURFACE: return "SURFACE";
    }
    return "UNKNOWN";
}

bool parse_mode(const std::string &value, hydrox::GNCMode &mode) noexcept
{
    if (value == "disabled")
        mode = hydrox::GNCMode::DISABLED;
    else if (value == "depth-hold")
        mode = hydrox::GNCMode::DEPTH_HOLD;
    else if (value == "waypoint-3d")
        mode = hydrox::GNCMode::WAYPOINT_3D;
    else if (value == "dp")
        mode = hydrox::GNCMode::DP;
    else if (value == "surface")
        mode = hydrox::GNCMode::SURFACE;
    else
        return false;
    return true;
}

bool parse_double(const char *text, double &value) noexcept
{
    if (text == nullptr || text[0] == '\0')
        return false;
    char *end = nullptr;
    value = std::strtod(text, &end);
    return end != text && end != nullptr && end[0] == '\0' &&
           std::isfinite(value);
}

bool parse_u32(const char *text, uint32_t &value) noexcept
{
    if (text == nullptr || text[0] == '\0' || text[0] == '-')
        return false;
    char *end = nullptr;
    const unsigned long parsed = std::strtoul(text, &end, 10);
    if (end == text || end == nullptr || end[0] != '\0' ||
        parsed > std::numeric_limits<uint32_t>::max())
        return false;
    value = static_cast<uint32_t>(parsed);
    return true;
}

std::string json_escape(const std::string &value)
{
    std::ostringstream escaped;
    for (const char ch : value)
    {
        switch (ch)
        {
        case '\\': escaped << "\\\\"; break;
        case '"': escaped << "\\\""; break;
        case '\n': escaped << "\\n"; break;
        case '\r': escaped << "\\r"; break;
        case '\t': escaped << "\\t"; break;
        default:
            if (static_cast<unsigned char>(ch) >= 0x20)
                escaped << ch;
            break;
        }
    }
    return escaped.str();
}

uint64_t unix_time_ms() noexcept
{
    return static_cast<uint64_t>(std::chrono::duration_cast<
        std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
}

bool write_status(const Config &config, const char *state,
                  uint32_t generation, uint64_t frames_sent,
                  uint64_t bytes_sent, uint16_t sequence,
                  hydrox::GNCMode active_mode = hydrox::GNCMode::DISABLED,
                  bool dds_connected = false,
                  bool fc_state_fresh = false,
                  bool setpoint_fresh = false,
                  uint64_t state_frames_received = 0)
{
    if (!config.ros_bridge_enabled())
        active_mode = config.mode;
    std::ostringstream json;
    json << "{\"schema_version\":2"
         << ",\"serial_device\":\"" << json_escape(config.serial_device) << "\""
         << ",\"baud_rate\":" << config.baud
         << ",\"hardware_flow_control\":"
         << (config.hardware_flow_control ? "true" : "false")
         << ",\"state\":\"" << json_escape(state) << "\""
         << ",\"mode\":\"" << mode_name(active_mode) << "\""
         << ",\"vehicle\":\"" << json_escape(config.vehicle) << "\""
         << ",\"ros_bridge\":"
         << (config.ros_bridge_enabled() ? "true" : "false")
         << ",\"dds_connected\":" << (dds_connected ? "true" : "false")
         << ",\"fc_state_fresh\":" << (fc_state_fresh ? "true" : "false")
         << ",\"setpoint_fresh\":" << (setpoint_fresh ? "true" : "false")
         << ",\"state_frames_received\":" << state_frames_received
         << ",\"sender_generation\":" << generation
         << ",\"frames_sent\":" << frames_sent
         << ",\"bytes_sent\":" << bytes_sent
         << ",\"last_sequence\":" << sequence
         << ",\"board_acknowledged\":false"
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
    if (!output)
        return false;
    output << payload << "\n";
    output.close();
    if (!output)
        return false;

    return hydrox::sitl::replace_file_atomically(
        temporary, config.status_file);
}

void print_usage(const char *program)
{
    std::fprintf(
        stderr,
        "Usage: %s --serial <COM12|/dev/ttyUSB0> [options]\n"
        "\n"
        "Modes:\n"
        "  --mode disabled|depth-hold|waypoint-3d|dp|surface\n"
        "  --mission ecaa9-diagnostic  run the accepted closed-loop SITL route\n"
        "\n"
        "ROS 2 HITL bridge (same topics as SITL):\n"
        "  --vehicle <name>           enable ROS bridge mode\n"
        "  --dds-host <ip>            (default 127.0.0.1)\n"
        "  --dds-port <port>          (default 8888)\n"
        "  --ros-domain-id <0..232>   (default 0)\n"
        "  --dds-client-key <key>     unique non-zero XRCE client key\n"
        "  --state-udp-bind <ip>      (default 127.0.0.1)\n"
        "  --state-udp-port <port>    FC state input from TELEM1 Router\n"
        "\n"
        "Closed-loop diagnostic state input:\n"
        "  --truth-udp-bind <ip>      (default 127.0.0.1)\n"
        "  --truth-udp-port <port>    (default 14610)\n"
        "  --state-timeout-ms <ms>    (default 500)\n"
        "\n"
        "Setpoint options (SI units except angles):\n"
        "  --depth-m <m>             --heading-deg <deg>\n"
        "  --surge-mps <m/s>         --yaw-rate-deg-s <deg/s>\n"
        "  --north-m <m>             --east-m <m>\n"
        "  --down-m <m>              --path-start-north-m <m>\n"
        "  --path-start-east-m <m>   --lookahead-m <m>\n"
        "  --arrival-radius-m <m>    --hold-heading\n"
        "  --use-yaw-rate            --use-path-segment\n"
        "\n"
        "Transport and safety:\n"
        "  --baud <baud>             (default 921600)\n"
        "  --rate-hz <5..100>        (default 20)\n"
        "  --duration-s <seconds>    (0 means until Ctrl+C)\n"
        "  --no-flow-control         (TELEM2 normally requires RTS/CTS)\n"
        "  --status-file <path>      write atomic JSON diagnostics\n"
        "  --parent-pid <pid>        exit when the owning OceanX process exits\n"
        "  --arm-command             required for non-disabled modes\n"
        "  --validate-only           validate without opening serial\n",
        program);
}

bool take_value(int argc, char **argv, int &index,
                const char *option, const char *&value)
{
    if (index + 1 >= argc)
    {
        std::fprintf(stderr, "Missing value for %s\n", option);
        return false;
    }
    value = argv[++index];
    return true;
}

bool parse_number_option(int argc, char **argv, int &index,
                         const std::string &option, double &target)
{
    const char *value = nullptr;
    return take_value(argc, argv, index, option.c_str(), value) &&
           parse_double(value, target);
}

bool parse_args(int argc, char **argv, Config &config)
{
    for (int index = 1; index < argc; ++index)
    {
        const std::string option = argv[index];
        const char *value = nullptr;
        double angle_degrees = 0.0;

        if (option == "--help" || option == "-h")
        {
            print_usage(argv[0]);
            std::exit(0);
        }
        else if (option == "--serial")
        {
            if (!take_value(argc, argv, index, option.c_str(), value))
                return false;
            config.serial_device = value;
        }
        else if (option == "--status-file")
        {
            if (!take_value(argc, argv, index, option.c_str(), value))
                return false;
            config.status_file = value;
        }
        else if (option == "--parent-pid")
        {
            if (!take_value(argc, argv, index, option.c_str(), value) ||
                !parse_u32(value, config.parent_pid))
            {
                std::fprintf(stderr, "Invalid parent PID\n");
                return false;
            }
        }
        else if (option == "--vehicle")
        {
            if (!take_value(argc, argv, index, option.c_str(), value))
                return false;
            config.vehicle = value;
        }
        else if (option == "--dds-host")
        {
            if (!take_value(argc, argv, index, option.c_str(), value))
                return false;
            config.dds_host = value;
        }
        else if (option == "--state-udp-bind")
        {
            if (!take_value(argc, argv, index, option.c_str(), value))
                return false;
            config.state_udp_bind = value;
        }
        else if (option == "--mission")
        {
            if (!take_value(argc, argv, index, option.c_str(), value))
                return false;
            config.mission = value;
        }
        else if (option == "--truth-udp-bind")
        {
            if (!take_value(argc, argv, index, option.c_str(), value))
                return false;
            config.truth_udp_bind = value;
        }
        else if (option == "--mode")
        {
            if (!take_value(argc, argv, index, option.c_str(), value) ||
                !parse_mode(value, config.mode))
            {
                std::fprintf(stderr, "Invalid mode\n");
                return false;
            }
        }
        else if (option == "--baud")
        {
            if (!take_value(argc, argv, index, option.c_str(), value) ||
                !parse_u32(value, config.baud) || config.baud == 0)
            {
                std::fprintf(stderr, "Invalid baud rate\n");
                return false;
            }
        }
        else if (option == "--rate-hz")
        {
            if (!parse_number_option(argc, argv, index, option, config.rate_hz))
                return false;
            config.rate_explicit = true;
        }
        else if (option == "--truth-udp-port")
        {
            uint32_t parsed = 0;
            if (!take_value(argc, argv, index, option.c_str(), value) ||
                !parse_u32(value, parsed) || parsed == 0 || parsed > 65535)
            {
                std::fprintf(stderr, "Invalid truth UDP port\n");
                return false;
            }
            config.truth_udp_port = static_cast<uint16_t>(parsed);
        }
        else if (option == "--dds-port")
        {
            uint32_t parsed = 0;
            if (!take_value(argc, argv, index, option.c_str(), value) ||
                !parse_u32(value, parsed) || parsed == 0 || parsed > 65535)
            {
                std::fprintf(stderr, "Invalid DDS port\n");
                return false;
            }
            config.dds_port = static_cast<uint16_t>(parsed);
        }
        else if (option == "--ros-domain-id")
        {
            uint32_t parsed = 0;
            if (!take_value(argc, argv, index, option.c_str(), value) ||
                !parse_u32(value, parsed) || parsed > 232)
            {
                std::fprintf(stderr, "Invalid ROS domain ID\n");
                return false;
            }
            config.ros_domain_id = static_cast<uint16_t>(parsed);
        }
        else if (option == "--dds-client-key")
        {
            if (!take_value(argc, argv, index, option.c_str(), value) ||
                !parse_u32(value, config.dds_client_key) ||
                config.dds_client_key == 0)
            {
                std::fprintf(stderr, "Invalid DDS client key\n");
                return false;
            }
        }
        else if (option == "--state-udp-port")
        {
            uint32_t parsed = 0;
            if (!take_value(argc, argv, index, option.c_str(), value) ||
                !parse_u32(value, parsed) || parsed == 0 || parsed > 65535)
            {
                std::fprintf(stderr, "Invalid FC state UDP port\n");
                return false;
            }
            config.state_udp_port = static_cast<uint16_t>(parsed);
        }
        else if (option == "--state-timeout-ms")
        {
            if (!take_value(argc, argv, index, option.c_str(), value) ||
                !parse_u32(value, config.state_timeout_ms) ||
                config.state_timeout_ms < 50 || config.state_timeout_ms > 5000)
            {
                std::fprintf(stderr, "State timeout must be 50..5000 ms\n");
                return false;
            }
        }
        else if (option == "--duration-s")
        {
            if (!parse_number_option(argc, argv, index, option,
                                     config.duration_s))
                return false;
        }
        else if (option == "--depth-m")
        {
            if (!parse_number_option(argc, argv, index, option,
                                     config.setpoint.depth_ref))
                return false;
        }
        else if (option == "--heading-deg")
        {
            if (!parse_number_option(argc, argv, index, option, angle_degrees))
                return false;
            config.setpoint.heading_ref = angle_degrees * kPi / 180.0;
        }
        else if (option == "--surge-mps")
        {
            if (!parse_number_option(argc, argv, index, option,
                                     config.setpoint.surge_ref))
                return false;
        }
        else if (option == "--yaw-rate-deg-s")
        {
            if (!parse_number_option(argc, argv, index, option, angle_degrees))
                return false;
            config.setpoint.yaw_rate_ref = angle_degrees * kPi / 180.0;
        }
        else if (option == "--north-m")
        {
            if (!parse_number_option(argc, argv, index, option,
                                     config.setpoint.wp_n))
                return false;
        }
        else if (option == "--east-m")
        {
            if (!parse_number_option(argc, argv, index, option,
                                     config.setpoint.wp_e))
                return false;
        }
        else if (option == "--down-m")
        {
            if (!parse_number_option(argc, argv, index, option,
                                     config.setpoint.wp_d))
                return false;
        }
        else if (option == "--path-start-north-m")
        {
            if (!parse_number_option(argc, argv, index, option,
                                     config.setpoint.path_start_n))
                return false;
        }
        else if (option == "--path-start-east-m")
        {
            if (!parse_number_option(argc, argv, index, option,
                                     config.setpoint.path_start_e))
                return false;
        }
        else if (option == "--lookahead-m")
        {
            if (!parse_number_option(argc, argv, index, option,
                                     config.setpoint.lookahead_m))
                return false;
        }
        else if (option == "--arrival-radius-m")
        {
            if (!parse_number_option(argc, argv, index, option,
                                     config.setpoint.arrival_radius_m))
                return false;
        }
        else if (option == "--hold-heading")
            config.setpoint.hold_heading = true;
        else if (option == "--use-yaw-rate")
            config.setpoint.use_yaw_rate_ref = true;
        else if (option == "--use-path-segment")
            config.setpoint.use_path_segment = true;
        else if (option == "--no-flow-control")
            config.hardware_flow_control = false;
        else if (option == "--arm-command")
            config.arm_command = true;
        else if (option == "--validate-only")
            config.validate_only = true;
        else
        {
            std::fprintf(stderr, "Unknown option: %s\n", option.c_str());
            return false;
        }
    }

    if (config.serial_device.empty())
    {
        std::fprintf(stderr, "--serial is required\n");
        return false;
    }
    if (config.ros_bridge_enabled())
    {
        if (!config.mission.empty())
        {
            std::fprintf(stderr, "ROS bridge mode and diagnostic mission are mutually exclusive\n");
            return false;
        }
        if (config.state_udp_port == 0 || config.dds_host.empty() ||
            config.state_udp_bind.empty())
        {
            std::fprintf(stderr, "ROS bridge mode requires DDS and FC state UDP configuration\n");
            return false;
        }
    }
    if (!config.mission.empty())
    {
        if (config.mission != "ecaa9-diagnostic")
        {
            std::fprintf(stderr, "Unknown mission: %s\n", config.mission.c_str());
            return false;
        }
        config.mode = hydrox::GNCMode::WAYPOINT_3D;
        if (!config.rate_explicit)
            config.rate_hz = 10.0;
    }
    if (config.rate_hz < kMinRateHz || config.rate_hz > kMaxRateHz)
    {
        std::fprintf(stderr, "--rate-hz must be between %.0f and %.0f\n",
                     kMinRateHz, kMaxRateHz);
        return false;
    }
    if (config.duration_s < 0.0)
    {
        std::fprintf(stderr, "--duration-s cannot be negative\n");
        return false;
    }
    if (config.mode != hydrox::GNCMode::DISABLED && !config.arm_command)
    {
        std::fprintf(stderr,
                     "Non-disabled modes require explicit --arm-command\n");
        return false;
    }
    if (!hydrox::runtime::valid_gnc_setpoint(config.setpoint, config.mode))
    {
        std::fprintf(stderr,
                     "Setpoint is outside the HydroX command contract\n");
        return false;
    }
    return true;
}

uint32_t make_generation() noexcept
{
    const uint64_t steady = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    const uint64_t wall = static_cast<uint64_t>(
        std::chrono::system_clock::now().time_since_epoch().count());
    uint32_t generation = static_cast<uint32_t>(steady ^ (wall >> 17) ^ wall);
    if (generation == 0)
        generation = 1;
    return generation;
}

void handle_signal(int) noexcept
{
    g_running.store(false, std::memory_order_release);
}

hydrox::runtime::FixedFrameSendStatus send_complete_frame(
    hydrox::platform::host::HostSerialByteStream &serial,
    hydrox::runtime::FixedFrameSender &sender,
    hydrox::platform::host::HostClock &clock,
    hydrox::platform::host::HostSleeper &sleeper,
    hydrox::runtime::HitlCommandFrame &frame,
    hydrox::GNCMode mode,
    const hydrox::GNCSetpoint &setpoint)
{
    frame.sample.mode = mode;
    frame.sample.setpoint = setpoint;
    frame.sender_time_us = clock.now_us();
    ++frame.sequence;
    const auto encoded = hydrox::runtime::encode_hitl_command(frame);
    auto status = sender.write_frame(
        serial, encoded.data(), encoded.size(), frame.sender_time_us);
    while (status == hydrox::runtime::FixedFrameSendStatus::TAIL_PENDING)
    {
        sleeper.sleep_for_us(1'000);
        status = sender.flush(serial, clock.now_us());
    }
    return status;
}

bool send_succeeded(hydrox::runtime::FixedFrameSendStatus status) noexcept
{
    return status == hydrox::runtime::FixedFrameSendStatus::COMPLETE;
}

hydrox::GNCSetpoint runtime_setpoint_from_dds(
    const hydrox::GNCSetpointDds &source)
{
    hydrox::GNCSetpoint setpoint;
    setpoint.depth_ref = source.depth_ref;
    setpoint.heading_ref = source.heading_ref;
    setpoint.surge_ref = source.surge_ref;
    setpoint.use_yaw_rate_ref = source.use_yaw_rate_ref;
    setpoint.yaw_rate_ref = source.yaw_rate_ref;
    setpoint.wp_n = source.wp_n;
    setpoint.wp_e = source.wp_e;
    setpoint.wp_d = source.wp_d;
    setpoint.use_path_segment = source.use_path_segment;
    setpoint.path_start_n = source.path_start_n;
    setpoint.path_start_e = source.path_start_e;
    setpoint.lookahead_m = source.lookahead_m;
    setpoint.arrival_radius_m = source.arrival_radius_m;
    setpoint.hold_heading = source.hold_heading;
    return setpoint;
}

hydrox::sitl::DdsTelemetrySample dds_telemetry_from_fc_state(
    const hydrox::HilFcStateMsg &state)
{
    hydrox::sitl::DdsTelemetrySample sample;
    hydrox::FcSnapshot &snapshot = sample.snapshot;
    snapshot.timestamp_us = state.time_usec;
    for (std::size_t index = 0; index < 6; ++index)
    {
        snapshot.eta[index] = state.eta[index];
        snapshot.nu[index] = state.nu[index];
    }
    snapshot.depth_m = state.depth_m;
    snapshot.dvl_valid =
        (state.flags & hydrox::HIL_FC_STATE_FLAG_DVL_VALID) != 0 ? 1u : 0u;
    for (std::size_t index = 0; index < 3; ++index)
    {
        snapshot.dvl_vel[index] = state.dvl_vel[index];
        snapshot.acc[index] = state.acc[index];
        snapshot.gyro[index] = state.gyro[index];
    }
    snapshot.gps_lat = state.gps_lat;
    snapshot.gps_lon = state.gps_lon;
    snapshot.gps_alt = state.gps_alt;
    snapshot.gps_vn = state.gps_vel_ned[0];
    snapshot.gps_ve = state.gps_vel_ned[1];
    snapshot.gps_vd = state.gps_vel_ned[2];
    snapshot.gps_fix = state.gps_fix;
    snapshot.gps_satellites = state.gps_satellites;
    for (std::size_t index = 0; index < 8; ++index)
        snapshot.normalized[index] = state.normalized[index];
    for (std::size_t index = 0; index < 4; ++index)
        snapshot.fins[index] = state.normalized[index];
    snapshot.thrust = state.normalized[4];
    snapshot.rpm = state.commanded_rpm;
    snapshot.actuator_channel_count = state.actuator_channel_count;
    std::snprintf(snapshot.mission_state, sizeof(snapshot.mission_state),
                  "%s", mission_state_name(state.mission_state));
    snapshot.motor_rpm_actual = state.motor_rpm_actual;
    snapshot.motor_thrust_N = state.motor_thrust_N;
    snapshot.motor_power_W = state.motor_power_W;
    snapshot.motor_current_A = state.motor_current_A;
    snapshot.power_total_W = state.power_total_W;
    snapshot.energy_Wh = state.energy_Wh;
    snapshot.battery_soc = state.battery_soc;
    snapshot.V_terminal = state.voltage_terminal;
    snapshot.runtime_rem_s = state.runtime_remaining_s;

    sample.gnc_mode = mode_name(static_cast<hydrox::GNCMode>(state.mode));
    sample.hil_connected = true;
    sample.ekf_initialized =
        (state.flags & hydrox::HIL_FC_STATE_FLAG_EKF_INITIALIZED) != 0;
    sample.armed = (state.flags & hydrox::HIL_FC_STATE_FLAG_ARMED) != 0;
    sample.actuator_authorized =
        (state.flags & hydrox::HIL_FC_STATE_FLAG_ACTUATOR_AUTHORIZED) != 0;
    return sample;
}

} // namespace

int main(int argc, char **argv)
{
    Config config;
    if (!parse_args(argc, argv, config))
    {
        print_usage(argv[0]);
        return 2;
    }

    std::printf(
        "[HITL Command Simulator] serial=%s baud=%u flow=%s mode=%s rate=%.1f Hz mission=%s\n",
        config.serial_device.c_str(), config.baud,
        config.hardware_flow_control ? "RTS/CTS" : "none",
        mode_name(config.mode), config.rate_hz,
        config.mission.empty() ? "static" : config.mission.c_str());
    if (config.validate_only)
    {
        std::printf("[HITL Command Simulator] configuration valid\n");
        return 0;
    }

    hydrox::sitl::ParentProcessGuard parent_guard(config.parent_pid);
    if (!parent_guard.arm())
    {
        std::fprintf(stderr, "Failed to monitor parent PID %u\n",
                     static_cast<unsigned>(config.parent_pid));
        return 5;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const uint32_t generation = make_generation();
    uint64_t frames_sent = 0;
    uint64_t bytes_sent = 0;
    hydrox::runtime::HitlCommandFrame frame{};
    frame.sender_generation = generation;

    write_status(config, "opening", generation, frames_sent, bytes_sent,
                 frame.sequence);
    hydrox::platform::host::HostSerialByteStream serial(
        config.serial_device, config.baud, config.hardware_flow_control);
    if (!serial.open())
    {
        write_status(config, "serial_unavailable", generation, frames_sent,
                     bytes_sent, frame.sequence);
        std::fprintf(stderr, "Failed to open serial port %s\n",
                     config.serial_device.c_str());
        return 3;
    }

    hydrox::platform::host::HostClock clock;
    hydrox::platform::host::HostSleeper sleeper;
    hydrox::runtime::FixedFrameSender sender(250'000);
    hydrox::sitl::NetworkRuntime network;
    std::unique_ptr<hydrox::sitl::UdpSender> truth_input;
    std::unique_ptr<hydrox::sitl::UdpSender> state_input;
    std::unique_ptr<hydrox::sitl::DdsWorker> dds_worker;
    hydrox::sitl::DdsControlLinkState dds_control_link;
    hydrox::MavlinkHIL truth_decoder;
    hydrox::MavlinkHIL state_decoder;
    hydrox::runtime::EcaA9DiagnosticMission mission;
    hydrox::runtime::MissionGuidanceState mission_state{};
    hydrox::HilFcStateMsg latest_fc_state{};
    hydrox::GNCSetpoint latest_ros_setpoint{};
    hydrox::GNCMode latest_ros_mode = hydrox::GNCMode::DISABLED;
    bool truth_seen = false;
    bool state_was_fresh = false;
    bool have_ros_setpoint = false;
    uint64_t last_truth_us = 0;
    uint64_t last_fc_state_us = 0;
    uint64_t last_ros_setpoint_us = 0;
    uint64_t state_frames_received = 0;
    uint64_t last_dds_setpoint_sequence = 0;
    uint64_t last_dds_status_sequence = 0;
    std::array<uint8_t, hydrox::MAVLINK_MAX_PACKET_LEN> truth_bytes{};
    std::array<uint8_t, hydrox::MAVLINK_MAX_PACKET_LEN> state_bytes{};
    if (config.ros_bridge_enabled())
    {
        if (!network.ready())
        {
            std::fprintf(stderr, "Failed to initialize UDP network runtime\n");
            serial.close();
            return 4;
        }
        state_input = std::make_unique<hydrox::sitl::UdpSender>(
            config.state_udp_bind, config.state_udp_port, false);
        if (!state_input->is_open() || !state_input->bind_local(
                config.state_udp_bind, config.state_udp_port))
        {
            std::fprintf(stderr, "Failed to bind FC state UDP input %s:%u\n",
                         config.state_udp_bind.c_str(), config.state_udp_port);
            serial.close();
            return 4;
        }
        dds_worker = std::make_unique<hydrox::sitl::DdsWorker>(
            hydrox::sitl::DdsWorkerConfig{
                config.dds_host,
                config.dds_port,
                config.ros_domain_id,
                config.vehicle,
                config.dds_client_key,
                false,
            },
            clock);
    }
    if (!config.mission.empty())
    {
        if (!network.ready())
        {
            std::fprintf(stderr, "Failed to initialize UDP network runtime\n");
            serial.close();
            return 4;
        }
        truth_input = std::make_unique<hydrox::sitl::UdpSender>(
            config.truth_udp_bind, config.truth_udp_port, false);
        if (!truth_input->is_open() || !truth_input->bind_local(
                config.truth_udp_bind, config.truth_udp_port))
        {
            std::fprintf(stderr, "Failed to bind truth UDP input %s:%u\n",
                         config.truth_udp_bind.c_str(), config.truth_udp_port);
            serial.close();
            return 4;
        }
    }
    const uint64_t period_us = static_cast<uint64_t>(
        std::llround(1'000'000.0 / config.rate_hz));
    const uint64_t started_us = clock.now_us();
    uint64_t next_send_us = started_us;
    uint64_t next_status_us = started_us;
    bool success = true;

    std::printf(
        "[HITL Command Simulator] transmitting without command acknowledgement "
        "generation=%u; Ctrl+C stops safely\n",
        generation);
    while (g_running.load(std::memory_order_acquire))
    {
        const uint64_t now_us = clock.now_us();
        if (state_input && dds_worker)
        {
            int received = 0;
            while ((received = state_input->receive(
                        state_bytes.data(), state_bytes.size())) > 0)
            {
                const auto messages = state_decoder.feed(
                    state_bytes.data(), static_cast<std::size_t>(received));
                for (const auto &message : messages)
                {
                    if (message.msg_id != hydrox::MSGID_HIL_FC_STATE)
                        continue;
                    const hydrox::HilFcStateMsg state =
                        state_decoder.parse_hil_fc_state(message);
                    if (!state.valid)
                        continue;
                    latest_fc_state = state;
                    last_fc_state_us = now_us;
                    ++state_frames_received;
                    (void)dds_worker->try_submit_telemetry(
                        dds_telemetry_from_fc_state(state));
                }
            }
            if (received < 0)
            {
                std::fprintf(stderr, "FC state UDP receive failed\n");
                success = false;
                break;
            }

            hydrox::sitl::DdsConnectionStatus dds_status;
            if (dds_worker->try_take_connection_status(
                    last_dds_status_sequence, dds_status))
            {
                if (dds_control_link.observe(dds_status))
                {
                    have_ros_setpoint = false;
                    latest_ros_mode = hydrox::GNCMode::DISABLED;
                    latest_ros_setpoint = {};
                    last_ros_setpoint_us = 0;
                }
            }
            hydrox::sitl::DdsSetpointSample dds_setpoint;
            if (dds_worker->try_take_setpoint(
                    last_dds_setpoint_sequence, dds_setpoint) &&
                dds_control_link.accepts_setpoint(
                    dds_setpoint.session_generation))
            {
                latest_ros_setpoint =
                    runtime_setpoint_from_dds(dds_setpoint.setpoint);
                latest_ros_mode = static_cast<hydrox::GNCMode>(
                    dds_setpoint.setpoint.mode);
                last_ros_setpoint_us = dds_setpoint.received_at_us;
                have_ros_setpoint = true;
            }
        }
        if (truth_input)
        {
            int received = 0;
            while ((received = truth_input->receive(
                        truth_bytes.data(), truth_bytes.size())) > 0)
            {
                const auto messages = truth_decoder.feed(
                    truth_bytes.data(), static_cast<std::size_t>(received));
                for (const auto &message : messages)
                {
                    if (message.msg_id != hydrox::MSGID_HIL_TRUTH_STATE)
                        continue;
                    const auto truth = truth_decoder.parse_hil_truth_state(message);
                    if (!truth.valid)
                        continue;
                    mission_state.north_m = truth.eta[0];
                    mission_state.east_m = truth.eta[1];
                    mission_state.down_m = truth.eta[2];
                    mission_state.yaw_rad = truth.eta[5];
                    truth_seen = true;
                    last_truth_us = now_us;
                }
            }
            if (received < 0)
            {
                std::fprintf(stderr, "Truth UDP receive failed\n");
                success = false;
                break;
            }
        }
        if (config.duration_s > 0.0 &&
            static_cast<double>(now_us - started_us) * 1e-6 >=
                config.duration_s)
            break;

        if (now_us < next_send_us)
        {
            sleeper.sleep_until_us(next_send_us);
            continue;
        }

        hydrox::GNCMode active_mode = config.mode;
        hydrox::GNCSetpoint active_setpoint = config.setpoint;
        const uint64_t timeout_us =
            static_cast<uint64_t>(config.state_timeout_ms) * 1000ULL;
        bool fc_state_fresh = true;
        bool setpoint_fresh = true;
        bool dds_connected = false;
        if (config.ros_bridge_enabled())
        {
            dds_connected = dds_control_link.is_connected();
            fc_state_fresh = latest_fc_state.valid &&
                (latest_fc_state.flags &
                 hydrox::HIL_FC_STATE_FLAG_EKF_INITIALIZED) != 0 &&
                last_fc_state_us > 0 && now_us >= last_fc_state_us &&
                now_us - last_fc_state_us <= timeout_us;
            setpoint_fresh = have_ros_setpoint &&
                last_ros_setpoint_us > 0 && now_us >= last_ros_setpoint_us &&
                now_us - last_ros_setpoint_us <= timeout_us;
            if (dds_connected && fc_state_fresh && setpoint_fresh)
            {
                active_mode = latest_ros_mode;
                active_setpoint = latest_ros_setpoint;
            }
            else
            {
                active_mode = hydrox::GNCMode::DISABLED;
                active_setpoint = {};
            }
        }
        else if (!config.mission.empty())
        {
            fc_state_fresh = truth_seen && now_us >= last_truth_us &&
                             now_us - last_truth_us <= timeout_us;
            if (fc_state_fresh)
            {
                if (!state_was_fresh)
                    mission.reset();
                active_setpoint = mission.update(mission_state);
                active_mode = hydrox::GNCMode::WAYPOINT_3D;
            }
            else
            {
                active_mode = hydrox::GNCMode::DISABLED;
                active_setpoint = {};
            }
            state_was_fresh = fc_state_fresh;
        }
        const auto status = send_complete_frame(
            serial, sender, clock, sleeper, frame, active_mode, active_setpoint);
        if (!send_succeeded(status))
        {
            std::fprintf(stderr, "Command serial write failed (status=%u)\n",
                         static_cast<unsigned int>(status));
            success = false;
            break;
        }

        ++frames_sent;
        bytes_sent += hydrox::runtime::kHitlCommandFrameSize;
        next_send_us += period_us;
        if (next_send_us <= now_us)
            next_send_us = now_us + period_us;

        if (now_us >= next_status_us)
        {
            const char *state = "transmitting_unconfirmed";
            if (config.ros_bridge_enabled())
            {
                if (!dds_connected)
                    state = "waiting_for_dds";
                else if (!fc_state_fresh)
                    state = "waiting_for_fc_state";
                else if (!setpoint_fresh)
                    state = "waiting_for_setpoint";
                else
                    state = "bridging";
            }
            else if (!config.mission.empty() && !fc_state_fresh)
            {
                state = "waiting_for_truth";
            }
            write_status(config, state, generation,
                         frames_sent, bytes_sent, frame.sequence,
                         active_mode, dds_connected, fc_state_fresh,
                         setpoint_fresh, state_frames_received);
            if (!config.mission.empty() && fc_state_fresh)
            {
                const uint64_t age_ms = (now_us - last_truth_us) / 1000ULL;
                std::printf(
                    "[HITL Command Simulator] tx=%llu seq=%u wp=%zu pos=(%.1f,%.1f,%.1f) ref=(yaw %.2f rate %.2f speed %.2f depth %.1f) state_age=%llums\n",
                    static_cast<unsigned long long>(frames_sent),
                    static_cast<unsigned int>(frame.sequence),
                    mission.waypoint_index(), mission_state.north_m,
                    mission_state.east_m, mission_state.down_m,
                    active_setpoint.heading_ref, active_setpoint.yaw_rate_ref,
                    active_setpoint.surge_ref, active_setpoint.depth_ref,
                    static_cast<unsigned long long>(age_ms));
            }
            else if (config.ros_bridge_enabled())
            {
                std::printf(
                    "[HITL ROS Bridge:%s] state=%s dds=%s fc_state=%s setpoint=%s rx=%llu tx=%llu seq=%u mode=%s\n",
                    config.vehicle.c_str(), state,
                    dds_connected ? "ready" : "waiting",
                    fc_state_fresh ? "fresh" : "waiting",
                    setpoint_fresh ? "fresh" : "waiting",
                    static_cast<unsigned long long>(state_frames_received),
                    static_cast<unsigned long long>(frames_sent),
                    static_cast<unsigned int>(frame.sequence),
                    mode_name(active_mode));
            }
            else
            {
                std::printf(
                    "[HITL Command Simulator] tx=%llu frames / %llu bytes seq=%u state=%s\n",
                    static_cast<unsigned long long>(frames_sent),
                    static_cast<unsigned long long>(bytes_sent),
                    static_cast<unsigned int>(frame.sequence), state);
            }
            next_status_us = now_us + 1'000'000;
        }
    }

    // Revoke command authority before closing instead of relying only on the
    // flight controller's 500 ms command timeout.
    hydrox::GNCSetpoint neutral{};
    for (uint32_t index = 0;
         index < kNeutralFramesOnStop && serial.is_open(); ++index)
    {
        const auto status = send_complete_frame(
            serial, sender, clock, sleeper, frame,
            hydrox::GNCMode::DISABLED, neutral);
        if (!send_succeeded(status))
        {
            success = false;
            break;
        }
        ++frames_sent;
        bytes_sent += hydrox::runtime::kHitlCommandFrameSize;
        sleeper.sleep_for_us(20'000);
    }

    serial.close();
    write_status(config, success ? "stopped" : "io_error", generation,
                 frames_sent, bytes_sent, frame.sequence);
    std::printf("[HITL Command Simulator] stopped after %llu frames\n",
                static_cast<unsigned long long>(frames_sent));
    return success ? 0 : 4;
}
