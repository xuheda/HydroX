#include "hydrox/gcs/mavlink_protocol.h"

#include <common/mavlink.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace hydrox::gcs
{
namespace
{
    constexpr uint32_t kDefaultFastIntervalUs = 250'000;
    constexpr uint32_t kDefaultSlowIntervalUs = 1'000'000;

    template<typename T>
    T clamp_cast(double value)
    {
        const double minimum = static_cast<double>(std::numeric_limits<T>::min());
        const double maximum = static_cast<double>(std::numeric_limits<T>::max());
        return static_cast<T>(std::clamp(value, minimum, maximum));
    }

    bool valid_mode(uint32_t raw, GNCMode &mode)
    {
        if (raw > static_cast<uint32_t>(GNCMode::SURFACE))
            return false;
        mode = static_cast<GNCMode>(raw);
        return true;
    }

    float normalize_heading_deg(float yaw_rad)
    {
        constexpr float radians_to_degrees = 57.29577951308232f;
        float heading = std::fmod(yaw_rad * radians_to_degrees, 360.0f);
        if (heading < 0.0f)
            heading += 360.0f;
        return heading;
    }
}

struct MavlinkProtocol::Impl
{
    struct Stream
    {
        uint32_t message_id = 0;
        int64_t interval_us = 0;
        int64_t default_interval_us = 0;
        uint64_t last_sent_us = 0;
    };

    struct Parameter
    {
        std::array<char, 16> id{};
        float value = 0.0f;
        bool writable = false;
    };

    Impl(uint8_t sysid, uint8_t compid, uint8_t type)
        : system_id(sysid), component_id(compid), mav_type(type)
    {
        streams = {{
            {MAVLINK_MSG_ID_HEARTBEAT, kDefaultSlowIntervalUs,
             kDefaultSlowIntervalUs, 0},
            {MAVLINK_MSG_ID_SYS_STATUS, kDefaultSlowIntervalUs,
             kDefaultSlowIntervalUs, 0},
            {MAVLINK_MSG_ID_ATTITUDE, kDefaultFastIntervalUs,
             kDefaultFastIntervalUs, 0},
            {MAVLINK_MSG_ID_LOCAL_POSITION_NED, kDefaultFastIntervalUs,
             kDefaultFastIntervalUs, 0},
            {MAVLINK_MSG_ID_GLOBAL_POSITION_INT, kDefaultFastIntervalUs,
             kDefaultFastIntervalUs, 0},
            {MAVLINK_MSG_ID_VFR_HUD, kDefaultFastIntervalUs,
             kDefaultFastIntervalUs, 0},
        }};

        set_parameter(parameters[0], "HYX_SYS_ID", static_cast<float>(sysid), false);
        set_parameter(parameters[1], "HYX_VEH_TYPE", static_cast<float>(type), false);
        set_parameter(parameters[2], "HYX_GCS_RATE", 4.0f, true);
    }

    static void set_parameter(Parameter &parameter,
                              const char *id,
                              float value,
                              bool writable)
    {
        std::memset(parameter.id.data(), 0, parameter.id.size());
        if (id != nullptr)
        {
            std::memcpy(parameter.id.data(), id,
                        std::min(std::strlen(id), parameter.id.size()));
        }
        parameter.value = value;
        parameter.writable = writable;
    }

    bool targets_this_vehicle(uint8_t target_system,
                              uint8_t target_component) const
    {
        return (target_system == 0 || target_system == system_id) &&
               (target_component == 0 || target_component == component_id);
    }

    template<typename Payload, typename Encoder>
    bool send_payload(const Payload &payload,
                      Encoder encoder,
                      void *context,
                      PacketSink sink)
    {
        if (sink == nullptr)
            return false;
        mavlink_message_t message{};
        encoder(system_id, component_id, &tx_status, &message, &payload);
        std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
        const uint16_t length = mavlink_msg_to_send_buffer(bytes.data(), &message);
        return length > 0 && sink(context, bytes.data(), length);
    }

    bool send_heartbeat(const TelemetryState &state,
                        void *context,
                        PacketSink sink)
    {
        mavlink_heartbeat_t heartbeat{};
        heartbeat.custom_mode = static_cast<uint32_t>(state.mode);
        heartbeat.type = state.mav_type;
        heartbeat.autopilot = MAV_AUTOPILOT_GENERIC;
        heartbeat.base_mode = static_cast<uint8_t>(
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_HIL_ENABLED |
            (state.mode != GNCMode::DISABLED ? MAV_MODE_FLAG_GUIDED_ENABLED : 0) |
            (state.armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0));
        heartbeat.system_status = !state.link_connected
                                      ? MAV_STATE_CRITICAL
                                      : (state.armed ? MAV_STATE_ACTIVE
                                                     : MAV_STATE_STANDBY);
        heartbeat.mavlink_version = 3;
        return send_payload(
            heartbeat, mavlink_msg_heartbeat_encode_status, context, sink);
    }

    bool send_sys_status(const TelemetryState &state,
                         void *context,
                         PacketSink sink)
    {
        constexpr uint32_t attitude = MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                      MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                      MAV_SYS_STATUS_SENSOR_3D_MAG;
        constexpr uint32_t navigation = MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                        MAV_SYS_STATUS_SENSOR_GPS |
                                        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                        MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                        MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        mavlink_sys_status_t status{};
        status.onboard_control_sensors_present = attitude | navigation;
        status.onboard_control_sensors_enabled = attitude | navigation;
        status.onboard_control_sensors_health = attitude |
            (state.ekf_initialized
                 ? (MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL)
                 : 0) |
            (state.gps_valid ? MAV_SYS_STATUS_SENSOR_GPS : 0) |
            MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        status.load = 0;
        status.voltage_battery = UINT16_MAX;
        status.current_battery = -1;
        status.battery_remaining = state.battery_remaining_pct < 0.0f
                                       ? -1
                                       : clamp_cast<int8_t>(
                                             state.battery_remaining_pct);
        return send_payload(
            status, mavlink_msg_sys_status_encode_status, context, sink);
    }

    bool send_attitude(const TelemetryState &state,
                       void *context,
                       PacketSink sink)
    {
        mavlink_attitude_t attitude{};
        attitude.time_boot_ms = state.time_boot_ms;
        attitude.roll = state.roll_rad;
        attitude.pitch = state.pitch_rad;
        attitude.yaw = state.yaw_rad;
        attitude.rollspeed = state.roll_rate_radps;
        attitude.pitchspeed = state.pitch_rate_radps;
        attitude.yawspeed = state.yaw_rate_radps;
        return send_payload(
            attitude, mavlink_msg_attitude_encode_status, context, sink);
    }

    bool send_local_position(const TelemetryState &state,
                             void *context,
                             PacketSink sink)
    {
        mavlink_local_position_ned_t position{};
        position.time_boot_ms = state.time_boot_ms;
        position.x = state.position_n_m;
        position.y = state.position_e_m;
        position.z = state.position_d_m;
        position.vx = state.velocity_n_mps;
        position.vy = state.velocity_e_mps;
        position.vz = state.velocity_d_mps;
        return send_payload(position,
                            mavlink_msg_local_position_ned_encode_status,
                            context, sink);
    }

    bool send_global_position(const TelemetryState &state,
                              void *context,
                              PacketSink sink)
    {
        if (!state.gps_valid)
            return false;
        mavlink_global_position_int_t position{};
        position.time_boot_ms = state.time_boot_ms;
        position.lat = state.latitude_deg7;
        position.lon = state.longitude_deg7;
        position.alt = state.altitude_msl_mm;
        position.relative_alt = clamp_cast<int32_t>(
            -static_cast<double>(state.position_d_m) * 1000.0);
        position.vx = clamp_cast<int16_t>(
            static_cast<double>(state.velocity_n_mps) * 100.0);
        position.vy = clamp_cast<int16_t>(
            static_cast<double>(state.velocity_e_mps) * 100.0);
        position.vz = clamp_cast<int16_t>(
            static_cast<double>(state.velocity_d_mps) * 100.0);
        position.hdg = clamp_cast<uint16_t>(
            static_cast<double>(normalize_heading_deg(state.yaw_rad)) * 100.0);
        return send_payload(position,
                            mavlink_msg_global_position_int_encode_status,
                            context, sink);
    }

    bool send_vfr_hud(const TelemetryState &state,
                      void *context,
                      PacketSink sink)
    {
        mavlink_vfr_hud_t hud{};
        hud.airspeed = state.equivalent_airspeed_mps;
        hud.groundspeed = state.ground_speed_mps;
        hud.heading = clamp_cast<int16_t>(normalize_heading_deg(state.yaw_rad));
        hud.throttle = static_cast<uint16_t>(
            std::min<uint16_t>(state.throttle_pct, 100));
        hud.alt = -state.position_d_m;
        hud.climb = -state.velocity_d_mps;
        return send_payload(hud, mavlink_msg_vfr_hud_encode_status,
                            context, sink);
    }

    bool send_autopilot_version(void *context, PacketSink sink)
    {
        mavlink_autopilot_version_t version{};
        version.capabilities = MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                               MAV_PROTOCOL_CAPABILITY_MAVLINK2;
        return send_payload(version,
                            mavlink_msg_autopilot_version_encode_status,
                            context, sink);
    }

    bool send_parameter(std::size_t index,
                        void *context,
                        PacketSink sink)
    {
        if (index >= parameters.size())
            return false;
        mavlink_param_value_t value{};
        value.param_value = parameters[index].value;
        value.param_count = static_cast<uint16_t>(parameters.size());
        value.param_index = static_cast<uint16_t>(index);
        value.param_type = MAV_PARAM_TYPE_REAL32;
        std::memcpy(value.param_id, parameters[index].id.data(),
                    parameters[index].id.size());
        return send_payload(value, mavlink_msg_param_value_encode_status,
                            context, sink);
    }

    int find_parameter(const char *id) const
    {
        if (id == nullptr)
            return -1;
        for (std::size_t i = 0; i < parameters.size(); ++i)
        {
            if (std::memcmp(parameters[i].id.data(), id,
                            parameters[i].id.size()) == 0)
                return static_cast<int>(i);
        }
        return -1;
    }

    Stream *find_stream(uint32_t message_id)
    {
        for (Stream &stream : streams)
        {
            if (stream.message_id == message_id)
                return &stream;
        }
        return nullptr;
    }

    bool set_message_interval(uint32_t message_id, int64_t interval_us)
    {
        Stream *stream = find_stream(message_id);
        if (stream == nullptr)
            return false;
        stream->interval_us = interval_us == 0
                                  ? stream->default_interval_us
                                  : interval_us;
        stream->last_sent_us = 0;
        return true;
    }

    bool send_requested_message(uint32_t message_id,
                                const TelemetryState *state,
                                void *context,
                                PacketSink sink)
    {
        if (message_id == MAVLINK_MSG_ID_AUTOPILOT_VERSION)
            return send_autopilot_version(context, sink);
        if (state == nullptr)
            return false;
        switch (message_id)
        {
        case MAVLINK_MSG_ID_HEARTBEAT:
            return send_heartbeat(*state, context, sink);
        case MAVLINK_MSG_ID_SYS_STATUS:
            return send_sys_status(*state, context, sink);
        case MAVLINK_MSG_ID_ATTITUDE:
            return send_attitude(*state, context, sink);
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            return send_local_position(*state, context, sink);
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            return send_global_position(*state, context, sink);
        case MAVLINK_MSG_ID_VFR_HUD:
            return send_vfr_hud(*state, context, sink);
        default:
            return false;
        }
    }

    bool send_ack(uint16_t command,
                  uint8_t result,
                  uint8_t target_system,
                  uint8_t target_component,
                  void *context,
                  PacketSink sink)
    {
        mavlink_command_ack_t ack{};
        ack.command = command;
        ack.result = result;
        ack.progress = UINT8_MAX;
        ack.target_system = target_system;
        ack.target_component = target_component;
        return send_payload(ack, mavlink_msg_command_ack_encode_status,
                            context, sink);
    }

    void emit_command(const Command &command,
                      void *command_context,
                      CommandSink command_sink,
                      void *packet_context,
                      PacketSink packet_sink)
    {
        if (command_sink != nullptr)
        {
            command_sink(command_context, command);
            return;
        }
        if (command.needs_ack)
        {
            send_ack(command.mav_command, MAV_RESULT_TEMPORARILY_REJECTED,
                     command.source_system, command.source_component,
                     packet_context, packet_sink);
        }
    }

    void handle_command_long(const mavlink_message_t &message,
                             void *command_context,
                             CommandSink command_sink,
                             void *packet_context,
                             PacketSink packet_sink)
    {
        mavlink_command_long_t request{};
        mavlink_msg_command_long_decode(&message, &request);
        if (!targets_this_vehicle(request.target_system,
                                  request.target_component))
            return;

        if (request.command == MAV_CMD_COMPONENT_ARM_DISARM)
        {
            Command command{};
            command.kind = CommandKind::SET_ARMED;
            command.armed = request.param1 >= 0.5f;
            command.mav_command = request.command;
            command.source_system = message.sysid;
            command.source_component = message.compid;
            command.needs_ack = true;
            emit_command(command, command_context, command_sink,
                         packet_context, packet_sink);
            return;
        }

        if (request.command == MAV_CMD_DO_PAUSE_CONTINUE && request.param1 == 1.0f)
        {
            Command command{};
            command.kind = CommandKind::RESUME_CONTROL;
            command.mav_command = request.command;
            command.source_system = message.sysid;
            command.source_component = message.compid;
            command.needs_ack = true;
            emit_command(command, command_context, command_sink, packet_context, packet_sink);
            return;
        }

        if (request.command == MAV_CMD_DO_SET_MODE)
        {
            GNCMode mode{};
            if (!valid_mode(static_cast<uint32_t>(request.param2), mode))
            {
                send_ack(request.command, MAV_RESULT_DENIED,
                         message.sysid, message.compid,
                         packet_context, packet_sink);
                return;
            }
            Command command{};
            command.kind = CommandKind::SET_MODE;
            command.mode = mode;
            command.mav_command = request.command;
            command.source_system = message.sysid;
            command.source_component = message.compid;
            command.needs_ack = true;
            emit_command(command, command_context, command_sink,
                         packet_context, packet_sink);
            return;
        }

        if (request.command == MAV_CMD_SET_MESSAGE_INTERVAL)
        {
            const bool accepted = set_message_interval(
                static_cast<uint32_t>(request.param1),
                static_cast<int64_t>(request.param2));
            send_ack(request.command,
                     accepted ? MAV_RESULT_ACCEPTED : MAV_RESULT_UNSUPPORTED,
                     message.sysid, message.compid,
                     packet_context, packet_sink);
            return;
        }

        if (request.command == MAV_CMD_REQUEST_MESSAGE)
        {
            const uint32_t requested_id = static_cast<uint32_t>(request.param1);
            const bool sent = send_requested_message(
                requested_id, last_state_valid ? &last_state : nullptr,
                packet_context, packet_sink);
            send_ack(request.command,
                     sent ? MAV_RESULT_ACCEPTED : MAV_RESULT_UNSUPPORTED,
                     message.sysid, message.compid,
                     packet_context, packet_sink);
            return;
        }

        send_ack(request.command, MAV_RESULT_UNSUPPORTED,
                 message.sysid, message.compid,
                 packet_context, packet_sink);
    }

    void handle_set_mode(const mavlink_message_t &message,
                         void *command_context,
                         CommandSink command_sink,
                         void *packet_context,
                         PacketSink packet_sink)
    {
        mavlink_set_mode_t request{};
        mavlink_msg_set_mode_decode(&message, &request);
        if (request.target_system != 0 && request.target_system != system_id)
            return;
        GNCMode mode{};
        if (!valid_mode(request.custom_mode, mode))
            return;
        Command command{};
        command.kind = CommandKind::SET_MODE;
        command.mode = mode;
        command.source_system = message.sysid;
        command.source_component = message.compid;
        command.needs_ack = false;
        emit_command(command, command_context, command_sink,
                     packet_context, packet_sink);
    }

    void handle_ping(const mavlink_message_t &message,
                     void *packet_context,
                     PacketSink packet_sink)
    {
        mavlink_ping_t request{};
        mavlink_msg_ping_decode(&message, &request);
        if (!targets_this_vehicle(request.target_system,
                                  request.target_component))
            return;
        mavlink_ping_t response{};
        response.time_usec = request.time_usec;
        response.seq = request.seq;
        response.target_system = message.sysid;
        response.target_component = message.compid;
        send_payload(response, mavlink_msg_ping_encode_status,
                     packet_context, packet_sink);
    }

    void handle_timesync(const mavlink_message_t &message,
                         uint64_t now_us,
                         void *packet_context,
                         PacketSink packet_sink)
    {
        mavlink_timesync_t request{};
        mavlink_msg_timesync_decode(&message, &request);
        if (request.tc1 != 0)
            return;
        mavlink_timesync_t response{};
        response.tc1 = static_cast<int64_t>(now_us * 1000ULL);
        response.ts1 = request.ts1;
        response.target_system = message.sysid;
        response.target_component = message.compid;
        send_payload(response, mavlink_msg_timesync_encode_status,
                     packet_context, packet_sink);
    }

    void handle_param_request_list(const mavlink_message_t &message,
                                   void *packet_context,
                                   PacketSink packet_sink)
    {
        mavlink_param_request_list_t request{};
        mavlink_msg_param_request_list_decode(&message, &request);
        if (!targets_this_vehicle(request.target_system,
                                  request.target_component))
            return;
        for (std::size_t i = 0; i < parameters.size(); ++i)
            send_parameter(i, packet_context, packet_sink);
    }

    void handle_param_request_read(const mavlink_message_t &message,
                                   void *packet_context,
                                   PacketSink packet_sink)
    {
        mavlink_param_request_read_t request{};
        mavlink_msg_param_request_read_decode(&message, &request);
        if (!targets_this_vehicle(request.target_system,
                                  request.target_component))
            return;
        int index = request.param_index;
        if (index < 0)
            index = find_parameter(request.param_id);
        if (index >= 0)
            send_parameter(static_cast<std::size_t>(index),
                           packet_context, packet_sink);
    }

    void handle_param_set(const mavlink_message_t &message,
                          void *packet_context,
                          PacketSink packet_sink)
    {
        mavlink_param_set_t request{};
        mavlink_msg_param_set_decode(&message, &request);
        if (!targets_this_vehicle(request.target_system,
                                  request.target_component))
            return;
        const int index = find_parameter(request.param_id);
        if (index < 0)
            return;
        Parameter &parameter = parameters[static_cast<std::size_t>(index)];
        if (parameter.writable &&
            std::strcmp(parameter.id.data(), "HYX_GCS_RATE") == 0 &&
            std::isfinite(request.param_value))
        {
            parameter.value = std::clamp(request.param_value, 1.0f, 20.0f);
            const int64_t interval = static_cast<int64_t>(
                1'000'000.0f / parameter.value);
            for (Stream &stream : streams)
            {
                if (stream.message_id != MAVLINK_MSG_ID_HEARTBEAT &&
                    stream.message_id != MAVLINK_MSG_ID_SYS_STATUS)
                {
                    stream.default_interval_us = interval;
                    stream.interval_us = interval;
                    stream.last_sent_us = 0;
                }
            }
        }
        // PARAM_VALUE is the Parameter Protocol acknowledgement. Read-only
        // parameters are echoed unchanged.
        send_parameter(static_cast<std::size_t>(index),
                       packet_context, packet_sink);
    }

    void handle_message(const mavlink_message_t &message,
                        uint64_t now_us,
                        void *command_context,
                        CommandSink command_sink,
                        void *packet_context,
                        PacketSink packet_sink)
    {
        ++received_messages;
        switch (message.msgid)
        {
        case MAVLINK_MSG_ID_COMMAND_LONG:
            handle_command_long(message, command_context, command_sink,
                                packet_context, packet_sink);
            break;
        case MAVLINK_MSG_ID_SET_MODE:
            handle_set_mode(message, command_context, command_sink,
                            packet_context, packet_sink);
            break;
        case MAVLINK_MSG_ID_PING:
            handle_ping(message, packet_context, packet_sink);
            break;
        case MAVLINK_MSG_ID_TIMESYNC:
            handle_timesync(message, now_us, packet_context, packet_sink);
            break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            handle_param_request_list(message, packet_context, packet_sink);
            break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
            handle_param_request_read(message, packet_context, packet_sink);
            break;
        case MAVLINK_MSG_ID_PARAM_SET:
            handle_param_set(message, packet_context, packet_sink);
            break;
        default:
            break;
        }
    }

    bool stream_due(Stream &stream, uint64_t now_us)
    {
        if (stream.interval_us < 0)
            return false;
        if (stream.last_sent_us == 0 || now_us < stream.last_sent_us ||
            now_us - stream.last_sent_us >=
                static_cast<uint64_t>(stream.interval_us))
        {
            stream.last_sent_us = now_us;
            return true;
        }
        return false;
    }

    uint8_t system_id = 1;
    uint8_t component_id = 1;
    uint8_t mav_type = 12;
    mavlink_message_t rx_buffer{};
    mavlink_status_t rx_status{};
    mavlink_status_t tx_status{};
    std::array<Stream, 6> streams{};
    std::array<Parameter, 3> parameters{};
    TelemetryState last_state{};
    bool last_state_valid = false;
    uint64_t received_messages = 0;
};

MavlinkProtocol::MavlinkProtocol(uint8_t system_id,
                                 uint8_t component_id,
                                 uint8_t mav_type)
    : impl_(std::make_unique<Impl>(system_id, component_id, mav_type))
{
}

MavlinkProtocol::~MavlinkProtocol() = default;

void MavlinkProtocol::ingest(const uint8_t *data,
                             std::size_t size,
                             uint64_t now_us,
                             void *command_context,
                             CommandSink command_sink,
                             void *packet_context,
                             PacketSink packet_sink)
{
    if (data == nullptr)
        return;
    for (std::size_t i = 0; i < size; ++i)
    {
        mavlink_message_t message{};
        mavlink_status_t status{};
        const uint8_t framing = mavlink_frame_char_buffer(
            &impl_->rx_buffer, &impl_->rx_status, data[i], &message, &status);
        if (framing == MAVLINK_FRAMING_OK)
        {
            impl_->handle_message(message, now_us,
                                  command_context, command_sink,
                                  packet_context, packet_sink);
        }
    }
}

void MavlinkProtocol::update(const TelemetryState &state,
                             void *packet_context,
                             PacketSink packet_sink)
{
    impl_->last_state = state;
    impl_->last_state_valid = true;
    for (Impl::Stream &stream : impl_->streams)
    {
        if (!impl_->stream_due(stream, state.monotonic_time_us))
            continue;
        impl_->send_requested_message(stream.message_id, &state,
                                      packet_context, packet_sink);
    }
}

bool MavlinkProtocol::acknowledge(const Command &command,
                                  CommandResult result,
                                  void *packet_context,
                                  PacketSink packet_sink)
{
    if (!command.needs_ack)
        return true;
    return impl_->send_ack(command.mav_command,
                           static_cast<uint8_t>(result),
                           command.source_system,
                           command.source_component,
                           packet_context, packet_sink);
}

bool MavlinkProtocol::send_statustext(uint8_t severity,
                                      const char *text,
                                      void *packet_context,
                                      PacketSink packet_sink)
{
    mavlink_statustext_t status{};
    status.severity = severity;
    if (text != nullptr)
    {
        std::memcpy(status.text, text,
                    std::min(std::strlen(text), sizeof(status.text)));
    }
    return impl_->send_payload(status,
                               mavlink_msg_statustext_encode_status,
                               packet_context, packet_sink);
}

uint64_t MavlinkProtocol::received_message_count() const noexcept
{
    return impl_->received_messages;
}

} // namespace hydrox::gcs
