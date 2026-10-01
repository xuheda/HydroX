#include "hydrox/gcs/mavlink_protocol.h"

#include <common/mavlink.h>

#include <cstdio>
#include <vector>

namespace
{
    struct Capture
    {
        std::vector<std::vector<uint8_t>> packets;
        std::vector<hydrox::gcs::Command> commands;
    };

    bool capture_packet(void *context, const uint8_t *data, std::size_t size)
    {
        auto &capture = *static_cast<Capture *>(context);
        capture.packets.emplace_back(data, data + size);
        return true;
    }

    void capture_command(void *context, const hydrox::gcs::Command &command)
    {
        static_cast<Capture *>(context)->commands.push_back(command);
    }

    bool decode_packet(const std::vector<uint8_t> &packet,
                       mavlink_message_t &message)
    {
        mavlink_message_t parser_buffer{};
        mavlink_status_t parser_status{};
        mavlink_status_t result_status{};
        for (uint8_t byte : packet)
        {
            if (mavlink_frame_char_buffer(
                    &parser_buffer, &parser_status, byte,
                    &message, &result_status) == MAVLINK_FRAMING_OK)
                return true;
        }
        return false;
    }

    std::vector<uint8_t> encode(const mavlink_message_t &message)
    {
        std::vector<uint8_t> bytes(MAVLINK_MAX_PACKET_LEN);
        const uint16_t size = mavlink_msg_to_send_buffer(bytes.data(), &message);
        bytes.resize(size);
        return bytes;
    }

    int expect(bool condition, const char *message)
    {
        if (condition)
            return 0;
        std::fprintf(stderr, "FAIL: %s\n", message);
        return 1;
    }
}

int main()
{
    using namespace hydrox;
    using namespace hydrox::gcs;

    int failures = 0;
    Capture capture;
    MavlinkProtocol protocol(42, 1, MAV_TYPE_SUBMARINE);

    TelemetryState state;
    state.monotonic_time_us = 1;
    state.time_boot_ms = 1234;
    state.mav_type = MAV_TYPE_SUBMARINE;
    state.mode = GNCMode::DEPTH_HOLD;
    state.link_connected = true;
    state.armed = true;
    state.actuator_authorized = false;
    state.ekf_initialized = true;
    state.ground_speed_mps = 17.0f;
    state.equivalent_airspeed_mps = 12.0f;
    protocol.update(state, &capture, capture_packet);

    bool found_air_data = false;
    for (const auto& packet : capture.packets)
    {
        mavlink_message_t message{};
        if (decode_packet(packet, message) && message.msgid == MAVLINK_MSG_ID_VFR_HUD)
        {
            mavlink_vfr_hud_t hud{};
            mavlink_msg_vfr_hud_decode(&message, &hud);
            found_air_data = hud.airspeed == 12.0f && hud.groundspeed == 17.0f;
        }
    }
    failures += expect(found_air_data, "HUD preserves independent airspeed and ground speed");
    bool found_heartbeat = false;
    for (const auto &packet : capture.packets)
    {
        mavlink_message_t message{};
        if (!decode_packet(packet, message) ||
            message.msgid != MAVLINK_MSG_ID_HEARTBEAT)
            continue;
        mavlink_heartbeat_t heartbeat{};
        mavlink_msg_heartbeat_decode(&message, &heartbeat);
        found_heartbeat = heartbeat.autopilot == MAV_AUTOPILOT_GENERIC &&
                          heartbeat.type == MAV_TYPE_SUBMARINE &&
                          heartbeat.custom_mode ==
                              static_cast<uint32_t>(GNCMode::DEPTH_HOLD) &&
                          (heartbeat.base_mode &
                           MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    }
    failures += expect(found_heartbeat,
                       "scheduled heartbeat exposes HydroX identity, mode, and arming");

    mavlink_message_t arm_message{};
    mavlink_msg_command_long_pack(
        255, 190, &arm_message,
        42, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    const std::vector<uint8_t> arm_bytes = encode(arm_message);
    std::vector<uint8_t> corrupt_arm_bytes = arm_bytes;
    // Change the checksum without changing the framing or command payload.
    corrupt_arm_bytes.back() = arm_bytes.back() == 0 ? 1 : 0;
    const auto packets_before_corrupt = capture.packets.size();
    protocol.ingest(corrupt_arm_bytes.data(), corrupt_arm_bytes.size(), 9,
                    &capture, capture_command,
                    &capture, capture_packet);
    failures += expect(
        capture.commands.empty() && protocol.received_message_count() == 0 &&
            capture.packets.size() == packets_before_corrupt,
        "bad CRC produces no command or reply; only valid frames reach the handler");
    protocol.ingest(arm_bytes.data(), arm_bytes.size(), 10,
                    &capture, capture_command,
                    &capture, capture_packet);
    failures += expect(
        capture.commands.size() == 1 &&
            capture.commands.back().kind == CommandKind::SET_ARMED &&
            !capture.commands.back().armed &&
            capture.commands.back().needs_ack,
        "targeted arm/disarm command is emitted as a typed runtime request");

    protocol.acknowledge(capture.commands.back(), CommandResult::ACCEPTED,
                         &capture, capture_packet);
    mavlink_message_t ack_message{};
    failures += expect(decode_packet(capture.packets.back(), ack_message) &&
                           ack_message.msgid == MAVLINK_MSG_ID_COMMAND_ACK,
                       "runtime result is returned as COMMAND_ACK");
    if (ack_message.msgid == MAVLINK_MSG_ID_COMMAND_ACK)
    {
        mavlink_command_ack_t ack{};
        mavlink_msg_command_ack_decode(&ack_message, &ack);
        failures += expect(
            ack.command == MAV_CMD_COMPONENT_ARM_DISARM &&
                ack.result == MAV_RESULT_ACCEPTED &&
                ack.target_system == 255,
            "COMMAND_ACK preserves command and requester identity");
    }

    mavlink_message_t mode_message{};
    mavlink_msg_set_mode_pack(
        255, 190, &mode_message, 42,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        static_cast<uint32_t>(GNCMode::DP));
    const std::vector<uint8_t> mode_bytes = encode(mode_message);
    protocol.ingest(mode_bytes.data(), mode_bytes.size(), 20,
                    &capture, capture_command,
                    &capture, capture_packet);
    failures += expect(
        capture.commands.size() == 2 &&
            capture.commands.back().kind == CommandKind::SET_MODE &&
            capture.commands.back().mode == GNCMode::DP,
        "SET_MODE is target-filtered and decoded to a HydroX mode");

    mavlink_message_t resume_message{};
    mavlink_msg_command_long_pack(255, 190, &resume_message, 42, 1,
        MAV_CMD_DO_PAUSE_CONTINUE, 0, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    const auto resume_bytes = encode(resume_message);
    protocol.ingest(resume_bytes.data(), resume_bytes.size(), 25,
        &capture, capture_command, &capture, capture_packet);
    failures += expect(capture.commands.size() == 3 &&
        capture.commands.back().kind == CommandKind::RESUME_CONTROL && capture.commands.back().needs_ack,
        "explicit continue is a typed takeover request, not a recovered setpoint");

    const std::size_t packets_before_params = capture.packets.size();
    mavlink_message_t param_message{};
    mavlink_msg_param_request_list_pack(
        255, 190, &param_message, 42, 1);
    const std::vector<uint8_t> param_bytes = encode(param_message);
    protocol.ingest(param_bytes.data(), param_bytes.size(), 30,
                    &capture, capture_command,
                    &capture, capture_packet);
    failures += expect(
        capture.packets.size() == packets_before_params + 3,
        "parameter discovery returns a finite list and can complete in XGC");

    if (failures == 0)
        std::puts("test_gcs_mavlink: all checks passed");
    return failures == 0 ? 0 : 1;
}
