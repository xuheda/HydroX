#include "mavlink_hil.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace
{
int expect(bool condition, const char *message)
{
    if (condition)
        return 0;
    std::fprintf(stderr, "FAIL: %s\n", message);
    return 1;
}

hydrox::HilProfileMsg profile_message(hydrox::HilProfileOperation operation)
{
    hydrox::HilProfileMsg message;
    message.fingerprint = 0xB85225A6148764ACULL;
    message.nonce = 0x71625344U;
    message.operation = operation;
    message.mav_type = 19;
    constexpr char kProfileId[] = "StandardVTOL";
    std::memcpy(message.profile_id.data(), kProfileId, sizeof(kProfileId));
    message.valid = true;
    return message;
}
} // namespace

int main()
{
    int failures = 0;
    hydrox::MavlinkHIL transmitter(42, 191);
    hydrox::MavlinkHIL receiver(1, 1);

    for (const auto operation : {
             hydrox::HilProfileOperation::Select,
             hydrox::HilProfileOperation::Ready,
             hydrox::HilProfileOperation::Rejected})
    {
        const hydrox::HilProfileMsg outbound = profile_message(operation);
        hydrox::MavlinkPacket packet;
        failures += expect(
            transmitter.encode_hil_profile(packet, outbound),
            "valid profile message encodes");
        failures += expect(
            packet.size() == 10U + hydrox::HIL_PROFILE_PAYLOAD_LEN + 2U,
            "profile packet has exact MAVLink 2 length");

        const auto frames = receiver.feed(packet.data(), packet.size());
        failures += expect(
            frames.size() == 1 &&
                frames[0].msg_id == hydrox::MSGID_HIL_PROFILE,
            "profile packet passes CRC and decodes once");
        if (frames.size() != 1)
            continue;

        const hydrox::HilProfileMsg inbound =
            receiver.parse_hil_profile(frames[0]);
        failures += expect(inbound.valid, "decoded profile message is valid");
        failures += expect(
            inbound.operation == operation &&
                inbound.fingerprint == outbound.fingerprint &&
                inbound.nonce == outbound.nonce &&
                inbound.mav_type == outbound.mav_type &&
                std::strcmp(inbound.profile_id.data(),
                            outbound.profile_id.data()) == 0,
            "profile identity, fingerprint, nonce, operation and MAV type round-trip");

        hydrox::MavFrame malformed_operation = frames[0];
        malformed_operation.payload[12] = 0xFF;
        failures += expect(
            !receiver.parse_hil_profile(malformed_operation).valid,
            "unknown profile operation is rejected");

        hydrox::MavFrame unterminated_id = frames[0];
        std::fill_n(
            unterminated_id.payload.data() + 14,
            hydrox::HIL_PROFILE_ID_LEN,
            static_cast<uint8_t>('X'));
        failures += expect(
            !receiver.parse_hil_profile(unterminated_id).valid,
            "unterminated profile identity is rejected");
    }

    hydrox::HilProfileMsg invalid = profile_message(
        hydrox::HilProfileOperation::Select);
    hydrox::MavlinkPacket packet;
    invalid.nonce = 0;
    failures += expect(
        !transmitter.encode_hil_profile(packet, invalid) && packet.empty(),
        "zero selection nonce cannot be encoded");
    invalid = profile_message(hydrox::HilProfileOperation::Select);
    invalid.profile_id.fill('X');
    failures += expect(
        !transmitter.encode_hil_profile(packet, invalid) && packet.empty(),
        "unterminated profile identity cannot be encoded");

    hydrox::MavFrame short_payload;
    short_payload.msg_id = hydrox::MSGID_HIL_PROFILE;
    short_payload.payload.resize(hydrox::HIL_PROFILE_PAYLOAD_LEN - 1U);
    failures += expect(
        !receiver.parse_hil_profile(short_payload).valid,
        "wrong profile payload length is rejected");

    if (failures == 0)
        std::printf("test_hitl_profile_protocol: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
