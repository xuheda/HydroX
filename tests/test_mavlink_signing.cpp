#include "mavlink_hil.h"

#include <array>
#include <cstdio>

namespace
{
    int expect(bool condition, const char *message)
    {
        if (condition)
            return 0;
        std::fprintf(stderr, "FAIL: %s\n", message);
        return 1;
    }

    hydrox::MavlinkSigningConfig signed_config(uint8_t link_id = 17)
    {
        hydrox::MavlinkSigningConfig config;
        config.enabled = true;
        config.sign_outgoing = true;
        config.require_incoming = true;
        config.link_id = link_id;
        for (size_t i = 0; i < config.secret_key.size(); ++i)
            config.secret_key[i] = static_cast<uint8_t>(0xA0u + i);
        return config;
    }
}

int main()
{
    int failures = 0;

    const hydrox::MavlinkSigningConfig config = signed_config();
    hydrox::MavlinkHIL transmitter(1, 1, config);
    hydrox::MavlinkHIL receiver(1, 1, config);

    const std::vector<uint8_t> signed_heartbeat = transmitter.encode_heartbeat();
    failures += expect(
        signed_heartbeat.size() == 9u + 12u + hydrox::MAVLINK_SIGNATURE_BLOCK_LEN &&
            (signed_heartbeat[2] & hydrox::MAVLINK_IFLAG_SIGNED) != 0,
        "outgoing MAVLink 2 heartbeat carries the signing flag and signature block");

    const auto decoded = receiver.feed(
        signed_heartbeat.data(), signed_heartbeat.size());
    failures += expect(
        decoded.size() == 1 && decoded[0].msg_id == hydrox::MSGID_HEARTBEAT &&
            decoded[0].signed_frame,
        "valid signed MAVLink frame is accepted");

    const auto replay = receiver.feed(
        signed_heartbeat.data(), signed_heartbeat.size());
    failures += expect(replay.empty(), "signed MAVLink frame replay is rejected");

    std::vector<uint8_t> tampered = transmitter.encode_heartbeat();
    tampered.back() ^= 0x80u;
    const auto tampered_result = receiver.feed(tampered.data(), tampered.size());
    failures += expect(tampered_result.empty(), "tampered signature is rejected");

    hydrox::MavlinkHIL unsigned_transmitter;
    const std::vector<uint8_t> unsigned_heartbeat = unsigned_transmitter.encode_heartbeat();
    const auto unsigned_result = receiver.feed(
        unsigned_heartbeat.data(), unsigned_heartbeat.size());
    failures += expect(unsigned_result.empty(), "secure endpoint rejects unsigned MAVLink frames");

    hydrox::MavlinkHIL wrong_link_receiver(1, 1, signed_config(18));
    const auto wrong_link = wrong_link_receiver.feed(
        signed_heartbeat.data(), signed_heartbeat.size());
    failures += expect(wrong_link.empty(), "secure endpoint rejects an unexpected MAVLink link id");

    if (failures == 0)
        std::printf("test_mavlink_signing: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
