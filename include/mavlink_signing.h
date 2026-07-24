#pragma once

/**
 * MAVLink 2 packet-signing primitives.
 *
 * Implements the MAVLink signing format rather than a transport-specific
 * authentication scheme: a signed packet sets incompat_flags bit 0 and appends
 * link_id(1), timestamp(6), signature(6).  The signature is the first six
 * bytes of SHA-256(secret_key + packet_without_magic + link_id + timestamp).
 */
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace hydrox
{
    constexpr size_t MAVLINK_SIGNING_KEY_LEN = 32;
    constexpr size_t MAVLINK_SIGNATURE_BLOCK_LEN = 13;
    constexpr uint8_t MAVLINK_IFLAG_SIGNED = 0x01;

    struct MavlinkSigningConfig
    {
        bool enabled = false;
        bool sign_outgoing = false;
        bool require_incoming = false;
        uint8_t link_id = 0;
        std::array<uint8_t, MAVLINK_SIGNING_KEY_LEN> secret_key{};

        bool valid() const
        {
            if (!enabled)
                return false;
            for (const uint8_t byte : secret_key)
            {
                if (byte != 0)
                    return true;
            }
            return false;
        }
    };

    /** Load exactly 32 secret bytes from a 64-hex-character text file. */
    bool load_mavlink_signing_key_file(
        const std::string &path,
        std::array<uint8_t, MAVLINK_SIGNING_KEY_LEN> &out_key,
        std::string *out_error = nullptr);

    /** MAVLink signing timestamp: 10 us ticks since 2015-01-01 UTC. */
    uint64_t mavlink_signing_timestamp_10us();

    /** SHA-256 digest used by the MAVLink signing specification. */
    bool mavlink_signing_digest(
        const std::array<uint8_t, MAVLINK_SIGNING_KEY_LEN> &key,
        const uint8_t *packet_without_magic,
        size_t packet_len,
        uint8_t link_id,
        uint64_t timestamp_10us,
        std::array<uint8_t, 32> &out_digest);

    /** Constant-time comparison for the six-byte on-wire signature. */
    bool mavlink_signature_equal_48(const uint8_t *lhs, const uint8_t *rhs);

} // namespace hydrox
