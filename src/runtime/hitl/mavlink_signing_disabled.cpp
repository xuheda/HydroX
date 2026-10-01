// HITL firmware policy: signing stays disabled until embedded key storage exists.
#include "mavlink_signing.h"

namespace hydrox
{
    bool load_mavlink_signing_key_file(
        const std::string &,
        std::array<uint8_t, MAVLINK_SIGNING_KEY_LEN> &out_key,
        std::string *out_error)
    {
        out_key.fill(0);
        if (out_error != nullptr)
            *out_error = "MAVLink signing is disabled in hydrox_pixhawk6c_hitl";
        return false;
    }

    uint64_t mavlink_signing_timestamp_10us()
    {
        return 0;
    }

    bool mavlink_signing_digest(
        const std::array<uint8_t, MAVLINK_SIGNING_KEY_LEN> &,
        const uint8_t *,
        size_t,
        uint8_t,
        uint64_t,
        std::array<uint8_t, 32> &out_digest)
    {
        out_digest.fill(0);
        return false;
    }

    bool mavlink_signature_equal_48(const uint8_t *lhs, const uint8_t *rhs)
    {
        if (lhs == nullptr || rhs == nullptr)
            return false;

        uint8_t difference = 0;
        for (size_t i = 0; i < 6; ++i)
            difference |= static_cast<uint8_t>(lhs[i] ^ rhs[i]);
        return difference == 0;
    }
} // namespace hydrox
