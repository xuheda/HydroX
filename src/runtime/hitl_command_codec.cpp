#include "hydrox/runtime/hitl_command_codec.h"
#include <cmath>
#include <cstring>

namespace hydrox::runtime
{
namespace
{
    constexpr uint8_t kMagic[4] = {'H', 'X', 'S', 'P'};
    constexpr uint8_t kVersion = 1;
    constexpr uint8_t kPayloadSize = 44;

    uint16_t crc16_x25(const uint8_t *data, std::size_t length) noexcept
    {
        uint16_t crc = 0xffff;
        for (std::size_t i = 0; i < length; ++i)
        {
            uint8_t tmp = data[i] ^ static_cast<uint8_t>(crc);
            tmp ^= static_cast<uint8_t>(tmp << 4);
            crc = static_cast<uint16_t>((crc >> 8) ^
                  (static_cast<uint16_t>(tmp) << 8) ^
                  (static_cast<uint16_t>(tmp) << 3) ^
                  (static_cast<uint16_t>(tmp) >> 4));
        }
        return crc;
    }

    uint16_t get_u16(const uint8_t *p) noexcept
    {
        return static_cast<uint16_t>(p[0]) |
               (static_cast<uint16_t>(p[1]) << 8);
    }

    uint32_t get_u32(const uint8_t *p) noexcept
    {
        return static_cast<uint32_t>(p[0]) |
               (static_cast<uint32_t>(p[1]) << 8) |
               (static_cast<uint32_t>(p[2]) << 16) |
               (static_cast<uint32_t>(p[3]) << 24);
    }

    uint64_t get_u64(const uint8_t *p) noexcept
    {
        uint64_t value = 0;
        for (unsigned int i = 0; i < 8; ++i)
            value |= static_cast<uint64_t>(p[i]) << (8 * i);
        return value;
    }

    float get_f32(const uint8_t *p) noexcept
    {
        const uint32_t bits = get_u32(p);
        float value = 0.0f;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    void put_u16(uint8_t *p, uint16_t value) noexcept
    {
        p[0] = static_cast<uint8_t>(value);
        p[1] = static_cast<uint8_t>(value >> 8);
    }

    void put_u32(uint8_t *p, uint32_t value) noexcept
    {
        for (unsigned int i = 0; i < 4; ++i)
            p[i] = static_cast<uint8_t>(value >> (8 * i));
    }

    void put_u64(uint8_t *p, uint64_t value) noexcept
    {
        for (unsigned int i = 0; i < 8; ++i)
            p[i] = static_cast<uint8_t>(value >> (8 * i));
    }

    void put_f32(uint8_t *p, double value) noexcept
    {
        const float f = static_cast<float>(value);
        uint32_t bits = 0;
        std::memcpy(&bits, &f, sizeof(bits));
        put_u32(p, bits);
    }

    bool finite_and_bounded(double value, double maximum) noexcept
    {
        return std::isfinite(value) && std::abs(value) <= maximum;
    }
}

void HitlCommandDecoder::discard_first() noexcept
{
    if (used_ > 1)
        std::memmove(buffer_.data(), buffer_.data() + 1, used_ - 1);
    if (used_ > 0)
        --used_;
}

bool HitlCommandDecoder::decode(HitlCommandFrame &frame) const noexcept
{
    if (used_ != buffer_.size() ||
        std::memcmp(buffer_.data(), kMagic, sizeof(kMagic)) != 0 ||
        buffer_[4] != kVersion || buffer_[5] != kPayloadSize ||
        get_u16(buffer_.data() + 52) !=
            crc16_x25(buffer_.data(), buffer_.size() - 2))
        return false;

    const uint8_t mode = buffer_[20];
    const uint8_t flags = buffer_[21];
    const uint32_t sender_generation = get_u32(buffer_.data() + 8);
    if (mode > static_cast<uint8_t>(GNCMode::SURFACE) ||
        (flags & ~uint8_t{1}) != 0 || sender_generation == 0)
        return false;

    HitlCommandFrame decoded{};
    decoded.sequence = get_u16(buffer_.data() + 6);
    decoded.sender_generation = sender_generation;
    decoded.sender_time_us = get_u64(buffer_.data() + 12);
    decoded.sample.mode = static_cast<GNCMode>(mode);
    decoded.sample.setpoint.use_yaw_rate_ref = (flags & 1) != 0;
    decoded.sample.setpoint.depth_ref = get_f32(buffer_.data() + 24);
    decoded.sample.setpoint.heading_ref = get_f32(buffer_.data() + 28);
    decoded.sample.setpoint.surge_ref = get_f32(buffer_.data() + 32);
    decoded.sample.setpoint.yaw_rate_ref = get_f32(buffer_.data() + 36);
    decoded.sample.setpoint.wp_n = get_f32(buffer_.data() + 40);
    decoded.sample.setpoint.wp_e = get_f32(buffer_.data() + 44);
    decoded.sample.setpoint.wp_d = get_f32(buffer_.data() + 48);

    const GNCSetpoint &sp = decoded.sample.setpoint;
    if (!finite_and_bounded(sp.depth_ref, 12000.0) ||
        !finite_and_bounded(sp.heading_ref, 1000.0) ||
        !finite_and_bounded(sp.surge_ref, 100.0) ||
        !finite_and_bounded(sp.yaw_rate_ref, 100.0) ||
        !finite_and_bounded(sp.wp_n, 10000000.0) ||
        !finite_and_bounded(sp.wp_e, 10000000.0) ||
        !finite_and_bounded(sp.wp_d, 12000.0))
        return false;

    frame = decoded;
    return true;
}

bool HitlCommandDecoder::feed(const uint8_t *data, std::size_t size,
                              HitlCommandFrame &latest) noexcept
{
    bool decoded_any = false;
    for (std::size_t i = 0; i < size; ++i)
    {
        if (used_ == buffer_.size())
            discard_first();
        buffer_[used_++] = data[i];

        while (used_ >= sizeof(kMagic) &&
               std::memcmp(buffer_.data(), kMagic, sizeof(kMagic)) != 0)
            discard_first();

        if (used_ == buffer_.size())
        {
            HitlCommandFrame candidate{};
            if (decode(candidate))
            {
                latest = candidate;
                decoded_any = true;
                used_ = 0;
            }
            else
            {
                discard_first();
            }
        }
    }
    return decoded_any;
}

std::array<uint8_t, kHitlCommandFrameSize> encode_hitl_command(
    const HitlCommandFrame &frame) noexcept
{
    std::array<uint8_t, kHitlCommandFrameSize> bytes{};
    std::memcpy(bytes.data(), kMagic, sizeof(kMagic));
    bytes[4] = kVersion;
    bytes[5] = kPayloadSize;
    put_u16(bytes.data() + 6, frame.sequence);
    put_u32(bytes.data() + 8, frame.sender_generation);
    put_u64(bytes.data() + 12, frame.sender_time_us);
    bytes[20] = static_cast<uint8_t>(frame.sample.mode);
    bytes[21] = frame.sample.setpoint.use_yaw_rate_ref ? 1 : 0;
    put_f32(bytes.data() + 24, frame.sample.setpoint.depth_ref);
    put_f32(bytes.data() + 28, frame.sample.setpoint.heading_ref);
    put_f32(bytes.data() + 32, frame.sample.setpoint.surge_ref);
    put_f32(bytes.data() + 36, frame.sample.setpoint.yaw_rate_ref);
    put_f32(bytes.data() + 40, frame.sample.setpoint.wp_n);
    put_f32(bytes.data() + 44, frame.sample.setpoint.wp_e);
    put_f32(bytes.data() + 48, frame.sample.setpoint.wp_d);
    put_u16(bytes.data() + 52,
            crc16_x25(bytes.data(), bytes.size() - 2));
    return bytes;
}
}