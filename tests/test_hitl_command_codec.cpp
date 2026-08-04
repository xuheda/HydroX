#include "hydrox/runtime/hitl_command_codec.h"
#include <cmath>
#include <cstdio>
#include <vector>

namespace
{
int fail(const char *message)
{
    std::fprintf(stderr, "FAIL: %s\n", message);
    return 1;
}
}

int main()
{
    hydrox::runtime::HitlCommandFrame source{};
    source.sequence = 42;
    source.sender_generation = 7;
    source.sender_time_us = 123456789;
    source.sample.mode = hydrox::GNCMode::WAYPOINT_3D;
    source.sample.setpoint.depth_ref = 12.5;
    source.sample.setpoint.heading_ref = 1.25;
    source.sample.setpoint.surge_ref = 2.0;
    source.sample.setpoint.use_yaw_rate_ref = true;
    source.sample.setpoint.yaw_rate_ref = -0.4;
    source.sample.setpoint.wp_n = 10.0;
    source.sample.setpoint.wp_e = -20.0;
    source.sample.setpoint.wp_d = 30.0;

    const auto encoded = hydrox::runtime::encode_hitl_command(source);
    hydrox::runtime::HitlCommandDecoder decoder;
    hydrox::runtime::HitlCommandFrame decoded{};
    const uint8_t garbage[] = {0, 1, 'H', 0xff};
    if (decoder.feed(garbage, sizeof(garbage), decoded))
        return fail("garbage decoded as a command");
    if (decoder.feed(encoded.data(), 13, decoded))
        return fail("partial command decoded early");
    if (!decoder.feed(encoded.data() + 13, encoded.size() - 13, decoded))
        return fail("fragmented command was not decoded");
    if (decoded.sequence != source.sequence ||
        decoded.sender_generation != source.sender_generation ||
        decoded.sender_time_us != source.sender_time_us ||
        decoded.sample.mode != source.sample.mode ||
        !decoded.sample.setpoint.use_yaw_rate_ref ||
        std::abs(decoded.sample.setpoint.depth_ref - 12.5) > 1e-5 ||
        std::abs(decoded.sample.setpoint.wp_e + 20.0) > 1e-5)
        return fail("decoded command does not match source");

    auto corrupt = encoded;
    corrupt[30] ^= 0x40;
    if (decoder.feed(corrupt.data(), corrupt.size(), decoded))
        return fail("CRC-corrupt command was accepted");

    std::vector<uint8_t> pair(encoded.begin(), encoded.end());
    pair.insert(pair.end(), encoded.begin(), encoded.end());
    if (!decoder.feed(pair.data(), pair.size(), decoded))
        return fail("back-to-back commands were not decoded");

    std::puts("PASS: HITL companion command codec");
    return 0;
}