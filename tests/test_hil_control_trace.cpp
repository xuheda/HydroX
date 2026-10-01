#include "mavlink_hil.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
int fail(const char *message)
{
    std::fprintf(stderr, "FAIL: %s\n", message);
    return 1;
}

bool near(double left, double right, double tolerance = 1.0e-6)
{
    return std::abs(left - right) <= tolerance;
}
}

int main()
{
    hydrox::HilControlTraceMsg source;
    source.time_usec = 9'876'543;
    source.tick = 321;
    source.dt_s = 0.01;
    source.mode = 2;
    source.mission_state = 1;
    source.actuator_channel_count = 5;
    source.flags =
        hydrox::HIL_CONTROL_TRACE_FLAG_USE_PATH_SEGMENT |
        hydrox::HIL_CONTROL_TRACE_FLAG_CONTROLLER_RESET |
        hydrox::HIL_CONTROL_TRACE_FLAG_ACTUATOR_AUTHORIZED |
        hydrox::HIL_CONTROL_TRACE_FLAG_EXTERNAL_SETPOINT |
        hydrox::HIL_CONTROL_TRACE_FLAG_DVL_VALID |
        hydrox::HIL_CONTROL_TRACE_FLAG_EKF_INITIALIZED;
    source.eta[0] = 12.25;
    source.eta[2] = 4.75;
    source.eta[5] = -0.6;
    source.nu[0] = 1.3;
    source.nu[5] = 0.15;
    source.depth_m = 4.75;
    source.depth_ref = 8.0;
    source.heading_ref = 0.4;
    source.surge_ref = 1.8;
    source.yaw_rate_ref = 0.2;
    source.wp_n = 50.0;
    source.wp_e = -20.0;
    source.wp_d = 8.0;
    source.path_start_n = -5.0;
    source.path_start_e = 2.0;
    source.lookahead_m = 10.0;
    source.arrival_radius_m = 3.0;
    source.wrench[0] = 42.5f;
    source.wrench[4] = -3.25f;
    source.wrench[5] = 7.75f;
    source.setpoint_age_s = 0.05f;
    source.valid = true;

    hydrox::MavlinkHIL codec;
    hydrox::MavlinkPacket packet;
    if (!codec.encode_hil_control_trace(packet, source))
        return fail("valid control trace did not encode");
    if (packet.size() != 10 + hydrox::HIL_CONTROL_TRACE_PAYLOAD_LEN + 2)
        return fail("control trace wire length");

    hydrox::MavlinkHIL decoder;
    const auto frames = decoder.feed(packet.data(), packet.size());
    if (frames.size() != 1 ||
        frames.front().msg_id != hydrox::MSGID_HIL_CONTROL_TRACE)
        return fail("control trace did not frame");
    const auto decoded = decoder.parse_hil_control_trace(frames.front());
    if (!decoded.valid || decoded.time_usec != source.time_usec ||
        decoded.tick != source.tick || decoded.flags != source.flags ||
        decoded.mode != source.mode ||
        decoded.mission_state != source.mission_state ||
        decoded.actuator_channel_count != source.actuator_channel_count ||
        !near(decoded.dt_s, source.dt_s, 1.0e-12) ||
        !near(decoded.eta[0], source.eta[0], 1.0e-12) ||
        !near(decoded.nu[5], source.nu[5], 1.0e-12) ||
        !near(decoded.depth_ref, source.depth_ref, 1.0e-12) ||
        !near(decoded.path_start_e, source.path_start_e, 1.0e-12) ||
        !near(decoded.wrench[4], source.wrench[4]) ||
        !near(decoded.setpoint_age_s, source.setpoint_age_s))
        return fail("decoded control trace differs from source");

    auto corrupt = packet;
    corrupt.bytes[80] ^= 0x20;
    hydrox::MavlinkHIL corrupt_decoder;
    if (!corrupt_decoder.feed(corrupt.data(), corrupt.size()).empty())
        return fail("CRC-corrupt control trace was accepted");

    source.flags = 0x8000;
    if (codec.encode_hil_control_trace(packet, source))
        return fail("unknown control trace flags were encoded");
    source.flags = 0;
    source.dt_s = std::numeric_limits<double>::quiet_NaN();
    if (codec.encode_hil_control_trace(packet, source))
        return fail("non-finite control trace was encoded");

    std::puts("PASS: HITL per-tick control trace codec");
    return 0;
}
