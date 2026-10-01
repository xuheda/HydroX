#include "mavlink_hil.h"

#include <cmath>
#include <cstdio>

namespace
{
int fail(const char *message)
{
    std::fprintf(stderr, "FAIL: %s\n", message);
    return 1;
}

bool near(double lhs, double rhs, double tolerance = 1.0e-6)
{
    return std::abs(lhs - rhs) <= tolerance;
}
}

int main()
{
    hydrox::HilFcStateMsg source;
    source.time_usec = 123456789;
    source.tick = 42;
    source.mode = 2;
    source.mission_state = 1;
    source.actuator_channel_count = 5;
    source.flags = hydrox::HIL_FC_STATE_FLAG_DVL_VALID |
                   hydrox::HIL_FC_STATE_FLAG_EKF_INITIALIZED |
                   hydrox::HIL_FC_STATE_FLAG_ARMED;
    source.eta[0] = 10.5;
    source.eta[5] = -1.2;
    source.nu[0] = 1.75;
    source.nu[5] = 0.25;
    source.depth_m = 6.25;
    source.normalized[4] = 0.6f;
    source.dvl_vel[0] = 1.7f;
    source.acc[2] = -9.8f;
    source.gyro[2] = 0.25f;
    source.gps_lat = 311234567;
    source.gps_lon = 1217654321;
    source.gps_alt = 12000;
    source.gps_vel_ned[0] = 1.5f;
    source.motor_rpm_actual = 900.0f;
    source.motor_thrust_N = 22.0f;
    source.motor_power_W = 80.0f;
    source.motor_current_A = 3.2f;
    source.power_total_W = 95.0f;
    source.energy_Wh = 12.0f;
    source.battery_soc = 0.8f;
    source.voltage_terminal = 23.5f;
    source.runtime_remaining_s = 3600.0f;
    source.commanded_rpm = 1000.0f;
    source.gps_fix = 3;
    source.gps_satellites = 14;
    source.valid = true;

    hydrox::MavlinkHIL encoder;
    hydrox::MavlinkPacket packet;
    if (!encoder.encode_hil_fc_state(packet, source))
        return fail("valid FC state did not encode");

    hydrox::MavlinkHIL decoder;
    const auto frames = decoder.feed(packet.data(), packet.size());
    if (frames.size() != 1 || frames.front().msg_id != hydrox::MSGID_HIL_FC_STATE)
        return fail("encoded FC state did not frame");
    const auto decoded = decoder.parse_hil_fc_state(frames.front());
    if (!decoded.valid || decoded.time_usec != source.time_usec ||
        decoded.tick != source.tick || decoded.flags != source.flags ||
        decoded.mode != source.mode ||
        decoded.actuator_channel_count != source.actuator_channel_count ||
        !near(decoded.eta[0], source.eta[0]) ||
        !near(decoded.eta[5], source.eta[5]) ||
        !near(decoded.nu[0], source.nu[0]) ||
        !near(decoded.depth_m, source.depth_m) ||
        !near(decoded.normalized[4], source.normalized[4]) ||
        decoded.gps_lat != source.gps_lat ||
        !near(decoded.motor_rpm_actual, source.motor_rpm_actual))
        return fail("decoded FC state differs from source");

    auto corrupt = packet;
    corrupt.bytes[120] ^= 0x40;
    hydrox::MavlinkHIL corrupt_decoder;
    if (!corrupt_decoder.feed(corrupt.data(), corrupt.size()).empty())
        return fail("CRC-corrupt FC state was accepted");

    source.flags = 0x8000;
    if (encoder.encode_hil_fc_state(packet, source))
        return fail("unknown FC state flags were encoded");

    std::puts("PASS: HITL flight-controller state telemetry codec");
    return 0;
}