#include "hydrox/runtime/hil_session_config.h"
#include "hydrox/runtime/hil_session_mapping.h"
#include "mavlink_hil.h"

#include <algorithm>
#include <cmath>
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

bool close(double left, double right)
{
    return std::abs(left - right) < 1e-9;
}
}

int main()
{
    int failures = 0;
    hydrox::runtime::HilSessionConfigValues values;
    values.nominal_period_us = 10'000;
    values.max_sensor_dt_us = 250'000;
    values.sensor_timeout_us = 700'000;
    values.setpoint_timeout_us = 900'000;
    values.initial_n_m = 12.345;
    values.initial_e_m = -6.789;
    values.initial_down_m = 4.25;
    values.initial_heading_rad = -0.5;
    values.initial_surge_mps = 1.25;
    values.gps_origin_lat_deg = 22.3;
    values.gps_origin_lon_deg = 114.17;
    values.gps_origin_altitude_msl_m = 8.5;
    values.gps_max_radius_m = 5000.0;
    values.mission_radius_m = 2.5;
    values.required_sensor_mask = 0x15;
    values.accel_mode = hydrox::runtime::HilSessionAccelMode::On;
    values.feedback_source =
        hydrox::runtime::HilSessionFeedbackSource::EstimatedState;

    hydrox::runtime::HilSessionConfigV1 config;
    hydrox::runtime::HilSessionField field =
        hydrox::runtime::HilSessionField::None;
    failures += expect(
        hydrox::runtime::resolve_hil_session_config(values, config, field),
        "valid human-unit session values resolve");
    failures += expect(
        config.initial_n_mm == 12'345 && config.initial_e_mm == -6'789 &&
            config.initial_down_mm == 4'250 &&
            config.initial_surge_mmps == 1'250,
        "physical values use canonical integer units");
    failures += expect(
        config.initial_heading_urad > 5'783'000 &&
            config.initial_heading_urad < 5'784'000,
        "negative heading is normalized to the canonical positive turn");

    const auto payload = hydrox::runtime::encode_hil_session_config(config);
    hydrox::runtime::HilSessionConfigV1 decoded;
    failures += expect(
        hydrox::runtime::decode_hil_session_config(
            payload.data(), payload.size(), decoded),
        "canonical payload decodes");
    failures += expect(
        hydrox::runtime::encode_hil_session_config(decoded) == payload,
        "canonical payload round-trips byte-for-byte");

    constexpr uint64_t kProfileFingerprint = 0x1122334455667788ULL;
    const auto digest = hydrox::runtime::hil_session_digest(
        kProfileFingerprint, config);
    const hydrox::runtime::HilSessionDigest golden_digest = {
        0x1c, 0x4f, 0x76, 0xa5, 0x11, 0x70, 0x66, 0xe0,
        0x99, 0x6e, 0x16, 0x59, 0x61, 0x98, 0x79, 0xde,
        0x68, 0x3f, 0xe7, 0x27, 0x23, 0xa4, 0xc8, 0xcb,
        0xb5, 0x75, 0xd4, 0x8a, 0x8f, 0x3d, 0x84, 0xe9};
    failures += expect(
        hydrox::runtime::hil_session_digest_equal(
            digest, golden_digest),
        "portable SHA-256 matches the independent canonical golden vector");
    failures += expect(
        !hydrox::runtime::hil_session_digest_is_zero(digest),
        "session digest is non-zero");
    failures += expect(
        !hydrox::runtime::hil_session_digest_equal(
            digest,
            hydrox::runtime::hil_session_digest(
                kProfileFingerprint + 1, config)),
        "session digest is bound to the vehicle profile");

    hydrox::HilSessionConfigMsg outbound;
    outbound.profile_fingerprint = kProfileFingerprint;
    outbound.nonce = 0x10203040U;
    outbound.digest = digest;
    outbound.config = config;
    outbound.valid = true;
    hydrox::MavlinkHIL transmitter(42, 191);
    hydrox::MavlinkPacket packet;
    failures += expect(
        transmitter.encode_hil_session_config(packet, outbound),
        "valid session config MAVLink message encodes");
    failures += expect(
        packet.size() == 10U + hydrox::HIL_SESSION_CONFIG_PAYLOAD_LEN + 2U,
        "session config uses one exact MAVLink 2 frame");

    hydrox::MavlinkHIL receiver;
    const auto frames = receiver.feed(packet.data(), packet.size());
    failures += expect(
        frames.size() == 1 &&
            frames[0].msg_id == hydrox::MSGID_HIL_SESSION_CONFIG,
        "session config frame passes MAVLink CRC");
    if (frames.size() == 1)
    {
        const auto inbound = receiver.parse_hil_session_config(frames[0]);
        failures += expect(
            inbound.valid && inbound.digest_valid &&
                inbound.profile_fingerprint == kProfileFingerprint &&
                inbound.nonce == outbound.nonce &&
                hydrox::runtime::hil_session_digest_equal(
                    inbound.digest, outbound.digest) &&
                hydrox::runtime::encode_hil_session_config(inbound.config) ==
                    payload,
            "session identity, nonce, digest and payload round-trip");

        auto tampered = frames[0];
        tampered.payload[20] ^= 0x40;
        const auto rejected = receiver.parse_hil_session_config(tampered);
        failures += expect(
            rejected.valid && !rejected.digest_valid,
            "canonical payload tampering is distinguished from malformed framing");
    }

    hydrox::HilSessionStatusMsg status;
    status.profile_fingerprint = kProfileFingerprint;
    status.nonce = outbound.nonce;
    status.digest = digest;
    status.operation = hydrox::HilSessionStatusOperation::Rejected;
    status.reason = hydrox::runtime::HilSessionRejectReason::UnsupportedPolicy;
    status.field = hydrox::runtime::HilSessionField::FeedbackSource;
    status.valid = true;
    failures += expect(
        transmitter.encode_hil_session_status(packet, status),
        "session rejection status encodes");
    const auto status_frames = receiver.feed(packet.data(), packet.size());
    failures += expect(status_frames.size() == 1,
                       "session status frame passes MAVLink CRC");
    if (status_frames.size() == 1)
    {
        const auto parsed = receiver.parse_hil_session_status(status_frames[0]);
        failures += expect(
            parsed.valid &&
                parsed.operation == hydrox::HilSessionStatusOperation::Rejected &&
                parsed.reason ==
                    hydrox::runtime::HilSessionRejectReason::UnsupportedPolicy &&
                parsed.field == hydrox::runtime::HilSessionField::FeedbackSource,
            "session rejection reason and field round-trip");
        auto unknown_reason = status_frames[0];
        unknown_reason.payload[45] = 0xff;
        unknown_reason.payload[46] = 0xff;
        failures += expect(
            !receiver.parse_hil_session_status(unknown_reason).valid,
            "unknown Session rejection reasons are rejected");
    }

    auto invalid = config;
    invalid.reserved[3] = 1;
    failures += expect(
        !hydrox::runtime::validate_hil_session_config(invalid, field) &&
            field == hydrox::runtime::HilSessionField::Reserved,
        "non-zero reserved bytes are rejected");
    invalid = config;
    invalid.max_sensor_dt_us = invalid.nominal_period_us - 1;
    failures += expect(
        !hydrox::runtime::validate_hil_session_config(invalid, field) &&
            field == hydrox::runtime::HilSessionField::MaxSensorDt,
        "sensor time gap smaller than the nominal period is rejected");

    hydrox::runtime::HilRuntimeConfig runtime;
    hydrox::SensorAdapter::Params sensors;
    auto short_session = config;
    short_session.setpoint_timeout_us = 20'000;
    hydrox::runtime::HilRuntimeConfig short_runtime;
    hydrox::SensorAdapter::Params short_sensors;
    hydrox::runtime::apply_hil_session_config(short_session, short_runtime, short_sensors);
    failures += expect(short_runtime.safety_profile.valid() &&
                       short_runtime.safety_profile.external_command_loss_us == 20'000 &&
                       short_runtime.safety_profile.external_command_warn_us == 10'000 &&
                       short_runtime.safety_profile.command_hold_max_us == 10'000,
                       "short Session timeout preserves a valid single safety timing policy");
    const auto original_motor = runtime.motor;
    const auto original_estimation = runtime.estimation_profile.vehicle_class;
    hydrox::runtime::apply_hil_session_config(config, runtime, sensors);
    failures += expect(
        close(runtime.initial_state.eta[0], 12.345) &&
            close(runtime.initial_state.eta[1], -6.789) &&
            close(runtime.initial_state.depth_m, 4.25) &&
            close(runtime.nominal_dt_s, 0.01) &&
            close(runtime.max_sensor_dt_s, 0.25) &&
            runtime.sensor_timeout_us == 700'000 &&
            runtime.safety_profile.external_command_loss_us == 900'000,
        "Session-owned runtime fields map from the canonical config");
    failures += expect(
        runtime.estimation_profile.vehicle_class == original_estimation &&
            close(runtime.motor.tau_m, original_motor.tau_m),
        "Session mapping does not overwrite Profile-owned fields");
    failures += expect(
        sensors.accel_mode == hydrox::AccelMode::On &&
            close(sensors.gps_origin_lat_deg, 22.3) &&
            close(sensors.gps_origin_lon_deg, 114.17) &&
            close(sensors.gps_origin_altitude_msl_m, 8.5) &&
            close(sensors.gps_geodetic_max_radius_m, 5000.0),
        "Session-owned sensor fields map from the canonical config");

    uint32_t period_us = 0;
    failures += expect(
        hydrox::runtime::hil_session_period_from_rate(100, period_us) &&
            period_us == 10'000 &&
            !hydrox::runtime::hil_session_period_from_rate(60, period_us),
        "rate conversion accepts only exact microsecond periods");

    if (failures == 0)
        std::printf("test_hil_session_config: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
