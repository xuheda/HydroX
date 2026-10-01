#include "hydrox/runtime/hil_session_config.h"
#include "hydrox/runtime/hil_session_mapping.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace hydrox::runtime
{
namespace
{
    constexpr double kPi = 3.1415926535897932384626433832795;
    constexpr double kTwoPi = 2.0 * kPi;
    constexpr uint32_t kMinPeriodUs = 1'000;
    constexpr uint32_t kMaxPeriodUs = 100'000;
    constexpr uint32_t kMaxSensorDtUs = 1'000'000;
    constexpr uint32_t kMaxTimeoutUs = 60'000'000;
    constexpr int32_t kMaxHorizontalMm = 1'000'000'000;
    constexpr int32_t kMaxDownMm = 12'000'000;
    constexpr int32_t kMaxSurgeMmps = 100'000;
    constexpr uint32_t kMaxGpsRadiusMm = 1'000'000'000;
    constexpr uint32_t kMaxMissionRadiusMm = 10'000'000;

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

    void put_i32(uint8_t *p, int32_t value) noexcept
    {
        put_u32(p, static_cast<uint32_t>(value));
    }

    void put_u64(uint8_t *p, uint64_t value) noexcept
    {
        for (unsigned int i = 0; i < 8; ++i)
            p[i] = static_cast<uint8_t>(value >> (8 * i));
    }

    uint16_t get_u16(const uint8_t *p) noexcept
    {
        return static_cast<uint16_t>(p[0]) |
               static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8);
    }

    uint32_t get_u32(const uint8_t *p) noexcept
    {
        uint32_t value = 0;
        for (unsigned int i = 0; i < 4; ++i)
            value |= static_cast<uint32_t>(p[i]) << (8 * i);
        return value;
    }

    int32_t get_i32(const uint8_t *p) noexcept
    {
        return static_cast<int32_t>(get_u32(p));
    }

    template<typename Integer>
    bool scaled_integer(double value, double scale, Integer &out) noexcept
    {
        if (!std::isfinite(value))
            return false;
        const double scaled = std::round(value * scale);
        if (scaled < static_cast<double>(std::numeric_limits<Integer>::min()) ||
            scaled > static_cast<double>(std::numeric_limits<Integer>::max()))
            return false;
        out = static_cast<Integer>(scaled);
        return true;
    }

    bool scaled_unsigned(double value, double scale, uint32_t &out) noexcept
    {
        if (!std::isfinite(value) || value < 0.0)
            return false;
        const double scaled = std::round(value * scale);
        if (scaled > static_cast<double>(std::numeric_limits<uint32_t>::max()))
            return false;
        out = static_cast<uint32_t>(scaled);
        return true;
    }

    struct Sha256
    {
        uint32_t state[8] = {
            0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
            0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};
        std::array<uint8_t, 64> buffer{};
        uint64_t total_bytes = 0;
        std::size_t buffered = 0;

        static uint32_t rotate_right(uint32_t value, unsigned int bits) noexcept
        {
            return (value >> bits) | (value << (32U - bits));
        }

        void transform(const uint8_t *block) noexcept
        {
            static constexpr uint32_t k[64] = {
                0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
                0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
                0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
                0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
                0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
                0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
                0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
                0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
                0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
                0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
                0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
                0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
                0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
                0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
                0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
                0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};
            uint32_t w[64]{};
            for (unsigned int i = 0; i < 16; ++i)
            {
                const uint8_t *p = block + i * 4;
                w[i] = (static_cast<uint32_t>(p[0]) << 24) |
                       (static_cast<uint32_t>(p[1]) << 16) |
                       (static_cast<uint32_t>(p[2]) << 8) |
                       static_cast<uint32_t>(p[3]);
            }
            for (unsigned int i = 16; i < 64; ++i)
            {
                const uint32_t s0 = rotate_right(w[i - 15], 7) ^
                                    rotate_right(w[i - 15], 18) ^
                                    (w[i - 15] >> 3);
                const uint32_t s1 = rotate_right(w[i - 2], 17) ^
                                    rotate_right(w[i - 2], 19) ^
                                    (w[i - 2] >> 10);
                w[i] = w[i - 16] + s0 + w[i - 7] + s1;
            }

            uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
            uint32_t e = state[4], f = state[5], g = state[6], h = state[7];
            for (unsigned int i = 0; i < 64; ++i)
            {
                const uint32_t s1 = rotate_right(e, 6) ^
                                    rotate_right(e, 11) ^ rotate_right(e, 25);
                const uint32_t choose = (e & f) ^ (~e & g);
                const uint32_t temp1 = h + s1 + choose + k[i] + w[i];
                const uint32_t s0 = rotate_right(a, 2) ^
                                    rotate_right(a, 13) ^ rotate_right(a, 22);
                const uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
                const uint32_t temp2 = s0 + majority;
                h = g;
                g = f;
                f = e;
                e = d + temp1;
                d = c;
                c = b;
                b = a;
                a = temp1 + temp2;
            }
            state[0] += a; state[1] += b; state[2] += c; state[3] += d;
            state[4] += e; state[5] += f; state[6] += g; state[7] += h;
        }

        void update(const uint8_t *data, std::size_t size) noexcept
        {
            total_bytes += size;
            while (size > 0)
            {
                const std::size_t count =
                    std::min(size, buffer.size() - buffered);
                std::memcpy(buffer.data() + buffered, data, count);
                buffered += count;
                data += count;
                size -= count;
                if (buffered == buffer.size())
                {
                    transform(buffer.data());
                    buffered = 0;
                }
            }
        }

        HilSessionDigest finish() noexcept
        {
            const uint64_t bit_count = total_bytes * 8U;
            buffer[buffered++] = 0x80;
            if (buffered > 56)
            {
                std::fill(buffer.begin() + buffered, buffer.end(), 0);
                transform(buffer.data());
                buffered = 0;
            }
            std::fill(buffer.begin() + buffered, buffer.begin() + 56, 0);
            for (unsigned int i = 0; i < 8; ++i)
                buffer[63 - i] = static_cast<uint8_t>(bit_count >> (8 * i));
            transform(buffer.data());

            HilSessionDigest digest{};
            for (unsigned int i = 0; i < 8; ++i)
            {
                digest[i * 4] = static_cast<uint8_t>(state[i] >> 24);
                digest[i * 4 + 1] = static_cast<uint8_t>(state[i] >> 16);
                digest[i * 4 + 2] = static_cast<uint8_t>(state[i] >> 8);
                digest[i * 4 + 3] = static_cast<uint8_t>(state[i]);
            }
            return digest;
        }
    };
}

bool hil_session_period_from_rate(int rate_hz, uint32_t &period_us) noexcept
{
    if (rate_hz <= 0 || 1'000'000 % rate_hz != 0)
        return false;
    const int period = 1'000'000 / rate_hz;
    if (period < static_cast<int>(kMinPeriodUs) ||
        period > static_cast<int>(kMaxPeriodUs))
        return false;
    period_us = static_cast<uint32_t>(period);
    return true;
}

bool resolve_hil_session_config(
    const HilSessionConfigValues &values,
    HilSessionConfigV1 &config,
    HilSessionField &field) noexcept
{
    HilSessionConfigV1 resolved;
    resolved.nominal_period_us = values.nominal_period_us;
    resolved.max_sensor_dt_us = values.max_sensor_dt_us;
    resolved.sensor_timeout_us = values.sensor_timeout_us;
    resolved.setpoint_timeout_us = values.setpoint_timeout_us;
    resolved.flags = values.allow_truth_heading_aid
                         ? kHilSessionFlagTruthHeadingAid : 0;
    resolved.required_sensor_mask = values.required_sensor_mask;
    resolved.accel_mode = values.accel_mode;
    resolved.feedback_source = values.feedback_source;

    if (!scaled_integer(values.initial_n_m, 1000.0, resolved.initial_n_mm))
        field = HilSessionField::InitialNorth;
    else if (!scaled_integer(values.initial_e_m, 1000.0, resolved.initial_e_mm))
        field = HilSessionField::InitialEast;
    else if (!scaled_integer(values.initial_down_m, 1000.0,
                             resolved.initial_down_mm))
        field = HilSessionField::InitialDown;
    else
    {
        double heading = values.initial_heading_rad;
        if (std::isfinite(heading))
        {
            heading = std::fmod(heading, kTwoPi);
            if (heading < 0.0)
                heading += kTwoPi;
        }
        if (!scaled_integer(heading, 1'000'000.0,
                            resolved.initial_heading_urad))
            field = HilSessionField::InitialHeading;
        else if (!scaled_integer(values.initial_surge_mps, 1000.0,
                                 resolved.initial_surge_mmps))
            field = HilSessionField::InitialSurge;
        else if (!scaled_integer(values.gps_origin_lat_deg, 10'000'000.0,
                                 resolved.gps_origin_lat_e7))
            field = HilSessionField::GpsLatitude;
        else if (!scaled_integer(values.gps_origin_lon_deg, 10'000'000.0,
                                 resolved.gps_origin_lon_e7))
            field = HilSessionField::GpsLongitude;
        else if (!scaled_integer(values.gps_origin_altitude_msl_m, 1000.0,
                                 resolved.gps_origin_alt_mm))
            field = HilSessionField::GpsAltitude;
        else if (!scaled_unsigned(values.gps_max_radius_m, 1000.0,
                                  resolved.gps_max_radius_mm))
            field = HilSessionField::GpsRadius;
        else if (!scaled_unsigned(values.mission_radius_m, 1000.0,
                                  resolved.mission_radius_mm))
            field = HilSessionField::MissionRadius;
        else if (validate_hil_session_config(resolved, field))
        {
            config = resolved;
            return true;
        }
    }
    return false;
}

bool validate_hil_session_config(
    const HilSessionConfigV1 &config,
    HilSessionField &field) noexcept
{
    field = HilSessionField::None;
    if (config.schema_version != kHilSessionConfigVersion)
        field = HilSessionField::SchemaVersion;
    else if ((config.flags & ~kHilSessionFlagTruthHeadingAid) != 0)
        field = HilSessionField::Flags;
    else if (config.nominal_period_us < kMinPeriodUs ||
             config.nominal_period_us > kMaxPeriodUs)
        field = HilSessionField::NominalPeriod;
    else if (config.max_sensor_dt_us < config.nominal_period_us ||
             config.max_sensor_dt_us > kMaxSensorDtUs)
        field = HilSessionField::MaxSensorDt;
    else if (config.sensor_timeout_us < config.nominal_period_us ||
             config.sensor_timeout_us > kMaxTimeoutUs)
        field = HilSessionField::SensorTimeout;
    else if (config.setpoint_timeout_us < config.nominal_period_us ||
             config.setpoint_timeout_us > kMaxTimeoutUs)
        field = HilSessionField::SetpointTimeout;
    else if (std::abs(static_cast<int64_t>(config.initial_n_mm)) >
             kMaxHorizontalMm)
        field = HilSessionField::InitialNorth;
    else if (std::abs(static_cast<int64_t>(config.initial_e_mm)) >
             kMaxHorizontalMm)
        field = HilSessionField::InitialEast;
    else if (std::abs(static_cast<int64_t>(config.initial_down_mm)) >
             kMaxDownMm)
        field = HilSessionField::InitialDown;
    else if (config.initial_heading_urad < 0 ||
             config.initial_heading_urad > 6'283'186)
        field = HilSessionField::InitialHeading;
    else if (std::abs(static_cast<int64_t>(config.initial_surge_mmps)) >
             kMaxSurgeMmps)
        field = HilSessionField::InitialSurge;
    else if (config.gps_origin_lat_e7 < -900'000'000 ||
             config.gps_origin_lat_e7 > 900'000'000)
        field = HilSessionField::GpsLatitude;
    else if (config.gps_origin_lon_e7 < -1'800'000'000 ||
             config.gps_origin_lon_e7 > 1'800'000'000)
        field = HilSessionField::GpsLongitude;
    else if (config.gps_max_radius_mm == 0 ||
             config.gps_max_radius_mm > kMaxGpsRadiusMm)
        field = HilSessionField::GpsRadius;
    else if (config.mission_radius_mm == 0 ||
             config.mission_radius_mm > kMaxMissionRadiusMm)
        field = HilSessionField::MissionRadius;
    else if (config.accel_mode != HilSessionAccelMode::Off &&
             config.accel_mode != HilSessionAccelMode::Auto &&
             config.accel_mode != HilSessionAccelMode::On)
        field = HilSessionField::AccelMode;
    else if (config.feedback_source != HilSessionFeedbackSource::EstimatedState &&
             config.feedback_source != HilSessionFeedbackSource::TruthDebug)
        field = HilSessionField::FeedbackSource;
    else if (config.reserved0 != 0 ||
             std::any_of(config.reserved.begin(), config.reserved.end(),
                         [](uint8_t value) { return value != 0; }))
        field = HilSessionField::Reserved;
    return field == HilSessionField::None;
}

HilSessionConfigPayload encode_hil_session_config(
    const HilSessionConfigV1 &config) noexcept
{
    HilSessionConfigPayload payload{};
    put_u16(payload.data(), config.schema_version);
    put_u16(payload.data() + 2, config.flags);
    put_u32(payload.data() + 4, config.nominal_period_us);
    put_u32(payload.data() + 8, config.max_sensor_dt_us);
    put_u32(payload.data() + 12, config.sensor_timeout_us);
    put_u32(payload.data() + 16, config.setpoint_timeout_us);
    put_i32(payload.data() + 20, config.initial_n_mm);
    put_i32(payload.data() + 24, config.initial_e_mm);
    put_i32(payload.data() + 28, config.initial_down_mm);
    put_i32(payload.data() + 32, config.initial_heading_urad);
    put_i32(payload.data() + 36, config.initial_surge_mmps);
    put_i32(payload.data() + 40, config.gps_origin_lat_e7);
    put_i32(payload.data() + 44, config.gps_origin_lon_e7);
    put_i32(payload.data() + 48, config.gps_origin_alt_mm);
    put_u32(payload.data() + 52, config.gps_max_radius_mm);
    put_u32(payload.data() + 56, config.mission_radius_mm);
    put_u32(payload.data() + 60, config.required_sensor_mask);
    payload[64] = static_cast<uint8_t>(config.accel_mode);
    payload[65] = static_cast<uint8_t>(config.feedback_source);
    put_u16(payload.data() + 66, config.reserved0);
    std::copy(config.reserved.begin(), config.reserved.end(),
              payload.begin() + 68);
    return payload;
}

bool decode_hil_session_config(
    const uint8_t *payload,
    std::size_t size,
    HilSessionConfigV1 &config) noexcept
{
    if (payload == nullptr || size != kHilSessionConfigPayloadSize)
        return false;
    HilSessionConfigV1 decoded;
    decoded.schema_version = get_u16(payload);
    decoded.flags = get_u16(payload + 2);
    decoded.nominal_period_us = get_u32(payload + 4);
    decoded.max_sensor_dt_us = get_u32(payload + 8);
    decoded.sensor_timeout_us = get_u32(payload + 12);
    decoded.setpoint_timeout_us = get_u32(payload + 16);
    decoded.initial_n_mm = get_i32(payload + 20);
    decoded.initial_e_mm = get_i32(payload + 24);
    decoded.initial_down_mm = get_i32(payload + 28);
    decoded.initial_heading_urad = get_i32(payload + 32);
    decoded.initial_surge_mmps = get_i32(payload + 36);
    decoded.gps_origin_lat_e7 = get_i32(payload + 40);
    decoded.gps_origin_lon_e7 = get_i32(payload + 44);
    decoded.gps_origin_alt_mm = get_i32(payload + 48);
    decoded.gps_max_radius_mm = get_u32(payload + 52);
    decoded.mission_radius_mm = get_u32(payload + 56);
    decoded.required_sensor_mask = get_u32(payload + 60);
    decoded.accel_mode = static_cast<HilSessionAccelMode>(payload[64]);
    decoded.feedback_source =
        static_cast<HilSessionFeedbackSource>(payload[65]);
    decoded.reserved0 = get_u16(payload + 66);
    std::copy(payload + 68, payload + 76, decoded.reserved.begin());
    config = decoded;
    return true;
}

HilSessionDigest hil_session_digest(
    uint64_t profile_fingerprint,
    const HilSessionConfigV1 &config) noexcept
{
    static constexpr uint8_t domain[] =
        "OceanX/HIL_SESSION_CONFIG/V1";
    const HilSessionConfigPayload payload =
        encode_hil_session_config(config);
    std::array<uint8_t, 8> fingerprint{};
    put_u64(fingerprint.data(), profile_fingerprint);
    Sha256 sha;
    sha.update(domain, sizeof(domain) - 1);
    sha.update(fingerprint.data(), fingerprint.size());
    sha.update(payload.data(), payload.size());
    return sha.finish();
}

bool hil_session_digest_equal(
    const HilSessionDigest &left,
    const HilSessionDigest &right) noexcept
{
    uint8_t difference = 0;
    for (std::size_t index = 0; index < left.size(); ++index)
        difference |= static_cast<uint8_t>(left[index] ^ right[index]);
    return difference == 0;
}

bool hil_session_digest_is_zero(const HilSessionDigest &digest) noexcept
{
    uint8_t combined = 0;
    for (const uint8_t value : digest)
        combined |= value;
    return combined == 0;
}

void apply_hil_session_config(
    const HilSessionConfigV1 &session,
    HilRuntimeConfig &runtime,
    SensorAdapter::Params &sensors) noexcept
{
    runtime.initial_state.eta[0] = session.initial_n_mm * 1e-3;
    runtime.initial_state.eta[1] = session.initial_e_mm * 1e-3;
    runtime.initial_state.eta[2] = session.initial_down_mm * 1e-3;
    runtime.initial_state.eta[5] = session.initial_heading_urad * 1e-6;
    runtime.initial_state.nu[0] = session.initial_surge_mmps * 1e-3;
    runtime.initial_state.depth_m = runtime.initial_state.eta[2];
    runtime.nominal_dt_s = session.nominal_period_us * 1e-6;
    runtime.max_sensor_dt_s = session.max_sensor_dt_us * 1e-6;
    runtime.sensor_timeout_us = session.sensor_timeout_us;
    auto &safety = runtime.safety_profile;
    safety.external_command_loss_us = session.setpoint_timeout_us;
    safety.external_command_warn_us = std::min(
        safety.external_command_warn_us, safety.external_command_loss_us / 2);
    safety.command_hold_max_us = std::min(
        safety.command_hold_max_us,
        safety.external_command_loss_us - safety.external_command_warn_us);
    runtime.mission_radius_m = session.mission_radius_mm * 1e-3;
    runtime.control_feedback_source =
        session.feedback_source == HilSessionFeedbackSource::TruthDebug
            ? ControlFeedbackSource::TruthDebug
            : ControlFeedbackSource::EstimatedState;
    runtime.allow_truth_heading_aid = session.allow_truth_heading_aid();

    switch (session.accel_mode)
    {
    case HilSessionAccelMode::Off: sensors.accel_mode = AccelMode::Off; break;
    case HilSessionAccelMode::On: sensors.accel_mode = AccelMode::On; break;
    default: sensors.accel_mode = AccelMode::Auto; break;
    }
    sensors.gps_origin_lat_deg = session.gps_origin_lat_e7 * 1e-7;
    sensors.gps_origin_lon_deg = session.gps_origin_lon_e7 * 1e-7;
    sensors.gps_origin_altitude_msl_m = session.gps_origin_alt_mm * 1e-3;
    sensors.gps_geodetic_max_radius_m =
        session.gps_max_radius_mm * 1e-3;
}
} // namespace hydrox::runtime
