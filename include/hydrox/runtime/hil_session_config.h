#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace hydrox::runtime
{
    constexpr uint16_t kHilSessionConfigVersion = 1;
    constexpr std::size_t kHilSessionConfigPayloadSize = 76;
    constexpr std::size_t kHilSessionDigestSize = 32;
    constexpr uint16_t kHilSessionFlagTruthHeadingAid = 1u << 0;

    enum class HilSessionAccelMode : uint8_t
    {
        Off = 0,
        Auto = 1,
        On = 2,
    };

    enum class HilSessionFeedbackSource : uint8_t
    {
        EstimatedState = 0,
        TruthDebug = 1,
    };

    enum class HilSessionField : uint16_t
    {
        None = 0,
        SchemaVersion,
        Flags,
        NominalPeriod,
        MaxSensorDt,
        SensorTimeout,
        SetpointTimeout,
        InitialNorth,
        InitialEast,
        InitialDown,
        InitialHeading,
        InitialSurge,
        GpsLatitude,
        GpsLongitude,
        GpsAltitude,
        GpsRadius,
        MissionRadius,
        RequiredSensors,
        AccelMode,
        FeedbackSource,
        Reserved,
    };

    enum class HilSessionRejectReason : uint16_t
    {
        None = 0,
        UnsupportedVersion,
        InvalidLength,
        HashMismatch,
        ProfileMismatch,
        InvalidRange,
        UnsupportedPolicy,
        MissingSensorCapability,
        StaleNonce,
        Busy,
        InternalError,
    };

    using HilSessionDigest = std::array<uint8_t, kHilSessionDigestSize>;
    using HilSessionConfigPayload =
        std::array<uint8_t, kHilSessionConfigPayloadSize>;

    /**
     * Canonical per-run HIL contract. All physical quantities use integer wire
     * units so host and embedded builds hash exactly the same bytes.
     */
    struct HilSessionConfigV1
    {
        uint16_t schema_version = kHilSessionConfigVersion;
        uint16_t flags = 0;

        uint32_t nominal_period_us = 10'000;
        uint32_t max_sensor_dt_us = 250'000;
        uint32_t sensor_timeout_us = 500'000;
        uint32_t setpoint_timeout_us = 500'000;

        int32_t initial_n_mm = 0;
        int32_t initial_e_mm = 0;
        int32_t initial_down_mm = 0;
        int32_t initial_heading_urad = 0;
        int32_t initial_surge_mmps = 0;

        int32_t gps_origin_lat_e7 = 0;
        int32_t gps_origin_lon_e7 = 0;
        int32_t gps_origin_alt_mm = 0;
        uint32_t gps_max_radius_mm = 10'000'000;

        uint32_t mission_radius_mm = 3'000;
        uint32_t required_sensor_mask = 0;
        HilSessionAccelMode accel_mode = HilSessionAccelMode::Auto;
        HilSessionFeedbackSource feedback_source =
            HilSessionFeedbackSource::EstimatedState;
        uint16_t reserved0 = 0;
        std::array<uint8_t, 8> reserved{};

        bool allow_truth_heading_aid() const noexcept
        {
            return (flags & kHilSessionFlagTruthHeadingAid) != 0;
        }
    };

    /** Human-unit input accepted by host configuration adapters. */
    struct HilSessionConfigValues
    {
        uint32_t nominal_period_us = 10'000;
        uint32_t max_sensor_dt_us = 250'000;
        uint32_t sensor_timeout_us = 500'000;
        uint32_t setpoint_timeout_us = 500'000;

        double initial_n_m = 0.0;
        double initial_e_m = 0.0;
        double initial_down_m = 0.0;
        double initial_heading_rad = 0.0;
        double initial_surge_mps = 0.0;

        double gps_origin_lat_deg = 0.0;
        double gps_origin_lon_deg = 0.0;
        double gps_origin_altitude_msl_m = 0.0;
        double gps_max_radius_m = 10'000.0;
        double mission_radius_m = 3.0;

        uint32_t required_sensor_mask = 0;
        HilSessionAccelMode accel_mode = HilSessionAccelMode::Auto;
        HilSessionFeedbackSource feedback_source =
            HilSessionFeedbackSource::EstimatedState;
        bool allow_truth_heading_aid = false;
    };

    /** Convert an integer rate to the one authoritative wire period. */
    bool hil_session_period_from_rate(
        int rate_hz, uint32_t &period_us) noexcept;

    /** Resolve human units into a complete canonical configuration. */
    bool resolve_hil_session_config(
        const HilSessionConfigValues &values,
        HilSessionConfigV1 &config,
        HilSessionField &field) noexcept;

    bool validate_hil_session_config(
        const HilSessionConfigV1 &config,
        HilSessionField &field) noexcept;

    HilSessionConfigPayload encode_hil_session_config(
        const HilSessionConfigV1 &config) noexcept;
    bool decode_hil_session_config(
        const uint8_t *payload,
        std::size_t size,
        HilSessionConfigV1 &config) noexcept;

    /** SHA-256 over the schema domain, profile fingerprint and payload. */
    HilSessionDigest hil_session_digest(
        uint64_t profile_fingerprint,
        const HilSessionConfigV1 &config) noexcept;
    bool hil_session_digest_equal(
        const HilSessionDigest &left,
        const HilSessionDigest &right) noexcept;
    bool hil_session_digest_is_zero(
        const HilSessionDigest &digest) noexcept;
} // namespace hydrox::runtime
