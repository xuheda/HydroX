// Copyright (c) 2026 OceanX.
#include "sensor_adapter.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

namespace
{
    constexpr uint32_t kHilFieldsAccel = (1u << 0) | (1u << 1) | (1u << 2);
    constexpr uint32_t kHilFieldsGyro = (1u << 3) | (1u << 4) | (1u << 5);
    constexpr uint32_t kHilFieldsMag = (1u << 6) | (1u << 7) | (1u << 8);
    constexpr uint32_t kHilFieldAbsPressure = 1u << 9;

    template <typename Buffer, typename T>
    void write_le(Buffer &buf, size_t offset, const T &value)
    {
        std::memcpy(buf.data() + offset, &value, sizeof(T));
    }

    hydrox::MavFrame hil_sensor(float ax, float ay, float az, uint32_t fields,
                                float mx = 25.0f, float my = 0.0f,
                                float mz = 43.30127f,
                                uint64_t time_us = 1000)
    {
        hydrox::MavFrame f;
        f.msg_id = hydrox::MSGID_HIL_SENSOR;
        f.payload.resize(64, 0);

        const float zero = 0.0f;
        const float pressure = 101325.0f;
        write_le(f.payload, 0, time_us);
        write_le(f.payload, 8, ax);
        write_le(f.payload, 12, ay);
        write_le(f.payload, 16, az);
        write_le(f.payload, 20, zero);
        write_le(f.payload, 24, zero);
        write_le(f.payload, 28, zero);
        write_le(f.payload, 32, mx);
        write_le(f.payload, 36, my);
        write_le(f.payload, 40, mz);
        write_le(f.payload, 44, pressure);
        write_le(f.payload, 60, fields);
        return f;
    }

    hydrox::MavFrame heartbeat(uint8_t system_status)
    {
        hydrox::MavFrame f;
        f.msg_id = hydrox::MSGID_HEARTBEAT;
        f.payload.resize(9, 0);
        f.payload[4] = 12;
        f.payload[7] = system_status;
        f.payload[8] = 3;
        return f;
    }

    hydrox::MavFrame hil_dvl(
        uint64_t time_us,
        hydrox::DvlTrackingMode tracking_mode = hydrox::DvlTrackingMode::BottomTrack,
        bool valid = true)
    {
        hydrox::MavFrame f;
        f.msg_id = hydrox::MSGID_HIL_DVL;
        f.payload.resize(hydrox::HIL_DVL_PAYLOAD_LEN, 0);
        const float vx = 1.0f;
        const float vy = 0.0f;
        const float vz = 0.0f;
        const bool bottom_track = tracking_mode == hydrox::DvlTrackingMode::BottomTrack;
        const float altitude = bottom_track ? 10.0f : std::numeric_limits<float>::quiet_NaN();
        const uint8_t flags = valid
            ? static_cast<uint8_t>(hydrox::HIL_DVL_FLAG_VELOCITY_VALID |
                                   (bottom_track ? hydrox::HIL_DVL_FLAG_ALTITUDE_VALID : 0u))
            : 0u;
        write_le(f.payload, 0, time_us);
        write_le(f.payload, 8, vx);
        write_le(f.payload, 12, vy);
        write_le(f.payload, 16, vz);
        write_le(f.payload, 20, altitude);
        f.payload[24] = static_cast<uint8_t>(tracking_mode);
        f.payload[25] = flags;
        return f;
    }

    hydrox::MavFrame hil_wheel_odometry(uint64_t time_us, bool valid = true)
    {
        hydrox::MavFrame f;
        f.msg_id = hydrox::MSGID_HIL_WHEEL_ODOMETRY;
        f.payload.resize(hydrox::HIL_WHEEL_ODOMETRY_PAYLOAD_LEN, 0);
        const float right_radps = 10.0f;
        const float left_radps = 12.0f;
        const float forward_mps = 0.7546f;
        const float yaw_rate_radps = 0.4204f;
        const float velocity_variance = 0.0025f;
        const float yaw_rate_variance = 0.01f;
        write_le(f.payload, 0, time_us);
        write_le(f.payload, 8, right_radps);
        write_le(f.payload, 12, left_radps);
        write_le(f.payload, 16, forward_mps);
        write_le(f.payload, 20, yaw_rate_radps);
        write_le(f.payload, 24, velocity_variance);
        write_le(f.payload, 28, yaw_rate_variance);
        f.payload[32] = valid
            ? hydrox::HIL_WHEEL_ODOMETRY_FLAG_VELOCITY_VALID |
              hydrox::HIL_WHEEL_ODOMETRY_FLAG_WHEEL_SPEEDS_VALID |
              hydrox::HIL_WHEEL_ODOMETRY_FLAG_YAW_RATE_VALID
            : 0u;
        return f;
    }

    hydrox::MavFrame hil_gps(
        uint64_t time_us,
        uint8_t fix_type = 3,
        uint16_t eph = 123,
        uint16_t epv = 234,
        uint16_t vel = 345,
        int16_t vn = 0,
        int16_t ve = 0,
        int16_t vd = 0)
    {
        hydrox::MavFrame f;
        f.msg_id = hydrox::MSGID_HIL_GPS;
        f.payload.resize(36, 0);
        const int32_t lat = 1000;
        const int32_t lon = 2000;
        const int32_t alt = -3000;
        const uint16_t cog = 9000;
        const uint8_t sats = 11;
        write_le(f.payload, 0, time_us);
        write_le(f.payload, 8, fix_type);
        write_le(f.payload, 9, lat);
        write_le(f.payload, 13, lon);
        write_le(f.payload, 17, alt);
        write_le(f.payload, 21, eph);
        write_le(f.payload, 23, epv);
        write_le(f.payload, 25, vel);
        write_le(f.payload, 27, vn);
        write_le(f.payload, 29, ve);
        write_le(f.payload, 31, vd);
        write_le(f.payload, 33, cog);
        write_le(f.payload, 35, sats);
        return f;
    }

    hydrox::MavFrame hil_truth(uint64_t time_us)
    {
        hydrox::MavFrame f;
        f.msg_id = hydrox::MSGID_HIL_TRUTH_STATE;
        f.payload.resize(104, 0);
        write_le(f.payload, 0, time_us);
        for (int i = 0; i < 6; ++i)
        {
            const double eta = static_cast<double>(i);
            const double nu = static_cast<double>(i + 10);
            write_le(f.payload, 8 + i * 8, eta);
            write_le(f.payload, 56 + i * 8, nu);
        }
        return f;
    }

    bool expect(bool ok, const char *msg)
    {
        if (!ok)
            std::fprintf(stderr, "FAIL: %s\n", msg);
        return ok;
    }

    bool have_accel_for(hydrox::AccelMode mode, const hydrox::MavFrame &frame)
    {
        hydrox::SensorAdapter adapter({hydrox::VehicleClass::UUV, mode});
        hydrox::MavlinkHIL codec;
        adapter.begin_cycle();
        adapter.ingest_frame(frame, codec);
        return adapter.build().have_accel;
    }

    hydrox::NavigationInput nav_for(const hydrox::MavFrame &frame)
    {
        hydrox::SensorAdapter adapter({hydrox::VehicleClass::UUV, hydrox::AccelMode::Auto});
        hydrox::MavlinkHIL codec;
        adapter.begin_cycle();
        adapter.ingest_frame(frame, codec);
        return adapter.build();
    }
} // namespace

int main()
{
    int fails = 0;

    const auto good = hil_sensor(0.0f, 0.0f, -9.80665f,
                                 kHilFieldsAccel | kHilFieldsGyro);

    hydrox::MavlinkHIL heartbeat_codec;
    const auto paused_heartbeat =
        heartbeat_codec.parse_heartbeat(heartbeat(hydrox::MAV_STATE_STANDBY));
    const auto active_heartbeat =
        heartbeat_codec.parse_heartbeat(heartbeat(hydrox::MAV_STATE_ACTIVE));
    fails += expect(paused_heartbeat.valid &&
                        paused_heartbeat.system_status == hydrox::MAV_STATE_STANDBY,
                    "heartbeat exposes simulator pause state") ? 0 : 1;
    fails += expect(active_heartbeat.valid &&
                        active_heartbeat.system_status == hydrox::MAV_STATE_ACTIVE,
                    "heartbeat exposes simulator active state") ? 0 : 1;
    const auto missing_bits = hil_sensor(0.0f, 0.0f, -9.80665f, kHilFieldsGyro);
    const auto absurd = hil_sensor(0.0f, 0.0f, 200.0f,
                                   kHilFieldsAccel | kHilFieldsGyro);

    fails += expect(have_accel_for(hydrox::AccelMode::Auto, good),
                    "auto accepts finite accel with all accel fields") ? 0 : 1;
    fails += expect(!have_accel_for(hydrox::AccelMode::Auto, missing_bits),
                    "auto rejects missing accel field bits") ? 0 : 1;
    fails += expect(!have_accel_for(hydrox::AccelMode::Auto, absurd),
                    "auto rejects absurd accel norm") ? 0 : 1;
    fails += expect(!have_accel_for(hydrox::AccelMode::Off, good),
                    "off disables accel mechanization") ? 0 : 1;
    fails += expect(have_accel_for(hydrox::AccelMode::On, missing_bits),
                    "on accepts finite accel without field bits") ? 0 : 1;

    const auto mag_good = hil_sensor(0.0f, 0.0f, -9.80665f,
                                     kHilFieldsAccel | kHilFieldsGyro | kHilFieldsMag);
    const auto mag_missing_bits = hil_sensor(0.0f, 0.0f, -9.80665f,
                                             kHilFieldsAccel | kHilFieldsGyro);
    const auto mag_absurd = hil_sensor(0.0f, 0.0f, -9.80665f,
                                       kHilFieldsAccel | kHilFieldsGyro | kHilFieldsMag,
                                       2000.0f, 0.0f, 0.0f);

    const auto nav_mag_good = nav_for(mag_good);
    fails += expect(nav_mag_good.have_mag,
                    "mag accepts finite field with all mag field bits") ? 0 : 1;
    fails += expect(!nav_for(mag_missing_bits).have_mag,
                    "mag rejects missing mag field bits") ? 0 : 1;
    fails += expect(!nav_for(mag_absurd).have_mag,
                    "mag rejects absurd field norm") ? 0 : 1;
    fails += expect(std::abs(nav_mag_good.mag_body.x() - 25.0) < 1e-5 &&
                        std::abs(nav_mag_good.mag_body.y()) < 1e-5,
                    "mag preserves parsed body-frame vector") ? 0 : 1;

    hydrox::SensorAdapter age_adapter;
    hydrox::MavlinkHIL codec;
    const auto parsed_bottom = codec.parse_hil_dvl(hil_dvl(100000));
    fails += expect(parsed_bottom.velocity_valid() &&
                        parsed_bottom.is_bottom_track() &&
                        parsed_bottom.altitude_valid() &&
                        std::abs(parsed_bottom.altitude_m - 10.0f) < 1e-6f,
                    "DVL bottom-track payload carries a valid bottom altitude") ? 0 : 1;
    const auto parsed_water = codec.parse_hil_dvl(
        hil_dvl(100000, hydrox::DvlTrackingMode::WaterTrack));
    fails += expect(parsed_water.velocity_valid() &&
                        parsed_water.is_water_track() &&
                        !parsed_water.altitude_valid() &&
                        std::isnan(parsed_water.altitude_m),
                    "DVL water-track payload carries no bottom altitude") ? 0 : 1;
    auto legacy_dvl = hil_dvl(100000);
    legacy_dvl.payload.resize(25);
    fails += expect(!codec.parse_hil_dvl(legacy_dvl).velocity_valid(),
                    "legacy 25-byte DVL payload is rejected") ? 0 : 1;

    const auto parsed_wheel = codec.parse_hil_wheel_odometry(
        hil_wheel_odometry(100000));
    fails += expect(parsed_wheel.velocity_valid() &&
                        parsed_wheel.wheel_speeds_valid() &&
                        std::abs(parsed_wheel.forward_mps - 0.7546f) < 1e-6f,
                    "wheel-odometry payload preserves encoder-derived velocity") ? 0 : 1;
    auto short_wheel = hil_wheel_odometry(100000);
    short_wheel.payload.resize(hydrox::HIL_WHEEL_ODOMETRY_PAYLOAD_LEN - 1);
    fails += expect(!codec.parse_hil_wheel_odometry(short_wheel).velocity_valid(),
                    "truncated wheel-odometry payload is rejected") ? 0 : 1;

    age_adapter.begin_cycle();
    age_adapter.ingest_frame(hil_dvl(100000), codec);
    age_adapter.ingest_frame(hil_gps(100000), codec);
    age_adapter.ingest_frame(hil_truth(100000), codec);
    age_adapter.ingest_frame(
        hil_sensor(0.0f, 0.0f, -9.80665f,
                   kHilFieldsAccel | kHilFieldsGyro,
                   25.0f, 0.0f, 43.30127f,
                   400000),
        codec);
    auto nav_age = age_adapter.build();
    fails += expect(nav_age.dvl_recent && std::abs(nav_age.dvl_age_s - 0.3) < 1e-6,
                    "DVL age is computed from HIL timestamps") ? 0 : 1;
    fails += expect(nav_age.gps_valid &&
                        std::abs(nav_age.measurements.gps_position_ned.meta.age_s - 0.3) < 1e-6,
                    "GPS age is computed from HIL timestamps") ? 0 : 1;
    fails += expect(nav_age.last_gps.eph == 123 &&
                        nav_age.last_gps.epv == 234 &&
                        nav_age.last_gps.vel == 345 &&
                        nav_age.last_gps.cog == 9000 &&
                        nav_age.last_gps.satellites_visible == 11,
                    "GPS quality fields are parsed and retained") ? 0 : 1;
    fails += expect(
                        std::abs(nav_age.measurements.gps_position_ned.covariance(0, 0) - 1.5129) < 1e-9 &&
                            std::abs(nav_age.measurements.gps_position_ned.covariance(1, 1) - 1.5129) < 1e-9 &&
                            std::abs(nav_age.measurements.gps_position_ned.covariance(2, 2) - 5.4756) < 1e-9,
                        "GPS eph/epv are converted from centimetre accuracy to position variance") ? 0 : 1;
    fails += expect(nav_age.gps.has_value() && nav_age.gps->has_velocity,
                    "GPS speed availability follows the HIL_GPS sentinel, not nonzero components") ? 0 : 1;
    fails += expect(nav_age.truth_valid &&
                        std::abs(nav_age.measurements.truth_heading_debug.meta.age_s - 0.3) < 1e-6,
                    "truth age is computed from HIL timestamps") ? 0 : 1;

    age_adapter.begin_cycle();
    age_adapter.ingest_frame(
        hil_sensor(0.0f, 0.0f, -9.80665f,
                   kHilFieldsAccel | kHilFieldsGyro,
                   25.0f, 0.0f, 43.30127f,
                   700000),
        codec);
    nav_age = age_adapter.build();
    fails += expect(!nav_age.dvl_recent && !nav_age.truth_valid,
                    "HIL-time timeout expires old DVL and truth samples") ? 0 : 1;

    hydrox::SensorAdapter water_dvl_adapter;
    water_dvl_adapter.begin_cycle();
    water_dvl_adapter.ingest_frame(
        hil_dvl(100000, hydrox::DvlTrackingMode::WaterTrack), codec);
    water_dvl_adapter.ingest_frame(
        hil_sensor(0.0f, 0.0f, -9.80665f,
                   kHilFieldsAccel | kHilFieldsGyro,
                   25.0f, 0.0f, 43.30127f,
                   100000),
        codec);
    const auto water_nav = water_dvl_adapter.build();
    fails += expect(water_nav.water_dvl_recent && water_nav.water_dvl.has_value() &&
                        !water_nav.dvl_recent && !water_nav.dvl.has_value(),
                    "water-track DVL is retained separately from EKF bottom-track input") ? 0 : 1;
    fails += expect(!water_nav.measurements.dvl_velocity_body.meta.valid &&
                        water_nav.measurements.water_dvl_velocity_body.meta.valid &&
                        !water_nav.dvl_altitude_valid &&
                        std::isnan(water_nav.dvl_altitude_m),
                    "water-track DVL uses its own velocity channel and never provides altitude") ? 0 : 1;

    hydrox::SensorAdapter stationary_adapter;
    stationary_adapter.begin_cycle();
    stationary_adapter.ingest_frame(hil_gps(100000, 3, 123, 234, 0), codec);
    stationary_adapter.ingest_frame(
        hil_sensor(0.0f, 0.0f, -9.80665f,
                   kHilFieldsAccel | kHilFieldsGyro,
                   25.0f, 0.0f, 43.30127f,
                   100000),
        codec);
    const auto stationary_nav = stationary_adapter.build();
    fails += expect(stationary_nav.gps.has_value() && stationary_nav.gps->has_velocity,
                    "A stationary zero-speed GPS measurement remains valid") ? 0 : 1;

    hydrox::SensorAdapter fallback_adapter;
    fallback_adapter.begin_cycle();
    fallback_adapter.ingest_frame(
        hil_gps(100000, 3, UINT16_MAX, UINT16_MAX, UINT16_MAX),
        codec);
    fallback_adapter.ingest_frame(
        hil_sensor(0.0f, 0.0f, -9.80665f,
                   kHilFieldsAccel | kHilFieldsGyro,
                   25.0f, 0.0f, 43.30127f,
                   100000),
        codec);
    const auto fallback_nav = fallback_adapter.build();
    fails += expect(
                        std::abs(fallback_nav.measurements.gps_position_ned.covariance(0, 0) - 0.02) < 1e-12 &&
                            std::abs(fallback_nav.measurements.gps_position_ned.covariance(2, 2) - 4.0) < 1e-12,
                        "Unknown GPS accuracy falls back to configured position variance") ? 0 : 1;
    fails += expect(fallback_nav.gps.has_value() && !fallback_nav.gps->has_velocity,
                    "UINT16_MAX marks GPS speed unavailable") ? 0 : 1;

    // Platform policy is a semantic gate in addition to raw sensor validity.
    hydrox::SensorAdapter uuv_adapter{
        hydrox::SensorAdapter::Params{hydrox::VehicleClass::UUV}};
    uuv_adapter.begin_cycle();
    uuv_adapter.ingest_frame(hil_dvl(100000), codec);
    uuv_adapter.ingest_frame(hil_gps(100000), codec);
    uuv_adapter.ingest_frame(
        hil_sensor(0.0f, 0.0f, -9.80665f,
                   kHilFieldsAccel | kHilFieldsGyro |
                       kHilFieldsMag | kHilFieldAbsPressure,
                   25.0f, 0.0f, 43.30127f, 100000),
        codec);
    const auto uuv_nav = uuv_adapter.build();
    fails += expect(
        uuv_nav.measurements.depth.meta.valid &&
            uuv_nav.measurements.depth.meta.source ==
                hydrox::NavMeasurementSource::Depth &&
            uuv_nav.dvl.has_value() &&
            uuv_nav.gps.has_value() &&
            !uuv_nav.gps->has_altitude,
        "UUV profile fuses pressure depth and DVL but not GPS altitude") ? 0 : 1;

    hydrox::SensorAdapter usv_adapter{
        hydrox::SensorAdapter::Params{hydrox::VehicleClass::USV}};
    usv_adapter.begin_cycle();
    usv_adapter.ingest_frame(hil_gps(100000), codec);
    usv_adapter.ingest_frame(
        hil_sensor(0.0f, 0.0f, -9.80665f,
                   kHilFieldsAccel | kHilFieldsGyro | kHilFieldsMag,
                   25.0f, 0.0f, 43.30127f, 100000),
        codec);
    const auto usv_nav = usv_adapter.build();
    fails += expect(
        usv_nav.measurements.depth.meta.valid &&
            usv_nav.measurements.depth.meta.source ==
                hydrox::NavMeasurementSource::SurfaceConstraint &&
            std::abs(usv_nav.measurements.depth.value) < 1e-12 &&
            usv_nav.gps.has_value() &&
            !usv_nav.gps->has_altitude,
        "USV profile applies a surface constraint without pressure data") ? 0 : 1;

    hydrox::SensorAdapter ugv_adapter{
        hydrox::SensorAdapter::Params{hydrox::VehicleClass::UGV_DIFFERENTIAL}};
    ugv_adapter.begin_cycle();
    ugv_adapter.ingest_frame(hil_dvl(100000), codec);
    ugv_adapter.ingest_frame(hil_wheel_odometry(100000), codec);
    ugv_adapter.ingest_frame(hil_gps(100000), codec);
    ugv_adapter.ingest_frame(
        hil_sensor(0.0f, 0.0f, -9.80665f,
                   kHilFieldsAccel | kHilFieldsGyro | kHilFieldsMag,
                   25.0f, 0.0f, 43.30127f, 100000),
        codec);
    const auto ugv_nav = ugv_adapter.build();
    fails += expect(
        ugv_nav.wheel_odometry_recent &&
            ugv_nav.measurements.wheel_odometry_velocity_body.meta.valid &&
            !ugv_nav.dvl.has_value() && !ugv_nav.water_dvl.has_value() &&
            ugv_nav.measurements.depth.meta.source ==
                hydrox::NavMeasurementSource::SurfaceConstraint,
        "UGV profile fuses wheel odometry and rejects DVL with a ground constraint") ? 0 : 1;

    hydrox::SensorAdapter uav_adapter{
        hydrox::SensorAdapter::Params{hydrox::VehicleClass::UAV_MULTIROTOR}};
    uav_adapter.begin_cycle();
    uav_adapter.ingest_frame(hil_dvl(100000), codec);
    uav_adapter.ingest_frame(
        hil_gps(100000, 3, UINT16_MAX, UINT16_MAX, 345), codec);
    uav_adapter.ingest_frame(
        hil_sensor(0.0f, 0.0f, -9.80665f,
                   kHilFieldsAccel | kHilFieldsGyro |
                       kHilFieldsMag | kHilFieldAbsPressure,
                   25.0f, 0.0f, 43.30127f, 100000),
        codec);
    const auto uav_nav = uav_adapter.build();
    fails += expect(
        !uav_nav.depth_m.has_value() &&
            !uav_nav.measurements.depth.meta.valid &&
            uav_nav.gps.has_value() &&
            uav_nav.gps->has_altitude &&
            !uav_nav.dvl.has_value() &&
            !uav_nav.water_dvl.has_value(),
        "UAV profile fuses GPS altitude and rejects depth/DVL observations") ? 0 : 1;
    fails += expect(
        std::abs(
            uav_nav.measurements.gps_position_ned.covariance(0, 0) -
            hydrox::estimation_profile_for(
                hydrox::VehicleClass::UAV_MULTIROTOR).ekf.r_gps_xy) < 1e-12,
        "UAV profile tuning supplies GPS fallback covariance") ? 0 : 1;

    if (fails == 0)
        std::printf("test_sensor_adapter: all checks passed\n");
    return fails == 0 ? 0 : 1;
}
