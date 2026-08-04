// Copyright (c) 2026 OceanX. Author: xuheda
#include "geodesy.h"
#include "sensor_adapter.h"

#include <algorithm>
#include <cmath>

namespace hydrox
{
    namespace
    {
        constexpr uint32_t kHilFieldXacc = 1u << 0;
        constexpr uint32_t kHilFieldYacc = 1u << 1;
        constexpr uint32_t kHilFieldZacc = 1u << 2;
        constexpr uint32_t kHilFieldsAccel =
            kHilFieldXacc | kHilFieldYacc | kHilFieldZacc;
        constexpr uint32_t kHilFieldXgyro = 1u << 3;
        constexpr uint32_t kHilFieldYgyro = 1u << 4;
        constexpr uint32_t kHilFieldZgyro = 1u << 5;
        constexpr uint32_t kHilFieldsGyro =
            kHilFieldXgyro | kHilFieldYgyro | kHilFieldZgyro;
        constexpr uint32_t kHilFieldXmag = 1u << 6;
        constexpr uint32_t kHilFieldYmag = 1u << 7;
        constexpr uint32_t kHilFieldZmag = 1u << 8;
        constexpr uint32_t kHilFieldsMag =
            kHilFieldXmag | kHilFieldYmag | kHilFieldZmag;
        constexpr uint32_t kHilFieldAbsPressure = 1u << 9;
        constexpr uint16_t kGpsUnknownU16 = 0xFFFFu;

        bool finite3(const Eigen::Vector3d &v)
        {
            return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
        }

        Eigen::Matrix3d diagonal3(double x, double y, double z)
        {
            Eigen::Matrix3d out = Eigen::Matrix3d::Zero();
            out(0, 0) = x;
            out(1, 1) = y;
            out(2, 2) = z;
            return out;
        }

        double hil_time_s(uint64_t time_usec)
        {
            return static_cast<double>(time_usec) * 1e-6;
        }

        double measurement_age_s(double now_s, double measurement_time_s)
        {
            if (now_s <= 0.0 || measurement_time_s < 0.0)
                return -1.0;
            return std::max(0.0, now_s - measurement_time_s);
        }

        double gps_accuracy_variance(
            uint16_t accuracy_cm,
            double fallback_variance,
            const SensorAdapter::Params &p)
        {
            const double min_variance =
                std::max(1.0e-9, std::min(p.gps_min_variance, p.gps_max_variance));
            const double max_variance =
                std::max(min_variance, p.gps_max_variance);
            if (accuracy_cm == kGpsUnknownU16)
                return std::clamp(fallback_variance, min_variance, max_variance);

            const double accuracy_m = static_cast<double>(accuracy_cm) * 0.01;
            const double variance = accuracy_m * accuracy_m;
            if (!std::isfinite(variance))
                return std::clamp(fallback_variance, min_variance, max_variance);
            return std::clamp(variance, min_variance, max_variance);
        }

        bool geodetic_to_local_ned(
            int32_t lat_e7,
            int32_t lon_e7,
            int32_t alt_mm,
            const SensorAdapter::Params &p,
            Eigen::Vector3d &out_position_ned)
        {
            const geodesy::Wgs84AeqdFrame frame{
                p.gps_origin_lat_deg,
                p.gps_origin_lon_deg,
                p.gps_origin_altitude_msl_m,
                p.gps_geodetic_max_radius_m};
            double north_m = 0.0;
            double east_m = 0.0;
            double down_m = 0.0;
            if (!geodesy::geodetic_to_local_ned(
                    frame,
                    static_cast<double>(lat_e7) * 1e-7,
                    static_cast<double>(lon_e7) * 1e-7,
                    static_cast<double>(alt_mm) * 0.001,
                    north_m,
                    east_m,
                    down_m))
            {
                return false;
            }

            out_position_ned = Eigen::Vector3d(north_m, east_m, down_m);
            return true;
        }
    } // namespace

    SensorAdapter::SensorAdapter(const Params &p) : _p(p) {}

    void SensorAdapter::reset()
    {
        _imu = {};
        _last_gps = {};
        _last_dvl = {};
        _last_wheel_odometry = {};
        _last_truth = {};
        _last_passive_sonar = {};
        _last_acoustic_neighbors = {};
        _last_rangefinder_scan = {};
        _got_imu_this_cycle = false;
        _gps_valid = false;
        _truth_valid = false;
        _gps_new_this_cycle = false;
        _last_gps_time_s = -1.0;
        _last_dvl_time_s = -1.0;
        _last_wheel_odometry_time_s = -1.0;
        _last_truth_time_s = -1.0;
    }

    void SensorAdapter::begin_cycle()
    {
        _got_imu_this_cycle = false;
        _gps_new_this_cycle = false;
    }

    void SensorAdapter::ingest_frame(
        const MavFrame &frame,
        const MavlinkHIL &codec)
    {
        switch (frame.msg_id)
        {
        case MSGID_HIL_SENSOR:
            _imu = codec.parse_hil_sensor(frame);
            _got_imu_this_cycle = true;
            break;
        case MSGID_HIL_GPS:
            _last_gps = codec.parse_hil_gps(frame);
            _gps_valid = (_last_gps.fix_type >= 3);
            if (_gps_valid)
            {
                _gps_new_this_cycle = true;
                _last_gps_time_s = hil_time_s(_last_gps.time_usec);
            }
            break;
        case MSGID_HIL_DVL:
            _last_dvl = codec.parse_hil_dvl(frame);
            if (_last_dvl.velocity_valid())
                _last_dvl_time_s = hil_time_s(_last_dvl.time_usec);
            break;
        case MSGID_HIL_WHEEL_ODOMETRY:
            _last_wheel_odometry = codec.parse_hil_wheel_odometry(frame);
            if (_last_wheel_odometry.velocity_valid())
                _last_wheel_odometry_time_s = hil_time_s(_last_wheel_odometry.time_usec);
            break;
        case MSGID_HIL_TRUTH_STATE:
            _last_truth = codec.parse_hil_truth_state(frame);
            _truth_valid = _last_truth.valid;
            if (_truth_valid)
                _last_truth_time_s = hil_time_s(_last_truth.time_usec);
            break;
        case MSGID_HIL_PASSIVE_SONAR:
            _last_passive_sonar = codec.parse_hil_passive_sonar(frame);
            break;
        case MSGID_HIL_ACOUSTIC_NEIGHBORS:
            _last_acoustic_neighbors =
                codec.parse_hil_acoustic_neighbors(frame);
            break;
        case MSGID_HIL_RANGEFINDER_SCAN:
            _last_rangefinder_scan =
                codec.parse_hil_rangefinder_scan(frame);
            break;
        default:
            break;
        }
    }

    NavigationInput SensorAdapter::build() const
    {
        NavigationInput out;
        out.imu = _imu;
        out.last_gps = _last_gps;
        out.last_dvl = _last_dvl;
        out.wheel_odometry = _last_wheel_odometry;
        out.truth = _last_truth;
        out.passive_sonar = _last_passive_sonar;
        out.acoustic_neighbors = _last_acoustic_neighbors;
        out.rangefinder_scan = _last_rangefinder_scan;
        const double imu_timestamp_s = hil_time_s(_imu.time_usec);
        const double dvl_age_s = measurement_age_s(imu_timestamp_s, _last_dvl_time_s);
        const double wheel_odometry_age_s = measurement_age_s(
            imu_timestamp_s, _last_wheel_odometry_time_s);
        const double gps_age_s = measurement_age_s(imu_timestamp_s, _last_gps_time_s);
        const double truth_age_s = measurement_age_s(imu_timestamp_s, _last_truth_time_s);
        out.truth_valid =
            _truth_valid &&
            truth_age_s >= 0.0 &&
            truth_age_s <= _p.truth_timeout_s;
        out.got_imu = _got_imu_this_cycle;
        out.accel_body = Eigen::Vector3d(_imu.xacc, _imu.yacc, _imu.zacc);
        out.omega_body = Eigen::Vector3d(_imu.xgyro, _imu.ygyro, _imu.zgyro);
        out.mag_body = Eigen::Vector3d(_imu.xmag, _imu.ymag, _imu.zmag);
        // Accel mechanization is enabled only when explicitly forced on, or when
        // UE marks all accel axes valid and the values pass a sanity gate. This
        // preserves kinematic fallback for older HIL streams without hiding the
        // full EKF path when modern HIL_SENSOR accel fields are present.
        const bool accel_fields_ok =
            (_imu.fields_updated & kHilFieldsAccel) == kHilFieldsAccel;
        const bool accel_values_ok =
            finite3(out.accel_body) &&
            out.accel_body.norm() <= _p.accel_max_norm;
        out.have_accel =
            (_p.accel_mode == AccelMode::On && accel_values_ok) ||
            (_p.accel_mode == AccelMode::Auto && accel_fields_ok && accel_values_ok);
        out.measurements.accel_body.value = out.accel_body;
        out.measurements.accel_body.covariance =
            Eigen::Matrix3d::Identity() * _p.accel_variance;
        out.measurements.accel_body.meta.valid = out.got_imu && out.have_accel;
        out.measurements.accel_body.meta.timestamp_s = imu_timestamp_s;
        out.measurements.accel_body.meta.age_s = out.got_imu ? 0.0 : -1.0;
        out.measurements.accel_body.meta.source = NavMeasurementSource::Imu;
        out.measurements.accel_body.meta.frame = NavMeasurementFrame::BodyFRD;

        const bool gyro_fields_ok =
            (_imu.fields_updated & kHilFieldsGyro) == kHilFieldsGyro;
        const bool gyro_values_ok = finite3(out.omega_body);
        out.measurements.gyro_body.value = out.omega_body;
        out.measurements.gyro_body.covariance =
            Eigen::Matrix3d::Identity() * _p.gyro_variance;
        out.measurements.gyro_body.meta.valid = out.got_imu && gyro_fields_ok && gyro_values_ok;
        out.measurements.gyro_body.meta.timestamp_s = imu_timestamp_s;
        out.measurements.gyro_body.meta.age_s = out.got_imu ? 0.0 : -1.0;
        out.measurements.gyro_body.meta.source = NavMeasurementSource::Imu;
        out.measurements.gyro_body.meta.frame = NavMeasurementFrame::BodyFRD;

        const bool mag_fields_ok =
            (_imu.fields_updated & kHilFieldsMag) == kHilFieldsMag;
        const bool mag_values_ok =
            finite3(out.mag_body) &&
            out.mag_body.norm() >= 1.0 &&
            out.mag_body.norm() <= 1000.0;
        out.have_mag =
            _p.estimation_profile.fuse_magnetometer &&
            mag_fields_ok &&
            mag_values_ok;
        out.measurements.mag_body.value = out.mag_body;
        out.measurements.mag_body.covariance =
            Eigen::Matrix3d::Identity() * _p.mag_variance;
        out.measurements.mag_body.meta.valid = out.got_imu && out.have_mag;
        out.measurements.mag_body.meta.timestamp_s = imu_timestamp_s;
        out.measurements.mag_body.meta.age_s = out.got_imu ? 0.0 : -1.0;
        out.measurements.mag_body.meta.source = NavMeasurementSource::Magnetometer;
        out.measurements.mag_body.meta.frame = NavMeasurementFrame::BodyFRD;

        const bool dvl_velocity_recent =
            _last_dvl.velocity_valid() &&
            dvl_age_s >= 0.0 &&
            dvl_age_s <= _p.dvl_timeout_s;
        const bool bottom_dvl_recent =
            _p.estimation_profile.fuse_bottom_track_dvl &&
            dvl_velocity_recent &&
            _last_dvl.is_bottom_track();
        const bool water_dvl_recent =
            _p.estimation_profile.fuse_relative_medium_velocity &&
            dvl_velocity_recent &&
            _last_dvl.is_water_track();
        out.dvl_age_s = dvl_age_s;
        out.dvl_altitude_valid = bottom_dvl_recent && _last_dvl.altitude_valid();
        if (out.dvl_altitude_valid)
            out.dvl_altitude_m = _last_dvl.altitude_m;

        const auto MakeDvlMeasurement = [this]()
        {
            DVLMeasurement dm;
            dm.vel_x = _last_dvl.vx;
            dm.vel_y = _last_dvl.vy;
            dm.vel_z = _last_dvl.vz;
            dm.beam_valid = 4;
            dm.timestamp = static_cast<double>(_last_dvl.time_usec) * 1e-6;
            dm.tracking_mode = static_cast<uint8_t>(_last_dvl.tracking_mode);
            dm.altitude_valid = _last_dvl.altitude_valid();
            if (dm.altitude_valid)
                dm.altitude_m = _last_dvl.altitude_m;
            return dm;
        };
        if (bottom_dvl_recent)
        {
            out.dvl = MakeDvlMeasurement();
        }
        else if (water_dvl_recent)
        {
            out.water_dvl = MakeDvlMeasurement();
        }
        out.dvl_recent = bottom_dvl_recent;
        out.water_dvl_recent = water_dvl_recent;
        out.measurements.dvl_velocity_body.value =
            Eigen::Vector3d(_last_dvl.vx, _last_dvl.vy, _last_dvl.vz);
        out.measurements.dvl_velocity_body.covariance =
            Eigen::Matrix3d::Identity() * _p.dvl_velocity_variance;
        out.measurements.dvl_velocity_body.meta.valid = bottom_dvl_recent;
        out.measurements.dvl_velocity_body.meta.timestamp_s =
            static_cast<double>(_last_dvl.time_usec) * 1e-6;
        out.measurements.dvl_velocity_body.meta.age_s = out.dvl_age_s;
        out.measurements.dvl_velocity_body.meta.source = NavMeasurementSource::Dvl;
        out.measurements.dvl_velocity_body.meta.frame = NavMeasurementFrame::BodyFRD;

        out.measurements.water_dvl_velocity_body.value =
            Eigen::Vector3d(_last_dvl.vx, _last_dvl.vy, _last_dvl.vz);
        out.measurements.water_dvl_velocity_body.covariance =
            Eigen::Matrix3d::Identity() * _p.water_dvl_velocity_variance;
        out.measurements.water_dvl_velocity_body.meta.valid = water_dvl_recent;
        out.measurements.water_dvl_velocity_body.meta.timestamp_s =
            static_cast<double>(_last_dvl.time_usec) * 1e-6;
        out.measurements.water_dvl_velocity_body.meta.age_s = out.dvl_age_s;
        out.measurements.water_dvl_velocity_body.meta.source = NavMeasurementSource::Dvl;
        out.measurements.water_dvl_velocity_body.meta.frame = NavMeasurementFrame::BodyFRD;

        const bool wheel_odometry_recent =
            _p.estimation_profile.fuse_wheel_odometry &&
            _last_wheel_odometry.velocity_valid() &&
            wheel_odometry_age_s >= 0.0 &&
            wheel_odometry_age_s <= _p.wheel_odometry_timeout_s;
        out.wheel_odometry_recent = wheel_odometry_recent;
        out.wheel_odometry_age_s = wheel_odometry_age_s;
        out.measurements.wheel_odometry_velocity_body.value = Eigen::Vector3d(
            _last_wheel_odometry.forward_mps, 0.0, 0.0);
        const double wheel_velocity_variance =
            _last_wheel_odometry.velocity_variance > 0.0f
                ? static_cast<double>(_last_wheel_odometry.velocity_variance)
                : _p.wheel_odometry_velocity_variance;
        out.measurements.wheel_odometry_velocity_body.covariance = diagonal3(
            wheel_velocity_variance,
            _p.wheel_nonholonomic_variance,
            _p.wheel_nonholonomic_variance);
        out.measurements.wheel_odometry_velocity_body.meta.valid = wheel_odometry_recent;
        out.measurements.wheel_odometry_velocity_body.meta.timestamp_s =
            hil_time_s(_last_wheel_odometry.time_usec);
        out.measurements.wheel_odometry_velocity_body.meta.age_s = wheel_odometry_age_s;
        out.measurements.wheel_odometry_velocity_body.meta.source =
            NavMeasurementSource::WheelOdometry;
        out.measurements.wheel_odometry_velocity_body.meta.frame =
            NavMeasurementFrame::BodyFRD;

        Eigen::Vector3d gps_position_ned = Eigen::Vector3d::Zero();
        const bool gps_projection_valid = geodetic_to_local_ned(
            _last_gps.lat,
            _last_gps.lon,
            _last_gps.alt,
            _p,
            gps_position_ned);
        const bool gps_recent =
            _p.estimation_profile.fuse_gps_position &&
            _gps_valid &&
            gps_projection_valid &&
            gps_age_s >= 0.0 &&
            gps_age_s <= _p.gps_timeout_s;
        out.gps_valid = gps_recent;
        if (_gps_new_this_cycle && gps_recent)
        {
            GPSMeasurement gm;
            gm.pos_n = gps_position_ned.x();
            gm.pos_e = gps_position_ned.y();
            gm.pos_d = gps_position_ned.z();
            gm.vel_n = static_cast<double>(_last_gps.vn) * 0.01;
            gm.vel_e = static_cast<double>(_last_gps.ve) * 0.01;
            gm.vel_d = static_cast<double>(_last_gps.vd) * 0.01;
            // HIL_GPS uses UINT16_MAX as the unavailable ground-speed sentinel.
            // A measured zero is still valid and must not be treated as missing.
            gm.has_velocity =
                _p.estimation_profile.fuse_gps_velocity &&
                _last_gps.vel != kGpsUnknownU16;
            gm.has_altitude =
                _p.estimation_profile.vertical_aid ==
                VerticalAidMode::GpsAltitude;
            gm.timestamp = static_cast<double>(_last_gps.time_usec) * 1e-6;
            out.gps = gm;
        }
        const double gps_timestamp_s =
            static_cast<double>(_last_gps.time_usec) * 1e-6;
        out.measurements.gps_position_ned.value = gps_position_ned;
        const double gps_xy_variance = gps_accuracy_variance(
            _last_gps.eph,
            _p.gps_xy_variance,
            _p);
        const double gps_z_variance = gps_accuracy_variance(
            _last_gps.epv,
            _p.gps_z_variance,
            _p);
        out.measurements.gps_position_ned.covariance =
            diagonal3(gps_xy_variance, gps_xy_variance, gps_z_variance);
        out.measurements.gps_position_ned.meta.valid = gps_recent;
        out.measurements.gps_position_ned.meta.timestamp_s = gps_timestamp_s;
        out.measurements.gps_position_ned.meta.age_s = gps_age_s;
        out.measurements.gps_position_ned.meta.source = NavMeasurementSource::Gps;
        out.measurements.gps_position_ned.meta.frame = NavMeasurementFrame::WorldNED;

        out.measurements.gps_velocity_ned.value = Eigen::Vector3d(
            static_cast<double>(_last_gps.vn) * 0.01,
            static_cast<double>(_last_gps.ve) * 0.01,
            static_cast<double>(_last_gps.vd) * 0.01);
        out.measurements.gps_velocity_ned.covariance =
            Eigen::Matrix3d::Identity() * _p.gps_velocity_variance;
        out.measurements.gps_velocity_ned.meta.valid =
            _p.estimation_profile.fuse_gps_velocity &&
            gps_recent &&
            _gps_new_this_cycle &&
            _last_gps.vel != kGpsUnknownU16;
        out.measurements.gps_velocity_ned.meta.timestamp_s = gps_timestamp_s;
        out.measurements.gps_velocity_ned.meta.age_s = gps_age_s;
        out.measurements.gps_velocity_ned.meta.source = NavMeasurementSource::Gps;
        out.measurements.gps_velocity_ned.meta.frame = NavMeasurementFrame::WorldNED;

        switch (_p.estimation_profile.vertical_aid)
        {
        case VerticalAidMode::PressureDepth:
            out.depth_m = pressure_to_depth(_imu.abs_pressure);
            out.measurements.depth.variance = _p.depth_variance;
            out.measurements.depth.meta.valid =
                out.got_imu &&
                ((_imu.fields_updated & kHilFieldAbsPressure) ==
                 kHilFieldAbsPressure);
            out.measurements.depth.meta.source = NavMeasurementSource::Depth;
            break;
        case VerticalAidMode::SurfaceConstraint:
            out.depth_m = _p.estimation_profile.surface_depth_m;
            out.measurements.depth.variance =
                _p.estimation_profile.surface_constraint_variance;
            out.measurements.depth.meta.valid = out.got_imu;
            out.measurements.depth.meta.source =
                NavMeasurementSource::SurfaceConstraint;
            break;
        case VerticalAidMode::GpsAltitude:
        default:
            // Air vehicles receive vertical position through GPS altitude.
            out.depth_m = std::nullopt;
            out.measurements.depth.variance = _p.depth_variance;
            out.measurements.depth.meta.valid = false;
            out.measurements.depth.meta.source = NavMeasurementSource::None;
            break;
        }
        out.measurements.depth.value = out.depth_m.value_or(0.0);
        out.measurements.depth.meta.timestamp_s = imu_timestamp_s;
        out.measurements.depth.meta.age_s = out.got_imu ? 0.0 : -1.0;
        out.measurements.depth.meta.frame = NavMeasurementFrame::Scalar;

        out.measurements.truth_heading_debug.value = _last_truth.eta[5];
        out.measurements.truth_heading_debug.variance = _p.truth_heading_variance;
        out.measurements.truth_heading_debug.meta.valid = out.truth_valid;
        out.measurements.truth_heading_debug.meta.timestamp_s =
            static_cast<double>(_last_truth.time_usec) * 1e-6;
        out.measurements.truth_heading_debug.meta.age_s = truth_age_s;
        out.measurements.truth_heading_debug.meta.source = NavMeasurementSource::TruthDebug;
        out.measurements.truth_heading_debug.meta.frame = NavMeasurementFrame::Scalar;

        return out;
    }
} // namespace hydrox
