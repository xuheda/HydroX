// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once

#include "estimation_profile.h"
#include "mavlink_hil.h"
#include "navigation_measurements.h"
#include "types.h"

#include <Eigen/Core>
#include <optional>

namespace hydrox
{
    enum class AccelMode
    {
        Off,
        Auto,
        On,
    };

    struct NavigationInput
    {
        HilSensorMsg imu{};
        HilGpsMsg last_gps{};
        HilDvlMsg last_dvl{};
        HilWheelOdometryMsg wheel_odometry{};
        HilTruthStateMsg truth{};
        HilPassiveSonarBearingMsg passive_sonar{};
        HilAcousticNeighborsMsg acoustic_neighbors{};
        HilRangeFinderScanMsg rangefinder_scan{};
        NavigationMeasurements measurements{};

        Eigen::Vector3d accel_body{0.0, 0.0, 0.0};
        Eigen::Vector3d omega_body{0.0, 0.0, 0.0};
        Eigen::Vector3d mag_body{0.0, 0.0, 0.0};
        bool got_imu = false;
        bool have_accel = false;
        bool have_mag = false;

        /** Bottom-track DVL measurement eligible for EKF velocity fusion. */
        std::optional<DVLMeasurement> dvl;
        /** Water-relative DVL measurement eligible for current-aware EKF fusion. */
        std::optional<DVLMeasurement> water_dvl;
        std::optional<GPSMeasurement> gps;
        std::optional<double> depth_m;

        /** Recent bottom-track measurement; this is the EKF DVL-valid signal. */
        bool dvl_recent = false;
        bool water_dvl_recent = false;
        bool wheel_odometry_recent = false;
        double wheel_odometry_age_s = -1.0;
        bool dvl_altitude_valid = false;
        float dvl_altitude_m = std::numeric_limits<float>::quiet_NaN();
        double dvl_age_s = -1.0;
        bool gps_valid = false;
        bool truth_valid = false;
    };

    class SensorAdapter
    {
    public:
        struct Params
        {
            EstimationProfile estimation_profile =
                estimation_profile_for(VehicleClass::UUV);
            AccelMode accel_mode = AccelMode::Auto;
            double dvl_timeout_s = 0.5;
            double wheel_odometry_timeout_s = 0.25;
            double gps_timeout_s = 3.0;
            double truth_timeout_s = 0.5;
            double accel_max_norm = 80.0;
            double accel_variance = 1.0;
            double gyro_variance = 1.0e-4;
            double mag_variance = 6.25;
            double dvl_velocity_variance = 0.01;
            double water_dvl_velocity_variance = 0.01;
            double wheel_odometry_velocity_variance = 0.04;
            double wheel_nonholonomic_variance = 0.01;
            double depth_variance = 0.001;
            double gps_xy_variance = 0.02;
            double gps_z_variance = 4.0;
            double gps_velocity_variance = 0.04;
            double truth_heading_variance = 0.0025;
            double gps_origin_lat_deg = 0.0;
            double gps_origin_lon_deg = 0.0;
            /** MAVLink HIL_GPS altitude datum: mean sea level, metres. */
            double gps_origin_altitude_msl_m = 0.0;
            double gps_geodetic_max_radius_m = 10000.0;
            double gps_min_variance = 1.0e-4;
            double gps_max_variance = 1.0e6;

            Params() = default;
            Params(
                const EstimationProfile &profile,
                AccelMode mode = AccelMode::Auto)
                : estimation_profile(profile),
                  accel_mode(mode),
                  dvl_velocity_variance(profile.ekf.r_dvl),
                  water_dvl_velocity_variance(
                      profile.ekf.r_relative_medium_velocity),
                  depth_variance(profile.ekf.r_vertical),
                  gps_xy_variance(profile.ekf.r_gps_xy),
                  gps_z_variance(profile.ekf.r_gps_z),
                  gps_velocity_variance(profile.ekf.r_gps_velocity)
            {
            }
            Params(
                VehicleClass vehicle_class,
                AccelMode mode = AccelMode::Auto)
                : Params(estimation_profile_for(vehicle_class), mode)
            {
            }
        };

        explicit SensorAdapter(const Params &p = {});

        void reset();
        void begin_cycle();
        void ingest_frame(
            const MavFrame &frame,
            const MavlinkHIL &codec);
        NavigationInput build() const;

    private:
        Params _p;
        HilSensorMsg _imu{};
        HilGpsMsg _last_gps{};
        HilDvlMsg _last_dvl{};
        HilWheelOdometryMsg _last_wheel_odometry{};
        HilTruthStateMsg _last_truth{};
        HilPassiveSonarBearingMsg _last_passive_sonar{};
        HilAcousticNeighborsMsg _last_acoustic_neighbors{};
        HilRangeFinderScanMsg _last_rangefinder_scan{};
        bool _got_imu_this_cycle = false;
        bool _gps_valid = false;
        bool _truth_valid = false;
        bool _gps_new_this_cycle = false;
        double _last_gps_time_s = -1.0;
        double _last_dvl_time_s = -1.0;
        double _last_wheel_odometry_time_s = -1.0;
        double _last_truth_time_s = -1.0;
    };
} // namespace hydrox
