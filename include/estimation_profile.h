#pragma once

#include "types.h"

namespace hydrox
{
    /** Selects the authoritative vertical observation for a vehicle class. */
    enum class VerticalAidMode : uint8_t
    {
        PressureDepth = 0,
        SurfaceConstraint = 1,
        GpsAltitude = 2,
        BarometerAndGps = 3,
    };

    /**
     * EKF tuning kept separate from the common filter implementation.
     *
     * Defaults preserve the historical UUV tuning. Platform profiles override
     * only parameters whose physical meaning changes across domains.
     */
    struct EstimatorTuning
    {
        double q_pos = 0.01;
        double q_att = 0.001;
        double q_vel = 0.1;
        double q_ba = 1e-6;
        double q_bg = 1e-7;
        double q_medium_velocity = 0.01;

        double r_dvl = 0.01;
        double r_relative_medium_velocity = 0.01;
        double r_vertical = 0.001;
        double r_zvu = 0.5;
        double r_gps_xy = 0.02;
        double r_gps_z = 4.0;
        double r_gps_velocity = 0.04;
        double r_level = 1.0;
        double r_heading = 0.0025;

        double level_gate = 0.5;
        // Static accelerometer leveling is ambiguous with real translational
        // acceleration. Large defaults preserve the historical UUV policy;
        // dynamic platform profiles opt into a stationary-only motion gate.
        double level_max_body_speed_mps = 1.0e9;
        double level_max_gyro_radps = 1.0e9;
        double gate_dvl_nis = 16.27;
        double gate_relative_medium_velocity_nis = 16.27;
        double gate_gps_velocity_nis = 16.27;
        double gate_level_nis = 16.27;
        double gate_gps_xy_nis = 13.82;
        double gate_scalar_nis = 10.83;
        // Allow absolute-heading aids to recover independently of depth gates.
        double gate_heading_nis = 10.83;
        // Bottom-track DVL can legitimately move outside a tightly converged
        // velocity gate when a vehicle departs after a long stationary hold.
        // Reacquire only after several time-adjacent, physically consistent
        // samples so an isolated acoustic outlier cannot reset navigation.
        int dvl_velocity_reacquire_samples = 3;
        double dvl_velocity_reacquire_consistency_mps = 2.0;
        double dvl_velocity_reacquire_max_interval_s = 0.5;
        double dvl_velocity_reacquire_max_speed_mps = 20.0;
        // A persistent, internally consistent GPS velocity innovation can be
        // a legitimate navigation-mode transition (for example an air-launched
        // fixed wing released from a frozen spawn hold). Isolated outliers are
        // still rejected; only this many consecutive close measurements may
        // reset horizontal GPS position and body velocity.
        int gps_navigation_reacquire_samples = 3;
        double gps_navigation_reacquire_consistency_mps = 1.5;
        double gps_navigation_reacquire_max_interval_s = 1.0;
        double gps_navigation_reacquire_max_speed_mps = 100.0;

        double min_variance = 1.0e-9;

        double initial_position_variance = 1.0;
        double initial_attitude_variance = 1.0;
        double initial_velocity_variance = 1.0;
        // Calibrated navigation IMU priors (SI): 0.1 m/s^2 and 0.01 rad/s
        // one-sigma bias uncertainty. A 1 rad/s gyro-bias prior lets noisy
        // startup position fixes invent multi-radian/s rotation corrections.
        double initial_accel_bias_variance = 0.01;
        double initial_gyro_bias_variance = 0.0001;
        double initial_medium_velocity_variance = 4.0;
        double medium_velocity_valid_std = 0.5;
    };

    /**
     * Platform policy applied before measurements enter the common EKF.
     *
     * Sensor presence is still determined by the HIL stream. These flags are
     * the second, semantic gate: they prevent a physically inappropriate
     * observation from being fused merely because a message was received.
     */
    struct EstimationProfile
    {
        VehicleClass vehicle_class = VehicleClass::UUV;
        VerticalAidMode vertical_aid = VerticalAidMode::PressureDepth;
        MediumVelocityKind medium_velocity_kind = MediumVelocityKind::WaterCurrent;

        bool estimate_medium_velocity = true;
        bool fuse_bottom_track_dvl = true;
        bool fuse_relative_medium_velocity = true;
        bool fuse_wheel_odometry = false;
        bool fuse_gps_position = true;
        bool fuse_gps_velocity = true;
        bool fuse_magnetometer = true;

        double surface_depth_m = 0.0;
        double surface_constraint_variance = 0.01;
        EstimatorTuning ekf{};
    };

    EstimationProfile estimation_profile_for(VehicleClass vehicle_class);
    const char *vertical_aid_mode_name(VerticalAidMode mode);
    const char *medium_velocity_kind_name(MediumVelocityKind kind);
} // namespace hydrox
