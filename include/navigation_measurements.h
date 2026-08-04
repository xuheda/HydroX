#pragma once

#include <Eigen/Core>

namespace hydrox
{
    enum class NavMeasurementSource
    {
        None,
        Imu,
        Depth,
        Dvl,
        Gps,
        Magnetometer,
        SurfaceConstraint,
        TruthDebug,
    };

    enum class NavMeasurementFrame
    {
        None,
        BodyFRD,
        WorldNED,
        Scalar,
    };

    struct NavMeasurementMeta
    {
        bool valid = false;
        double timestamp_s = 0.0;
        double age_s = -1.0;
        NavMeasurementSource source = NavMeasurementSource::None;
        NavMeasurementFrame frame = NavMeasurementFrame::None;
    };

    struct ScalarMeasurement
    {
        double value = 0.0;
        double variance = 0.0;
        NavMeasurementMeta meta{};
    };

    struct Vector3Measurement
    {
        Eigen::Vector3d value = Eigen::Vector3d::Zero();
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        NavMeasurementMeta meta{};
    };

    struct NavigationMeasurements
    {
        Vector3Measurement accel_body;
        Vector3Measurement gyro_body;
        Vector3Measurement mag_body;
        /** Bottom-track DVL: body velocity relative to the sea floor. */
        Vector3Measurement dvl_velocity_body;
        /** Water-track DVL: body velocity relative to the water mass. */
        Vector3Measurement water_dvl_velocity_body;
        Vector3Measurement gps_position_ned;
        /** GPS ground velocity in the world NED frame. */
        Vector3Measurement gps_velocity_ned;
        ScalarMeasurement depth;
        ScalarMeasurement truth_heading_debug;
    };
} // namespace hydrox
