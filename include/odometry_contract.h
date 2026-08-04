#pragma once

#include <algorithm>
#include <cmath>

namespace hydrox::odometry_contract
{
// EKF state order:
//   [N, E, D, roll, pitch, yaw, u, v, w, ...]
// ROS nav_msgs/Odometry order:
//   pose  [N, E, D, roll, pitch, yaw] in map_ned
//   twist [u, v, w, p, q, r] in child BodyFRD
//
// Cross-frame pose/twist covariance is not part of nav_msgs/Odometry. Within
// each published 6x6 matrix, finite values and symmetry are enforced here so
// every publisher follows one contract.
template <typename Matrix>
bool encode_covariances(
    const Matrix &ekf_covariance,
    double (&pose_covariance)[36],
    double (&twist_covariance)[36],
    double angular_rate_variance_rad2_s2 = 0.01)
{
    std::fill_n(pose_covariance, 36, 0.0);
    std::fill_n(twist_covariance, 36, 0.0);

    if (!std::isfinite(angular_rate_variance_rad2_s2) ||
        angular_rate_variance_rad2_s2 < 0.0)
    {
        return false;
    }

    const auto symmetrized = [&](int row, int column)
    {
        return 0.5 * (static_cast<double>(ekf_covariance(row, column)) +
                      static_cast<double>(ekf_covariance(column, row)));
    };

    for (int row = 0; row < 6; ++row)
    {
        for (int column = 0; column < 6; ++column)
        {
            const double value = symmetrized(row, column);
            if (!std::isfinite(value))
                return false;
            pose_covariance[row * 6 + column] = value;
        }
    }

    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            const double value = symmetrized(6 + row, 6 + column);
            if (!std::isfinite(value))
                return false;
            twist_covariance[row * 6 + column] = value;
        }
    }

    for (int axis = 0; axis < 3; ++axis)
        twist_covariance[(axis + 3) * 6 + axis + 3] =
            angular_rate_variance_rad2_s2;

    for (int axis = 0; axis < 6; ++axis)
    {
        // A negative diagonal cannot be a covariance. Small round-off below
        // zero is normalized, while a material violation fails closed.
        double &pose_variance = pose_covariance[axis * 6 + axis];
        double &twist_variance = twist_covariance[axis * 6 + axis];
        if (pose_variance < -1e-12 || twist_variance < -1e-12)
            return false;
        pose_variance = std::max(0.0, pose_variance);
        twist_variance = std::max(0.0, twist_variance);
    }

    return true;
}
} // namespace hydrox::odometry_contract
