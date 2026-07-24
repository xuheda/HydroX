#pragma once
/**
 * ekf.h - Hybrid aided EKF with error states on a nominal navigation state.
 *
 * Ported/refactored from Python hydrox/estimator/ekf.py, rewritten in 2026-06
 * into an 18-dimensional aided navigation filter. It runs mechanized INS
 * propagation only when valid specific force is present; otherwise it falls
 * back to kinematic propagation aided by DVL/ZVU/depth/GPS.
 *
 * State Vector (18-dimensional, double precision, NED / SI):
 *   x = [N, E, D,          // Position (m)
 *         roll, pitch, yaw, // Euler angles ZYX (rad)
 *         u, v, w,          // Body frame velocity (m/s)
 *         b_ax, b_ay, b_az, // Accelerometer bias (m/s^2, body frame)
 *         b_gx, b_gy, b_gz, // Gyroscope bias (rad/s, body frame)
 *         c_N, c_E, c_D]    // Water-current velocity (m/s, world NED)
 *
 * Mechanization (predict, requires specific force f_b):
 *   omega = omega_meas - b_g
 *   f     = f_meas     - b_a
 *   p_dot = R_nb(theta) v_b
 *   theta_dot = J2(phi, theta) omega
 *   v_dot = f + R_nb(theta)^T g_n - omega x v_b      g_n = [0, 0, +g]
 *   b_dot = 0 (random walk)
 *   Convention: When static and level, the accelerometer reading is f_b = [0, 0, -g], at which point v_dot = 0.
 *
 * When a frame does not contain the specific force field (have_accel=false, e.g., legacy UE only sends gyro/DVL ground truth),
 * predict degrades to a kinematic model (v_b is driven by DVL/ZVU updates, without specific force integration),
 * behaving consistently with the pre-rewrite version — ensuring zero regression when UE is not upgraded.
 *
 * Measurement Updates:
 *   z_bottom_dvl = [u, v, w]                 Bottom-track body velocity
 *   z_water_dvl  = v_ground_body - R_nb^T c  Water-relative body velocity
 *   z_zvu   = [0, 0, 0]   Pseudo zero-velocity update (when DVL is invalid, weak constraint)
 *   z_depth = [D]         NED depth
 *   z_gps   = [N, E]      Surface/near-surface GPS position
 *   z_gps_v = R_nb v_b    Surface/near-surface GPS ground velocity
 *   z_level = f_b         Accelerometer leveling measurement (when |norm(f)-g| < threshold, making roll/pitch
 *                         and b_gx/b_gy observable; yaw/b_gz are unobservable via gravity and require
 *                         GPS trajectory, magnetometers, USBL, or a simulated heading aid).
 *   z_heading = yaw       Optional heading/compass-like observation, used by SITL when the simulator supplies one.
 */
#include "navigation_measurements.h"
#include "types.h"
#include <Eigen/Dense>
#include <limits>
#include <optional>

namespace hydrox
{

    static constexpr int EKF_N = 18;
    static constexpr double EKF_G = 9.80665; // Standard gravity (m/s²), g_n = [0, 0, +EKF_G]

    class EKF
    {
    public:
        struct Params
        {
            double q_pos = 0.01;
            double q_att = 0.001;
            double q_vel = 0.1;
            double q_ba = 1e-6;  // Accelerometer bias random walk
            double q_bg = 1e-7;  // Gyroscope bias random walk
            double q_current = 0.01; // Water-current random walk (m/s)^2/s
            double r_dvl = 0.01;
            double r_water_dvl = 0.01;
            double r_depth = 0.001;
            double r_zvu = 0.5;   // Zero-velocity pseudo-measurement noise (effective when DVL is invalid)
            double r_gps_xy = 0.02;
            double r_gps_z = 4.0;
            double r_gps_velocity = 0.04;
            double r_level = 1.0; // Accelerometer leveling measurement noise (m/s²)^2
            double r_heading = 0.0025; // Heading measurement noise (rad^2)
            double level_gate = 0.5; // |norm(f_meas) - g| <= this value (m/s²) to perform leveling update
            double gate_dvl_nis = 16.27;     // Chi-square 3D, ~99.9%
            double gate_water_dvl_nis = 16.27; // Chi-square 3D, ~99.9%
            double gate_gps_velocity_nis = 16.27; // Chi-square 3D, ~99.9%
            double gate_level_nis = 16.27;   // Chi-square 3D, ~99.9%
            double gate_gps_xy_nis = 13.82;  // Chi-square 2D, ~99.9%
            double gate_scalar_nis = 10.83;  // Chi-square 1D, ~99.9%
            double min_variance = 1.0e-9;
            double initial_current_variance = 4.0;
            double current_valid_std = 0.5;
        };

        struct InnovationStats
        {
            uint32_t dvl_accepted = 0;
            uint32_t dvl_rejected = 0;
            uint32_t water_dvl_accepted = 0;
            uint32_t water_dvl_rejected = 0;
            uint32_t depth_accepted = 0;
            uint32_t depth_rejected = 0;
            uint32_t gps_xy_accepted = 0;
            uint32_t gps_xy_rejected = 0;
            uint32_t gps_z_accepted = 0;
            uint32_t gps_z_rejected = 0;
            uint32_t gps_velocity_accepted = 0;
            uint32_t gps_velocity_rejected = 0;
            uint32_t heading_accepted = 0;
            uint32_t heading_rejected = 0;
            uint32_t level_accepted = 0;
            uint32_t level_rejected = 0;
        };

        explicit EKF(const Params &p = {});

        void reset(const AUVState &init);

        AUVState update(const NavigationMeasurements &meas,
                        const std::optional<DVLMeasurement> &bottom_dvl,
                        const std::optional<DVLMeasurement> &water_dvl,
                        const std::optional<GPSMeasurement> &gps,
                        double dt);

        const Eigen::Matrix<double, EKF_N, 1> &state() const { return _x; }
        const Eigen::Matrix<double, EKF_N, EKF_N> &covariance() const { return _P; }
        const InnovationStats &last_stats() const { return _last_stats; }

    private:
        Eigen::Matrix<double, EKF_N, 1> _x;
        Eigen::Matrix<double, EKF_N, EKF_N> _P;
        Eigen::Matrix<double, EKF_N, EKF_N> _Q;
        Eigen::Matrix<double, 3, 3> _R_dvl;
        Eigen::Matrix<double, 3, 3> _R_water_dvl;
        Eigen::Matrix<double, 3, 3> _R_zvu;
        Eigen::Matrix<double, 1, 1> _R_depth;
        Eigen::Matrix<double, 2, 2> _R_gps;
        Eigen::Matrix<double, 1, 1> _R_gps_z;
        Eigen::Matrix<double, 3, 3> _R_gps_velocity;
        Eigen::Matrix<double, 1, 1> _R_heading;
        Eigen::Matrix<double, 3, 3> _R_level;
        double _level_gate = 0.5;
        double _gate_dvl_nis = 16.27;
        double _gate_water_dvl_nis = 16.27;
        double _gate_gps_velocity_nis = 16.27;
        double _gate_level_nis = 16.27;
        double _gate_gps_xy_nis = 13.82;
        double _gate_scalar_nis = 10.83;
        double _min_variance = 1.0e-9;
        double _initial_current_variance = 4.0;
        double _current_valid_std = 0.5;
        double _last_bottom_dvl_timestamp_s = -std::numeric_limits<double>::infinity();
        double _last_water_dvl_timestamp_s = -std::numeric_limits<double>::infinity();
        double _last_gps_timestamp_s = -std::numeric_limits<double>::infinity();
        bool _initialized = false;
        InnovationStats _last_stats{};

        void _predict(const Eigen::Vector3d &accel, const Eigen::Vector3d &omega,
                      bool have_accel, double dt);
        bool _update_dvl(const Eigen::Vector3d &z_dvl, const Eigen::Matrix3d &R);
        bool _update_water_dvl(const Eigen::Vector3d &z_water_dvl, const Eigen::Matrix3d &R);
        void _update_zvu(); // zero-velocity pseudo-update when DVL unavailable
        bool _update_depth(double depth_m, double variance);
        bool _update_gps_xy(const GPSMeasurement &gps, const Eigen::Matrix2d &R);
        bool _update_gps_velocity(const GPSMeasurement &gps, const Eigen::Matrix3d &R);
        bool _update_gps_course_yaw(const GPSMeasurement &gps, double variance);
        bool _update_gps_altitude(double pos_d, double variance);
        std::optional<double> _heading_from_magnetometer(const Eigen::Vector3d &mag_body) const;
        bool _update_heading(double heading_yaw, double variance);
        bool _update_level(const Eigen::Vector3d &accel, const Eigen::Matrix3d &R); // accel leveling (roll/pitch)
        Eigen::Matrix3d _valid_cov3(const Eigen::Matrix3d &R, const Eigen::Matrix3d &fallback) const;
        Eigen::Matrix2d _valid_cov2(const Eigen::Matrix2d &R, const Eigen::Matrix2d &fallback) const;
        double _valid_variance(double variance, double fallback) const;
        double _heading_variance_from_mag(const Vector3Measurement &mag) const;
        bool _consume_new_sample(double timestamp_s, double &last_timestamp_s);

        // Continuous-time dynamics x_dot = f(x, accel, omega), shared by predict integration and numerical Jacobian.
        static Eigen::Matrix<double, EKF_N, 1>
        _dynamics(const Eigen::Matrix<double, EKF_N, 1> &x,
                  const Eigen::Vector3d &accel, const Eigen::Vector3d &omega,
                  bool have_accel);

        static Eigen::Matrix3d _rot_nb(double phi, double theta, double psi);
        static Eigen::Matrix3d _J2(double phi, double theta);
        // Projection of nominal gravity onto the body frame R_nb^T g_n (static leveling measurement model h = -this value)
        static Eigen::Vector3d _gravity_body(double phi, double theta, double psi);
        static double _ssa(double a);
    };

} // namespace hydrox
