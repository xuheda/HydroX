#pragma once
/**
 * types.h — HydroX public data structures
 *
 * Coordinate system: NED (North-East-Down), SI units.
 *   eta = [x, y, z, roll, pitch, yaw]   (m, rad)
 *   nu  = [u, v, w, p, q, r]            (m/s, rad/s)
 */
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>

namespace hydrox
{
    enum class VehicleClass : int
    {
        UUV = 0,
        USV = 1,
        UAV_MULTIROTOR = 2,
        UAV_FIXED_WING = 3,
        UAV_VTOL = 4,
    };

    /**
     * Physical meaning of the optional environmental-velocity states.
     *
     * The navigation core carries one generic NED medium-velocity vector.
     * Measurement profiles decide whether it represents water current, wind,
     * or is intentionally disabled because the active sensor suite cannot
     * observe it.
     */
    enum class MediumVelocityKind : uint8_t
    {
        None = 0,
        WaterCurrent = 1,
        Wind = 2,
    };

    // Vehicle-neutral 6-DOF navigation state used by every control archetype.
    struct NavigationState
    {
        Eigen::Vector<double, 6> eta; // [N, E, D, roll, pitch, yaw]
        Eigen::Vector<double, 6> nu;  // [u, v, w, p, q, r]
        double depth_m = 0.0;
        // EKF-estimated environmental-medium velocity in world NED.
        // `nu[0..2]` remains body-frame ground velocity.
        Eigen::Vector3d medium_velocity_ned = Eigen::Vector3d::Zero();
        Eigen::Vector3d medium_velocity_std_ned = Eigen::Vector3d::Zero();
        MediumVelocityKind medium_velocity_kind = MediumVelocityKind::None;
        bool medium_velocity_valid = false;
        bool dvl_valid = false;
        double timestamp = 0.0;

        static NavigationState zeros()
        {
            NavigationState s;
            s.eta.setZero();
            s.nu.setZero();
            return s;
        }

        double heading_rad() const { return eta[5]; }
        double surge() const { return nu[0]; }
    };

    // Source compatibility for downstream code while internal code migrates to
    // the vehicle-neutral name. New code should use NavigationState.
    // Actuator Command
    struct FinCmd
    {
        // [d_ss, d_ps, d_br, d_tr] fin angles (degrees, ±delta_max)
        std::array<double, 4> fins = {0, 0, 0, 0};
        double rpm = 0.0;          // Thruster speed (RPM), used by motor model
        double thrust_frac = 0.0;  // tau_X / max_thrust_N ∈ [-1, 1], used by control loop

        void clamp(double delta_max = 15.0, double rpm_max = 1525.0)
        {
            for (auto &f : fins)
                f = std::max(-delta_max, std::min(delta_max, f));
            rpm = std::max(-rpm_max, std::min(rpm_max, rpm));
            thrust_frac = std::max(-1.0, std::min(1.0, thrust_frac));
        }

        /** Normalized to [-1, 1], used for HIL_ACTUATOR_CONTROLS message
         *  channel[0..3] = fins / delta_max
         *  channel[4]    = thrust_frac (tau_X / max_thrust_N, linear thrust normalization)
         */
        std::array<float, 5> normalized(double delta_max = 15.0,
                                        double /*rpm_max*/ = 1525.0) const
        {
            return {
                static_cast<float>(fins[0] / delta_max),
                static_cast<float>(fins[1] / delta_max),
                static_cast<float>(fins[2] / delta_max),
                static_cast<float>(fins[3] / delta_max),
                static_cast<float>(thrust_frac),
            };
        }
    };

    // Sensor Measurements
    struct IMUMeasurement
    {
        double ax, ay, az; // Specific force (m/s², body frame, including gravity)
        double p, q, r;    // Angular velocity (rad/s)
        double timestamp = 0.0;
    };

    struct DVLMeasurement
    {
        double vel_x, vel_y, vel_z;            // Body frame velocity (m/s)
        double range0, range1, range2, range3; // Four beam ranges (m)
        int beam_valid = 0;                    // Number of valid beams
        double timestamp = 0.0;
        uint8_t tracking_mode = 0;             // 1=bottom-track, 2=water-track
        bool altitude_valid = false;
        double altitude_m = std::numeric_limits<double>::quiet_NaN();

        Eigen::Vector3d velocity_body() const
        {
            return {vel_x, vel_y, vel_z};
        }
    };

    struct DepthMeasurement
    {
        double depth_m = 0.0; // NED depth, positive downwards (m)
        double timestamp = 0.0;
    };

    struct GPSMeasurement
    {
        double pos_n = 0.0; // NED north (m)
        double pos_e = 0.0; // NED east (m)
        double pos_d = 0.0; // NED down (m), optional
        double vel_n = 0.0; // NED north velocity (m/s), optional
        double vel_e = 0.0; // NED east velocity (m/s), optional
        double vel_d = 0.0; // NED down velocity (m/s), optional
        bool has_altitude = false;
        bool has_velocity = false;
        double timestamp = 0.0;
    };

    // GNC Setpoint
    enum class GNCMode : int
    {
        DISABLED = 0,
        DEPTH_HOLD = 1,
        WAYPOINT_3D = 2,
        DP = 3,
        SURFACE = 4, // float at surface: depth PID off, heading maintained
    };

    struct GNCSetpoint
    {
        double depth_ref = 0.0;   // Target depth (m, NED)
        double heading_ref = 0.0; // Target heading (rad)
        double surge_ref = 0.0;   // Target surge velocity (m/s)
        bool use_yaw_rate_ref = false;
        double yaw_rate_ref = 0.0; // target yaw rate (rad/s)
        // Waypoint mode
        double wp_n = 0.0, wp_e = 0.0, wp_d = 0.0;
    };

} // namespace hydrox
