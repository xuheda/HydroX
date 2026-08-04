#pragma once
/**
 * fc_snapshot.h — HydroX flight-controller state snapshot
 *
 * `hydrox_sitl` fills `FcSnapshot` once per GNC tick. `DdsPublisher` then
 * serializes the snapshot directly to Micro XRCE-DDS / ROS 2 topics.
 *
 * This header is an in-process data handoff between the flight-control loop and
 * the DDS publisher. It is not a transport protocol contract.
 */
#include <cstdint>

namespace hydrox
{

#pragma pack(push, 1)
    struct FcSnapshot
    {
        uint64_t timestamp_us = 0; // HIL timestamp (us)

        // EKF output in NED.
        double eta[6] = {}; // [N, E, D, roll, pitch, yaw] (m, rad)
        double nu[6] = {};  // [u, v, w, p, q, r]          (m/s, rad/s)
        double depth_m = 0.0;
        uint8_t dvl_valid = 0;

        // Simulator truth in NED, when supplied by UE/Fossen HIL.
        double truth_eta[6] = {};
        double truth_nu[6] = {};
        uint8_t truth_valid = 0;
        uint8_t _pad_truth[7] = {};

        // Raw sensors used by sensor_combined.
        float acc[3] = {};     // IMU specific force (m/s^2)
        float gyro[3] = {};    // IMU angular velocity (rad/s)
        float dvl_vel[3] = {}; // DVL body-frame velocity (m/s), invalid when dvl_valid=0

        // GPS fields used by MAVLink/ROS status outputs.
        int32_t gps_lat = 0; // deg * 1e7
        int32_t gps_lon = 0; // deg * 1e7
        int32_t gps_alt = 0; // mm ellipsoid altitude
        double gps_vn = 0.0; // m/s
        double gps_ve = 0.0;
        double gps_vd = 0.0;
        uint8_t gps_fix = 0; // 0=invalid, 3=3D fix
        uint8_t gps_satellites = 0;

        // Normalized and physical actuator outputs.
        float fins[4] = {};     // fin command [-1, 1]
        double fin_deg[4] = {}; // fin command in degrees for telemetry
        float thrust = 0.0f;    // normalized thrust [-1, 1]
        float rpm = 0.0f;       // commanded/actual RPM used by actuator output topic
        float normalized[8] = {}; // vehicle-neutral logical actuator channels
        uint8_t actuator_channel_count = 0;

        // Mission state exposed on the vehicle state-estimate topic.
        char mission_state[16] = "IDLE"; // IDLE / RUNNING / COMPLETE / FAILED

        // Motor model state.
        float motor_rpm_actual = 0.0f;
        float motor_thrust_N = 0.0f;
        float motor_power_W = 0.0f;
        float motor_current_A = 0.0f;

        // Energy model state.
        float power_total_W = 0.0f;
        float energy_Wh = 0.0f;
        float battery_soc = 1.0f;   // [0, 1]
        float V_terminal = 24.0f;   // V
        float runtime_rem_s = 0.0f; // 0 means unavailable
        uint8_t _pad2[4] = {};      // keep 8-byte alignment

        // nav_msgs/Odometry covariance, row-major [x,y,z,roll,pitch,yaw].
        double pose_cov[36] = {};
        double twist_cov[36] = {};
        uint8_t odometry_covariance_valid = 0;
    };
#pragma pack(pop)

} // namespace hydrox
