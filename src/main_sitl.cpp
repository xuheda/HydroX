/**
 * main_sitl.cpp — HydroX FC SITL process entry point
 *
 * Analogous to: px4_sitl process (independently executable, zero ROS dependency)
 *
 * Architecture:
 *   UE5 FOceanXCommBridge <─TCP─> hydrox_sitl [uxr client] ──UDP──> MicroXRCEAgent ──> ROS 2
 *                                             <─UDP──────────────> QGroundControl
 *
 * Data flow:
 *   UE5 → HIL_SENSOR/HIL_GPS/HIL_DVL → EKF → GNC → HIL_ACTUATOR_CONTROLS → UE5
 *   GNC Output → DdsPublisher (uxr) → MicroXRCEAgent → ROS 2 Topics
 *   ROS 2 setpoint → MicroXRCEAgent → DdsPublisher (uxr) → GNC Setpoint update
 *   EKF/GNC State → QGC UDP → ATTITUDE/LOCAL_POS/VFR_HUD/GLOBAL_POS @4Hz
 *                          → SYS_STATUS @1Hz
 *                          → STATUSTEXT (event triggered)
 *
 * Usage:
 *   hydrox_sitl [--ue5-host 127.0.0.1] [--ue5-port 14600]
 *                  [--qgc-host 255.255.255.255] [--qgc-port 14550]
 *                  [--vehicle auv0] [--vehicle-type EcaA9]
 *                  [--vehicle-params %OCEANX_ROOT%\engine\Content\Fossen\eca_a9_params.json]
 *                  [--vehicle-params-dir %OCEANX_ROOT%\engine\Content\Fossen]
 *                  [--dds-host 127.0.0.1] [--dds-port 8888] [--ros-domain-id 0]
 *                  [--mavlink-signing-key-file D:\secure\hil.key]
 *                  [--mavlink-signing-link-id 0]
 *                  [--ekf-accel auto|off|on]
 *                  [--xlog auto|off|path.xlog]
 *                  [--time-mode hil|wall]
 *                  [--parent-pid UE_PROCESS_ID]
 *                  [--publish-truth-state true|false]
 *                  [--allow-truth-heading-aid true|false]
 *                  [--rate 100]
 *                  [--depth 5.0] [--heading 0.0] [--surge 0.0]
 *                  [--mission-radius 3.0] [--mission-timeout 2.0]
 *                  [--mode DISABLED]
 */
#include "ekf.h"
#include "fossen_vehicle_params.h"
#include "mavlink_signing.h"
#include "gnc/control_factory.h"
#include "gnc/control_interfaces.h"
#include "gnc/energy_model.h"
#include "gnc/motor_model.h"
#include "mavlink_hil.h"
#include "sensor_adapter.h"
#ifdef HYDROX_DDS_ENABLED
#include "sitl/dds_worker.h"
#endif
#include "sitl/sitl_config.h"
#include "sitl/sitl_platform.h"
#include "sitl/sitl_xlog.h"
#include "tcp_transport.h"
#include "ue_control_session.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <utility>

using namespace hydrox;
using namespace std::chrono_literals;
namespace sitl = hydrox::sitl;

// ─────────────────────────────────────────────────────────────────────────────
// Global shutdown flag
// ─────────────────────────────────────────────────────────────────────────────
static std::atomic<bool> g_running{true};
static void sig_handler(int) { g_running = false; }

enum class MissionState : uint8_t
{
    IDLE = 0,
    RUNNING = 1,
    COMPLETE = 2,
    FAILED = 3,
};

static const char *mission_state_name(MissionState s)
{
    switch (s)
    {
    case MissionState::RUNNING:
        return "RUNNING";
    case MissionState::COMPLETE:
        return "COMPLETE";
    case MissionState::FAILED:
        return "FAILED";
    case MissionState::IDLE:
    default:
        return "IDLE";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char *argv[])
{
    // Disable stdout buffering so ros2 launch (which pipes stdout) sees output
    // immediately instead of waiting for a 4096-byte buffer fill.
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    std::signal(SIGINT, sig_handler);
    std::signal(SIGTERM, sig_handler);

    sitl::Config cfg;
    try
    {
        cfg = sitl::parse_config(argc, argv);
    }
    catch (const std::exception &error)
    {
        std::fprintf(stderr, "[FC] Invalid command line: %s\n", error.what());
        return 2;
    }

    sitl::NetworkRuntime network;
    if (!network.ready())
    {
        std::fprintf(stderr, "[FC] Network initialization failed.\n");
        return 4;
    }

    sitl::ParentProcessGuard parent_guard(cfg.parent_pid);
    if (!parent_guard.arm())
    {
        std::fprintf(stderr,
                     "[FC] Parent PID %llu is unavailable; refusing to start an orphan SITL.\n",
                     static_cast<unsigned long long>(cfg.parent_pid));
        return 3;
    }
    std::printf("[FC] HydroX FC SITL process (analogous to px4_sitl)\n");
    std::printf("[FC] UE5:    %s:%d\n", cfg.ue5_host.c_str(), cfg.ue5_port);
    std::printf("[FC] QGC:    %s:%d\n", cfg.qgc_host.c_str(), cfg.qgc_port);
    std::printf("[FC] DDS:    %s:%d  domain=%u  vehicle=%s  type=%s\n",
                cfg.dds_host.c_str(), cfg.dds_port,
                static_cast<unsigned>(cfg.ros_domain_id),
                cfg.vehicle.c_str(), cfg.vehicle_type.c_str());
    std::printf("[FC] Rate:   %dHz  Mode:%s  Depth:%.1fm  InitNE=(%.1f, %.1f)m\n",
                cfg.rate_hz, cfg.init_mode.c_str(), cfg.init_depth,
                cfg.init_n, cfg.init_e);
    if (cfg.parent_pid != 0)
        std::printf("[FC] Parent: PID %llu (exit when parent exits)\n",
                    static_cast<unsigned long long>(cfg.parent_pid));
    const AccelMode accel_mode = sitl::accel_mode_from_string(cfg.ekf_accel);
    std::printf("[FC] EKF:    accel=%s truth_heading_aid=%s\n",
                sitl::accel_mode_name(accel_mode),
                cfg.allow_truth_heading_aid ? "ON(debug)" : "OFF");

    MavlinkSigningConfig mavlink_signing;
    if (!cfg.mavlink_signing_key_file.empty())
    {
        std::string signing_error;
        if (!load_mavlink_signing_key_file(
                cfg.mavlink_signing_key_file,
                mavlink_signing.secret_key,
                &signing_error))
        {
            std::fprintf(stderr,
                         "[FC] FATAL: invalid MAVLink signing key file: %s\n",
                         signing_error.c_str());
            return 2;
        }
        mavlink_signing.enabled = true;
        mavlink_signing.sign_outgoing = true;
        mavlink_signing.require_incoming = true;
        mavlink_signing.link_id = cfg.mavlink_signing_link_id;
        std::printf("[FC] HIL:    MAVLink2 signing enabled (link_id=%u)\n",
                    static_cast<unsigned>(mavlink_signing.link_id));
    }

    // ── Autopilot Core Components ──────────────────────────────────────────────
    std::string params_error;
    const auto vehicle_params = load_fossen_control_params(
        cfg.vehicle_type, cfg.vehicle_params, cfg.vehicle_params_dir, &params_error);
    if (!vehicle_params.valid)
    {
        std::fprintf(stderr,
                     "[FC] FATAL: vehicle params unavailable for type '%s': %s\n",
                     cfg.vehicle_type.c_str(), params_error.c_str());
        return 2;
    }
    std::printf("[FC] Params: %s%s\n",
                vehicle_params.source_path.c_str(),
                vehicle_params.loaded_from_json ? "" : " (fallback)");
    const std::string initial_mode = cfg.init_mode;
    std::printf("[FC] Alloc:  S_fin=%.5fm2 CL=(%.2f, %.2f) x_fin=%.3fm "
                "D_prop=%.3fm n_max=%.0frpm Tmax=%.1fN u_min=%.2fm/s\n",
                vehicle_params.allocator.S_fin,
                vehicle_params.allocator.CL_s,
                vehicle_params.allocator.CL_r,
                vehicle_params.allocator.x_fin,
                vehicle_params.allocator.D_prop,
                vehicle_params.allocator.n_max_rpm,
                vehicle_params.allocator.max_thrust_N,
                vehicle_params.allocator.u_min);

    const int effective_rate_hz = std::max(1, cfg.rate_hz);
    sitl::XLogRecorder xlog_recorder(
        cfg, vehicle_params, accel_mode, effective_rate_hz);

    TcpTransport transport(cfg.ue5_host, cfg.ue5_port, /*client=*/true);
    MavlinkHIL codec(1, 1, mavlink_signing);
    const uint8_t mav_type = sitl::mav_type_for_vehicle_class(vehicle_params.vehicle_class);
    EKF ekf;
    SensorAdapter::Params sensor_params;
    sensor_params.vehicle_class = vehicle_params.vehicle_class;
    sensor_params.accel_mode = accel_mode;
    sensor_params.gps_origin_lat_deg = cfg.gps_origin_lat_deg;
    sensor_params.gps_origin_lon_deg = cfg.gps_origin_lon_deg;
    sensor_params.gps_origin_alt_m = cfg.gps_origin_alt_m;
    SensorAdapter sensor_adapter(sensor_params);
    // Build the {controller, allocator} stack for this vehicle's archetype.
    // Slender-body torpedoes get the cascade + fin allocator; thruster ROVs get
    // the 6-DOF controller + thruster-matrix allocator. main loop talks only the
    // IController/IAllocator interfaces below.
    ControlStack stack = build_control_stack(vehicle_params);
    IController &gnc = *stack.controller;
    IAllocator &alloc = *stack.allocator;
    MotorModel motor{vehicle_params.motor}; // fin/prop power model; thruster vehicles feed rpm=0
    EnergyModel energy;

    GNCSetpoint sp;
    sp.depth_ref = cfg.init_depth;
    sp.heading_ref = cfg.init_heading;
    sp.surge_ref = cfg.init_surge;
    GNCMode cur_mode = sitl::gnc_mode_from_string(initial_mode);
    MissionState mission_state = (cur_mode == GNCMode::DISABLED)
                                     ? MissionState::IDLE
                                     : MissionState::RUNNING;
    bool have_external_setpoint = false;
    bool reset_controller_on_setpoint = false;
    sitl::UeControlSessionGate ue_control_session;
    gnc.set_mode(cur_mode);
    gnc.set_setpoint(sp);

    const auto enter_safe_control_state = [&](const char *reason)
    {
        const bool state_changed =
            have_external_setpoint || cur_mode != GNCMode::DISABLED ||
            sp.surge_ref != 0.0 || sp.use_yaw_rate_ref || sp.yaw_rate_ref != 0.0;
        have_external_setpoint = false;
        cur_mode = GNCMode::DISABLED;
        mission_state = MissionState::IDLE;
        sp.surge_ref = 0.0;
        sp.use_yaw_rate_ref = false;
        sp.yaw_rate_ref = 0.0;
        gnc.set_mode(cur_mode);
        gnc.set_setpoint(sp);
        reset_controller_on_setpoint = true;
        if (state_changed)
            std::fprintf(stderr, "[FC] %s; control set to DISABLED\n", reason);
    };

#ifdef HYDROX_DDS_ENABLED
    // Micro XRCE-DDS is isolated from the real-time control loop. The worker
    // owns all network/session calls; this thread only uses non-blocking
    // latest-value mailboxes.
    sitl::DdsWorker dds_worker({
        cfg.dds_host,
        cfg.dds_port,
        cfg.ros_domain_id,
        cfg.vehicle,
        static_cast<uint32_t>(cfg.ue5_port),
        cfg.publish_truth_state,
    });
    uint64_t last_dds_setpoint_sequence = 0;
    uint64_t last_dds_status_sequence = 0;
    sitl::DdsControlLinkState dds_control_link;

    const auto apply_dds_setpoint = [&](const hydrox::GNCSetpointDds &isp)
    {
        sp.depth_ref = isp.depth_ref;
        sp.heading_ref = isp.heading_ref;
        sp.surge_ref = isp.surge_ref;
        sp.use_yaw_rate_ref = isp.use_yaw_rate_ref;
        sp.yaw_rate_ref = isp.yaw_rate_ref;
        sp.wp_n = isp.wp_n;
        sp.wp_e = isp.wp_e;
        sp.wp_d = isp.wp_d;
        const GNCMode prev_mode = cur_mode;
        cur_mode = static_cast<GNCMode>(isp.mode);
        have_external_setpoint = true;
        mission_state = (cur_mode == GNCMode::DISABLED)
                            ? MissionState::IDLE
                            : MissionState::RUNNING;
        if (cur_mode != GNCMode::DISABLED && prev_mode != cur_mode)
            reset_controller_on_setpoint = true;
        gnc.set_mode(cur_mode);
        gnc.set_setpoint(sp);
    };

    const auto revoke_dds_setpoint = [&](std::chrono::steady_clock::time_point now,
                                         const char *reason)
    {
        if (!have_external_setpoint)
            return;

        ue_control_session.revoke_setpoint(now);
        enter_safe_control_state(reason);
    };
#endif

    // ── QGC UDP Socket (Broadcast) ─────────────────────────────────────────────
    sitl::UdpSender qgc_sender(cfg.qgc_host, cfg.qgc_port, true);
    if (qgc_sender.is_open())
        std::printf("[FC] QGC UDP ready → %s:%d\n",
                    cfg.qgc_host.c_str(), cfg.qgc_port);

    const auto period = std::chrono::microseconds(1'000'000 / effective_rate_hz);
    const double expected_dt = 1.0 / static_cast<double>(effective_rate_hz);
    const bool use_hil_time = cfg.time_mode == "hil";
    xlog_recorder.start_session_clock();

    // ── Disconnect Reconnection Outer Loop ────────────────────────────────────────────────────
    while (g_running && parent_guard.is_parent_alive())
    {
        std::printf("[FC] Connecting to UE5 %s:%d ...\n",
                    cfg.ue5_host.c_str(), cfg.ue5_port);
        if (!transport.connect())
        {
            std::printf("[FC] Connection failed, retrying in 1s\n");
            std::this_thread::sleep_for(1s);
            continue;
        }
        const auto ue_connected_at = std::chrono::steady_clock::now();
        const uint64_t ue_session_generation =
            ue_control_session.on_connected(ue_connected_at);
        std::printf("[FC] Connected! GNC loop @%dHz time_mode=%s\n",
                    cfg.rate_hz, use_hil_time ? "hil" : "wall");
        std::printf("[FC] UE control session=%llu waiting for a valid HIL_SENSOR\n",
                    static_cast<unsigned long long>(ue_session_generation));
        if (qgc_sender.is_open())
        {
            const auto packet =
                codec.encode_statustext(6, "HydroX SITL connected");
            qgc_sender.send(packet.data(), packet.size());
        }

        AUVState init_state = AUVState::zeros();
        init_state.eta[0] = cfg.init_n;
        init_state.eta[1] = cfg.init_e;
        init_state.eta[2] = cfg.init_depth;
        init_state.eta[5] = cfg.init_heading;
        if (cur_mode != GNCMode::DISABLED)
            init_state.nu[0] = cfg.init_surge;
        init_state.depth_m = cfg.init_depth;
        ekf.reset(init_state);
        gnc.reset(init_state);
        motor.reset();
        energy.reset();
        sensor_adapter.reset();

        uint32_t tick = 0;
        bool ekf_init = false;
        bool gps_valid = false;
        HilGpsMsg last_gps{};
        HilDvlMsg last_dvl{};
        AUVState state = AUVState::zeros();
        ActuatorCmd cmd{};

        auto t_prev = std::chrono::steady_clock::now();
        auto t_last_status = t_prev;
        auto t_last_heartbeat = t_prev;
        auto t_last_imu = t_prev;
        auto t_last_no_imu_warning = t_prev - 2s;
        uint64_t prev_imu_time_us = 0;
        uint32_t total_bytes_without_imu = 0;
        bool simulator_paused = false;

        // ── 100Hz GNC Inner Loop ──────────────────────────────────────────────
        while (g_running && parent_guard.is_parent_alive() && transport.is_connected())
        {
            const auto t_now = std::chrono::steady_clock::now();
            const double wall_dt = std::chrono::duration<double>(t_now - t_prev).count();
            t_prev = t_now;
            ++tick;

            // ── Read UE5 HIL data ─────────────────────────────────────────
            uint8_t buf[1024];
            const int n = transport.read(buf, sizeof(buf));
            sensor_adapter.begin_cycle();

            if (n > 0)
            {
                for (const auto &f : codec.feed(buf, static_cast<size_t>(n)))
                {
                    if (f.msg_id == MSGID_HEARTBEAT)
                    {
                        const HeartbeatMsg heartbeat = codec.parse_heartbeat(f);
                        if (heartbeat.valid)
                        {
                            const bool next_paused =
                                heartbeat.system_status == MAV_STATE_STANDBY;
                            if (next_paused != simulator_paused)
                            {
                                simulator_paused = next_paused;
                                std::printf(
                                    "[FC:%d] simulator control chain %s\n",
                                    cfg.ue5_port,
                                    simulator_paused ? "paused" : "resuming");
                            }
                        }
                    }
                    sensor_adapter.ingest_frame(f, codec);
                }
            }
            else if (n < 0)
            {
                break;
            }

            const NavigationInput nav = sensor_adapter.build();
            gps_valid = nav.gps_valid;
            last_gps = nav.last_gps;
            last_dvl = nav.last_dvl;

            // A new simulator byte stream is a new state epoch. Only a real
            // timestamped IMU sample opens the gate for post-reconnect commands;
            // the default value produced by a malformed short packet cannot.
            if (nav.got_imu && nav.imu.time_usec > 0 &&
                ue_control_session.observe_valid_sensor(t_now))
            {
                std::printf(
                    "[FC] UE control session=%llu sensor-ready; waiting for a fresh Setpoint\n",
                    static_cast<unsigned long long>(ue_session_generation));
            }

            // TCP liveness must not depend on receiving IMU samples. During an
            // intentional simulator pause the control loop is frozen, but both
            // peers continue HEARTBEAT traffic so neither side declares a
            // transport failsafe.
            if (std::chrono::duration<double>(t_now - t_last_heartbeat).count() >= 1.0)
            {
                t_last_heartbeat = t_now;
                const auto hb = codec.encode_heartbeat(mav_type);
                transport.write(hb.data(), hb.size());
            }

#ifdef HYDROX_DDS_ENABLED
            // DDS connection/reception remains live on the worker while paused.
            // Status is consumed first so a disconnect epoch invalidates the
            // previous session before any newly received Setpoint is applied.
            sitl::DdsConnectionStatus dds_status;
            if (dds_worker.try_take_connection_status(
                    last_dds_status_sequence, dds_status))
            {
                if (dds_control_link.observe(dds_status))
                    revoke_dds_setpoint(t_now, "DDS connection lost");
            }

            // Setpoints are stamped with the XRCE session generation. A stale
            // command left in the mailbox across reconnect is never accepted.
            sitl::DdsSetpointSample incoming_setpoint;
            if (dds_worker.try_take_setpoint(
                    last_dds_setpoint_sequence, incoming_setpoint))
            {
                if (!dds_control_link.accepts_setpoint(
                        incoming_setpoint.session_generation))
                {
                    std::fprintf(
                        stderr,
                        "[FC][DDS] stale Setpoint rejected session=%llu active=%llu\n",
                        static_cast<unsigned long long>(
                            incoming_setpoint.session_generation),
                        static_cast<unsigned long long>(
                            dds_control_link.session_generation()));
                }
                else if (!ue_control_session.accept_setpoint(
                             incoming_setpoint.received_at))
                {
                    std::fprintf(
                        stderr,
                        "[FC][DDS] pre-session Setpoint rejected dds_session=%llu "
                        "ue_session=%llu phase=%s\n",
                        static_cast<unsigned long long>(
                            incoming_setpoint.session_generation),
                        static_cast<unsigned long long>(
                            ue_session_generation),
                        sitl::ue_control_session_phase_name(
                            ue_control_session.phase()));
                }
                else
                {
                    apply_dds_setpoint(incoming_setpoint.setpoint);
                }
            }
#endif

            // Command freshness is wall/steady-clock based and remains active
            // while simulation time is paused or no IMU sample is arriving.
            if (have_external_setpoint &&
                ue_control_session.setpoint_timed_out(
                    t_now, std::chrono::duration<double>(cfg.mission_timeout_s)))
            {
                ue_control_session.revoke_setpoint(t_now);
                enter_safe_control_state("external Setpoint timed out");
            }

            if (simulator_paused)
            {
                t_last_imu = t_now;
                t_last_no_imu_warning = t_now - 2s;
                total_bytes_without_imu = 0;
                auto pause_sleep = period;
                pause_sleep = std::max(pause_sleep, std::chrono::microseconds(1000));
                pause_sleep = std::min(pause_sleep, std::chrono::microseconds(20000));
                std::this_thread::sleep_for(pause_sleep);
                continue;
            }

            if (!nav.got_imu)
            {
                // UE5 sends HIL_SENSOR only when a new IMU sample is available,
                // capped by the bridge max publish rate. During UE editor pause
                // HIL time stops, so rate-limit this warning by wall time.
                total_bytes_without_imu += (n > 0) ? static_cast<uint32_t>(n) : 0;
                const auto no_imu_elapsed = t_now - t_last_imu;
                const auto warn_elapsed = t_now - t_last_no_imu_warning;
                if (no_imu_elapsed >= 500ms && warn_elapsed >= 2s)
                {
                    std::printf("[FC:%d] waiting for HIL_SENSOR - no IMU frame for %.1fs, %u raw bytes received\n",
                                cfg.ue5_port,
                                std::chrono::duration<double>(no_imu_elapsed).count(),
                                total_bytes_without_imu);
                    t_last_no_imu_warning = t_now;
                    total_bytes_without_imu = 0;
                }
                if (use_hil_time)
                {
                    auto no_imu_sleep = period;
                    no_imu_sleep = std::max(no_imu_sleep, std::chrono::microseconds(1000));
                    no_imu_sleep = std::min(no_imu_sleep, std::chrono::microseconds(20000));
                    std::this_thread::sleep_for(no_imu_sleep);
                }
                else
                    std::this_thread::sleep_until(t_prev + period);
                continue;
            }

            t_last_imu = t_now;
            t_last_no_imu_warning = t_now - 2s;
            total_bytes_without_imu = 0;

            double dt = wall_dt;
            if (use_hil_time)
            {
                if (prev_imu_time_us > 0 && nav.imu.time_usec > prev_imu_time_us)
                    dt = static_cast<double>(nav.imu.time_usec - prev_imu_time_us) * 1e-6;
                else
                    dt = expected_dt;

                prev_imu_time_us = nav.imu.time_usec;
                dt = std::clamp(dt, expected_dt * 0.1, expected_dt * 10.0);
            }
            // ── EKF Update ──────────────────────────────────────────────────
            // Specific force (body frame, m/s²). Only enabled when UE marks all three axes XACC|YACC|ZACC as valid,
            // otherwise have_accel=false -> predict degrades to kinematics (zero regression for legacy UE).
            NavigationMeasurements nav_meas = nav.measurements;
            if (!cfg.allow_truth_heading_aid)
                nav_meas.truth_heading_debug.meta.valid = false;
            // DVL stays available to the EKF while recent, but the EKF consumes
            // each HIL timestamp only once. This avoids falsely shrinking its
            // covariance by reusing one 5 Hz frame on every 100 Hz IMU tick.
            state = ekf.update(
                nav_meas,
                nav.dvl,
                nav.water_dvl,
                nav.gps,
                dt);
            state.dvl_valid = nav.dvl_recent;
            ekf_init = true;

            // ── GNC Update + Actuator Output ─────────────────────────────────────
            if (reset_controller_on_setpoint)
            {
                gnc.reset(state);
                reset_controller_on_setpoint = false;
            }

            // Controller generates generalized force tau -> allocator maps to normalized actuator channels (clamp+normalize inside allocator)
            const Wrench tau = gnc.update(state, dt);
            cmd = alloc.allocate(tau, state.surge());
            const auto &norm = cmd.ch;
            constexpr uint8_t MAV_MODE_FLAG_HIL_ENABLED = 32;
            constexpr uint8_t MAV_MODE_FLAG_SAFETY_ARMED = 128;
            const uint8_t actuator_mode =
                static_cast<uint8_t>(MAV_MODE_FLAG_HIL_ENABLED |
                                     (cur_mode != GNCMode::DISABLED ? MAV_MODE_FLAG_SAFETY_ARMED : 0));
            if (cur_mode != GNCMode::DISABLED)
            {
                const auto act_pkt = codec.encode_hil_actuator_controls(
                    norm, static_cast<uint64_t>(nav.imu.time_usec), actuator_mode, 0);
                transport.write(act_pkt.data(), act_pkt.size());
            }

            // ── Motor Model (First-order lag + power calculation) ─────────────────────────
            const MotorState ms = motor.step(cmd.rpm, dt);

            // ── Energy Model (Battery SOC + runtime estimation) ──────────────────────────
            const EnergyState es = energy.update(ms.power_W, dt);

            // ── Mission State ────────────────────────────────────────────────
            double waypoint_distance_m = -1.0;
            if (cur_mode == GNCMode::WAYPOINT_3D)
            {
                const double dn = state.eta[0] - sp.wp_n;
                const double de = state.eta[1] - sp.wp_e;
                const double dd = state.eta[2] - sp.wp_d;
                waypoint_distance_m = std::sqrt(dn * dn + de * de + dd * dd);
            }

            if (cur_mode == GNCMode::DISABLED)
                mission_state = MissionState::IDLE;
            else if (mission_state == MissionState::IDLE)
                mission_state = MissionState::RUNNING;
            else if (mission_state == MissionState::RUNNING &&
                     waypoint_distance_m >= 0.0 &&
                     waypoint_distance_m <= cfg.mission_radius)
                mission_state = MissionState::COMPLETE;

            sitl::XLogTickData xlog_tick;
            xlog_tick.state = &state;
            xlog_tick.setpoint = &sp;
            xlog_tick.wrench = &tau;
            xlog_tick.actuator = &cmd;
            xlog_tick.navigation = &nav;
            xlog_tick.ekf = &ekf;
            xlog_tick.tick = tick;
            xlog_tick.gnc_mode = cur_mode;
            xlog_tick.mission_state = static_cast<uint8_t>(mission_state);
            xlog_tick.gps_valid = gps_valid;
            xlog_tick.ekf_initialized = ekf_init;
            xlog_tick.have_external_setpoint = have_external_setpoint;
            xlog_tick.setpoint_age_s =
                ue_control_session.setpoint_age_s(t_now);
            xlog_tick.waypoint_distance_m = waypoint_distance_m;
            xlog_tick.dt = dt;
            xlog_tick.expected_dt = expected_dt;
            xlog_tick.wall_time = t_now;
            xlog_recorder.record_tick(xlog_tick);

            // UE-managed SITL already records this numeric series in XLog. Keep
            // the console status only for interactive standalone runs so the UE
            // session log remains an operational diagnostic rather than a
            // duplicate telemetry stream.
            if (cfg.parent_pid == 0 &&
                std::chrono::duration<double>(t_now - t_last_status).count() >= 1.0)
            {
                t_last_status = t_now;
                constexpr double kMsToKn = 1.0 / 0.514444;
                const double surge_kn = state.nu[0] * kMsToKn;
                const double gs_ms = std::sqrt(state.nu[0]*state.nu[0] + state.nu[1]*state.nu[1]);
                const double gs_kn = gs_ms * kMsToKn;
                const std::string runtime_suffix =
                    es.runtime_rem_s > 0.0
                        ? " rem=" +
                              std::to_string(
                                  static_cast<int>(es.runtime_rem_s / 60.0)) +
                              "min"
                        : std::string{};
                std::printf("[FC:%d] depth=%.2fm hdg=%.1fdeg "
                            "surge=%.2fm/s(%.1fkn) gs=%.2fm/s(%.1fkn) pqr=(%.2f,%.2f,%.2f) "
                            "tau=(X%.1f M%.1f N%.1f) act=(%.2f %.2f %.2f %.2f T%.2f) dvl=%s accel=%s "
                            "| rpm=%.0f T=%.1fN P=%.1fW SOC=%.1f%%%s\n",
                            cfg.ue5_port,
                            state.depth_m,
                            state.eta[5] * 180.0 / 3.14159265358979,
                            state.nu[0], surge_kn,
                            gs_ms, gs_kn,
                            state.nu[3],
                            state.nu[4],
                            state.nu[5],
                            tau[0],
                            tau[4],
                            tau[5],
                            norm[0],
                            norm[1],
                            norm[2],
                            norm[3],
                            norm[4],
                            nav.dvl_recent ? "BT" : (nav.water_dvl_recent ? "WT" : "--"),
                            nav.have_accel ? "ON" : "--",
                            ms.rpm_actual,
                            ms.thrust_N,
                            es.power_total_W,
                            es.soc * 100.0,
                            runtime_suffix.c_str());
            }

            // ── DDS Publish State @rate_hz ────────────────────────────────────
#ifdef HYDROX_DDS_ENABLED
            {
                sitl::DdsTelemetrySample dds_sample;
                hydrox::FcSnapshot &fs = dds_sample.snapshot;
                fs.timestamp_us = nav.imu.time_usec;
                for (int i = 0; i < 6; ++i)
                {
                    fs.eta[i] = state.eta[i];
                    fs.nu[i] = state.nu[i];
                    fs.truth_eta[i] = nav.truth.eta[i];
                    fs.truth_nu[i] = nav.truth.nu[i];
                }
                fs.depth_m = state.depth_m;
                fs.dvl_valid = state.dvl_valid ? 1u : 0u;
                fs.truth_valid = nav.truth_valid ? 1u : 0u;
                fs.acc[0] = nav.imu.xacc;
                fs.acc[1] = nav.imu.yacc;
                fs.acc[2] = nav.imu.zacc;
                fs.gyro[0] = nav.imu.xgyro;
                fs.gyro[1] = nav.imu.ygyro;
                fs.gyro[2] = nav.imu.zgyro;
                fs.dvl_vel[0] = last_dvl.vx;
                fs.dvl_vel[1] = last_dvl.vy;
                fs.dvl_vel[2] = last_dvl.vz;
                fs.gps_fix = last_gps.fix_type;
                fs.gps_satellites = last_gps.satellites_visible;
                if (gps_valid)
                {
                    fs.gps_lat = last_gps.lat;
                    fs.gps_lon = last_gps.lon;
                    fs.gps_alt = last_gps.alt;
                    fs.gps_vn = static_cast<double>(last_gps.vn) * 0.01; // cm/s -> m/s
                    fs.gps_ve = static_cast<double>(last_gps.ve) * 0.01;
                    fs.gps_vd = static_cast<double>(last_gps.vd) * 0.01;
                }
                fs.fins[0] = norm[0];
                fs.fins[1] = norm[1];
                fs.fins[2] = norm[2];
                fs.fins[3] = norm[3];
                for (int i = 0; i < 4; ++i)
                {
                    fs.fin_deg[i] =
                        static_cast<double>(fs.fins[i]) * vehicle_params.allocator.delta_max_deg;
                }
                fs.thrust = norm[4];
                fs.rpm = static_cast<float>(cmd.rpm);
                std::snprintf(fs.mission_state, sizeof(fs.mission_state),
                               "%s", mission_state_name(mission_state));
                // Motor model outputs.
                fs.motor_rpm_actual = static_cast<float>(ms.rpm_actual);
                fs.motor_thrust_N = static_cast<float>(ms.thrust_N);
                fs.motor_power_W = static_cast<float>(ms.power_W);
                fs.motor_current_A = static_cast<float>(ms.current_A);
                // Energy model outputs.
                fs.power_total_W = static_cast<float>(es.power_total_W);
                fs.energy_Wh = static_cast<float>(es.energy_Wh);
                fs.battery_soc = static_cast<float>(es.soc);
                fs.V_terminal = static_cast<float>(es.V_terminal);
                fs.runtime_rem_s = static_cast<float>(es.runtime_rem_s > 0 ? es.runtime_rem_s : 0.0);

                // Fill nav_msgs/Odometry covariance from the EKF covariance matrix.
                const auto &P = ekf.covariance();
                for (int r = 0; r < 6; ++r)
                {
                    for (int c = 0; c < 6; ++c)
                    {
                        fs.pose_cov[r * 6 + c] = P(r, c);
                    }
                }
                for (int r = 0; r < 3; ++r)
                {
                    for (int c = 0; c < 3; ++c)
                    {
                        fs.twist_cov[r * 6 + c] = P(6 + r, 6 + c);
                    }
                }
                fs.twist_cov[3 * 6 + 3] = 0.01;
                fs.twist_cov[4 * 6 + 4] = 0.01;
                fs.twist_cov[5 * 6 + 5] = 0.01;

                const char *mode_str = sitl::gnc_mode_name(cur_mode);
                dds_sample.gnc_mode = mode_str;
                dds_sample.hil_connected = true;
                dds_sample.ekf_initialized = ekf_init;
                dds_sample.passive_sonar = nav.passive_sonar;
                dds_sample.acoustic_neighbors = nav.acoustic_neighbors;
                dds_sample.rangefinder_scan = nav.rangefinder_scan;
                (void)dds_worker.try_submit_telemetry(std::move(dds_sample));
            }
#endif

            // ── QGC Telemetry @4Hz ─────────────────────────────────────────────
            if (qgc_sender.is_open() &&
                tick % static_cast<uint32_t>(std::max(1, effective_rate_hz / 4)) == 0)
            {
                const uint32_t tbm = static_cast<uint32_t>(nav.imu.time_usec / 1000ULL);
                constexpr double kR2D = 180.0 / 3.14159265358979323846;

                {
                    const auto packet = codec.encode_attitude(
                        static_cast<float>(state.eta[3]),
                        static_cast<float>(state.eta[4]),
                        static_cast<float>(state.eta[5]),
                        static_cast<float>(state.nu[3]),
                        static_cast<float>(state.nu[4]),
                        static_cast<float>(state.nu[5]), tbm);
                    qgc_sender.send(packet.data(), packet.size());
                }

                // LOCAL_POSITION_NED
                {
                    auto pkt = codec.encode_local_position_ned(
                        static_cast<float>(state.eta[0]), static_cast<float>(state.eta[1]),
                        static_cast<float>(state.eta[2]),
                        static_cast<float>(state.nu[0]), static_cast<float>(state.nu[1]),
                        static_cast<float>(state.nu[2]), tbm);
                    qgc_sender.send(pkt.data(), static_cast<int>(pkt.size()));
                }

                // VFR_HUD
                {
                    const float gs = static_cast<float>(
                        std::sqrt(state.nu[0] * state.nu[0] + state.nu[1] * state.nu[1]));
                    const float alt = -static_cast<float>(state.eta[2]);
                    const float climb = -static_cast<float>(state.nu[2]);
                    const int16_t hdg = static_cast<int16_t>(
                        std::fmod(state.eta[5] * kR2D + 360.0, 360.0));
                    const uint16_t thr = static_cast<uint16_t>(
                        std::clamp(std::abs(norm[4]) * 100.0f, 0.0f, 100.0f));
                    auto pkt = codec.encode_vfr_hud(gs, gs, alt, climb, hdg, thr);
                    qgc_sender.send(pkt.data(), static_cast<int>(pkt.size()));
                }

                // GLOBAL_POSITION_INT
                if (gps_valid)
                {
                    const int32_t rel_alt = static_cast<int32_t>(-state.eta[2] * 1000.0);
                    const int16_t vx = static_cast<int16_t>(state.nu[0] * 100.0);
                    const int16_t vy = static_cast<int16_t>(state.nu[1] * 100.0);
                    const int16_t vz = static_cast<int16_t>(state.nu[2] * 100.0);
                    const uint16_t hd = static_cast<uint16_t>(
                        std::fmod(state.eta[5] * kR2D + 360.0, 360.0) * 100.0);
                    auto pkt = codec.encode_global_position_int(
                        last_gps.lat, last_gps.lon, last_gps.alt, rel_alt,
                        vx, vy, vz, hd, tbm);
                    qgc_sender.send(pkt.data(), static_cast<int>(pkt.size()));
                }
            }

            // ── QGC SYS_STATUS @1Hz ───────────────────────────────────────
            if (qgc_sender.is_open() &&
                tick % static_cast<uint32_t>(effective_rate_hz) == 0)
            {
                constexpr uint32_t kPresent = 0x1000U | 0x40U | 0x20U | 0x08U | 0x02U | 0x01U;
                uint32_t health = 0x01U | 0x02U | 0x08U | 0x40U | 0x1000U;
                if (ekf_init)
                    health |= 0x20U;
                if (!gps_valid)
                    health &= ~0x20U;
                auto pkt = codec.encode_sys_status(kPresent, kPresent, health);
                qgc_sender.send(pkt.data(), static_cast<int>(pkt.size()));
            }

            if (use_hil_time)
                std::this_thread::sleep_for(1ms);
            else
                std::this_thread::sleep_until(t_prev + period);
        }

        // ── Disconnect Handling ──────────────────────────────────────────────────────
        const auto ue_disconnected_at = std::chrono::steady_clock::now();
        ue_control_session.on_disconnected(ue_disconnected_at);
        enter_safe_control_state("UE connection lost");
        transport.disconnect();
        if (!parent_guard.is_parent_alive())
        {
            std::printf("[FC] Parent process exited; stopping HydroX SITL.\n");
            break;
        }
        std::printf("[FC] Connection lost, preparing to reconnect...\n");
        if (qgc_sender.is_open())
        {
            auto pkt = codec.encode_statustext(4, "HydroX SITL disconnected");
            qgc_sender.send(pkt.data(), static_cast<int>(pkt.size()));
        }
    }

    std::printf("[FC] Exited.\n");
    return 0;
}
