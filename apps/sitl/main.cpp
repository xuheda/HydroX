/**
 * main_sitl.cpp — HydroX FC SITL process entry point
 *
 * Standalone HydroX autopilot process with no ROS runtime dependency.
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
 *                  [--vehicle vehicle0] [--vehicle-type EcaA9]
 *                  [--vehicle-bundle profiles/generic-auv-fin/vehicle-bundle.json]
 *                  [--vehicle-params path/to/legacy_params.json]
 *                  [--dds-host 127.0.0.1] [--dds-port 8888] [--ros-domain-id 0]
 *                  [--mavlink-signing-key-file D:\secure\hil.key]
 *                  [--mavlink-signing-link-id 0]
 *                  [--ekf-accel auto|off|on]
 *                  [--xlog auto|off|path.xlog]
 *                  [--parent-pid UE_PROCESS_ID]
 *                  [--publish-truth-state true|false]
 *                  [--allow-truth-heading-aid true|false]
 *                  [--control-feedback-source estimated_state|truth_debug]
 *                  [--rate 100]
 *                  [--depth 5.0] [--heading 0.0] [--surge 0.0]
 *                  [--mission-radius 3.0] [--mission-timeout 2.0]
 *                  [--mode DISABLED]
 */
#include "fossen_vehicle_params.h"
#include "vehicle_bundle.h"
#include "mavlink_signing.h"
#include "gnc/control_factory.h"
#include "gnc/control_interfaces.h"
#ifdef HYDROX_ENABLE_RESIDUAL_RL
#include "learning/residual_rl.h"
#endif
#include "hydrox/platform/host/host_clock.h"
#include "hydrox/platform/host/host_sleeper.h"
#include "hydrox/runtime/hil_runtime.h"
#include "mavlink_hil.h"
#include "odometry_contract.h"
#include "sensor_adapter.h"
#ifdef HYDROX_DDS_ENABLED
#include "sitl/dds_worker.h"
#endif
#include "sitl/control_feedback.h"
#include "sitl/sitl_config.h"
#include "sitl/sitl_platform.h"
#include "sitl/sitl_xlog.h"
#include "tcp_transport.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdio>
#include <deque>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <utility>

using namespace hydrox;
using namespace std::chrono_literals;
namespace sitl = hydrox::sitl;
namespace runtime = hydrox::runtime;

// ─────────────────────────────────────────────────────────────────────────────
// Global shutdown flag
// ─────────────────────────────────────────────────────────────────────────────
static std::atomic<bool> g_running{true};
static void sig_handler(int) { g_running = false; }

#ifdef HYDROX_ENABLE_RESIDUAL_RL
class ResidualAugmentor final : public runtime::IWrenchAugmentor
{
public:
    void reset() override { module_.reset(); }

    Wrench update(const NavigationState &estimated_state,
                  const GNCSetpoint &setpoint,
                  const Wrench &base_wrench,
                  double dt) override
    {
        return module_.update(
            {estimated_state, setpoint, base_wrench}, dt);
    }

private:
    learning::ResidualRlModule module_;
};
#endif

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
    hydrox::platform::host::HostClock monotonic_clock;
    hydrox::platform::host::HostSleeper monotonic_sleeper;

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
    std::printf("[FC] Control feedback: %s\n",
                sitl::control_feedback_source_name(
                    cfg.control_feedback_source));
    if (cfg.control_feedback_source ==
        sitl::ControlFeedbackSource::TruthDebug)
    {
        std::fprintf(
            stderr,
            "[FC] WARNING: TRUTH DEBUG feedback is enabled; this run does not validate estimator-closed-loop control.\n");
    }

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
    VehicleBundle vehicle_bundle;
    const bool use_vehicle_bundle = !cfg.vehicle_bundle.empty();
    FossenControlParams vehicle_params;
    if (use_vehicle_bundle)
    {
        vehicle_bundle = load_vehicle_bundle(cfg.vehicle_bundle, &params_error);
        vehicle_params = vehicle_bundle.control;
    }
    else
        vehicle_params = load_fossen_control_params(
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
    if (use_vehicle_bundle)
    {
        std::printf("[FC] Bundle: %s  contract=%s fingerprint=%016llx\n",
                    vehicle_bundle.id.c_str(),
                    vehicle_bundle.control_contract.c_str(),
                    static_cast<unsigned long long>(vehicle_bundle.fingerprint));
        if (vehicle_bundle.logical_actuator_count != 0)
            std::printf("[FC] Bundle allocation: actuators=%zu rank=%zu/6\n",
                        vehicle_bundle.logical_actuator_count,
                        vehicle_bundle.allocation_rank);
        for (const auto &issue : vehicle_bundle.validation)
        {
            if (issue.severity == BundleIssueSeverity::Warning)
                std::fprintf(stderr, "[FC] Bundle warning [%s]: %s\n",
                             issue.field.c_str(), issue.message.c_str());
        }
    }
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
    const EstimationProfile estimation_profile =
        estimation_profile_for(vehicle_params.vehicle_class);
    std::printf("[FC] Estimator: class=%s vertical=%s medium=%s estimate_medium=%s\n",
                sitl::vehicle_class_name(vehicle_params.vehicle_class),
                vertical_aid_mode_name(estimation_profile.vertical_aid),
                medium_velocity_kind_name(estimation_profile.medium_velocity_kind),
                estimation_profile.estimate_medium_velocity ? "ON" : "OFF");
    SensorAdapter::Params sensor_params(estimation_profile, accel_mode);
    sensor_params.gps_origin_lat_deg = cfg.gps_origin_lat_deg;
    sensor_params.gps_origin_lon_deg = cfg.gps_origin_lon_deg;
    sensor_params.gps_origin_altitude_msl_m = cfg.gps_origin_altitude_msl_m;
    sensor_params.gps_geodetic_max_radius_m = cfg.gps_max_radius_m;
    SensorAdapter sensor_adapter(sensor_params);
    // Build the {controller, allocator} stack for this vehicle's archetype.
    // Slender-body torpedoes get the cascade + fin allocator; thruster ROVs get
    // the 6-DOF controller + thruster-matrix allocator. main loop talks only the
    // IController/IAllocator interfaces below.
    ControlStack stack = use_vehicle_bundle
                             ? build_control_stack(vehicle_bundle)
                             : build_control_stack(vehicle_params);
#ifdef HYDROX_ENABLE_RESIDUAL_RL
    // A deployment supplies a model-backed IResidualPolicy and explicit per-axis
    // limits before learned residual authority is enabled.
    ResidualAugmentor residual_augmentor;
#endif

    const GNCMode startup_mode = sitl::gnc_mode_from_string(initial_mode);
    runtime::HilRuntimeConfig runtime_config;
    runtime_config.estimation_profile = estimation_profile;
    runtime_config.initial_state.eta[0] = cfg.init_n;
    runtime_config.initial_state.eta[1] = cfg.init_e;
    runtime_config.initial_state.eta[2] = cfg.init_depth;
    runtime_config.initial_state.eta[5] = cfg.init_heading;
    if (startup_mode != GNCMode::DISABLED)
        runtime_config.initial_state.nu[0] = cfg.init_surge;
    runtime_config.initial_state.depth_m = cfg.init_depth;
    runtime_config.initial_setpoint.depth_ref = cfg.init_depth;
    runtime_config.initial_setpoint.heading_ref = cfg.init_heading;
    runtime_config.initial_setpoint.surge_ref = cfg.init_surge;
    runtime_config.motor = vehicle_params.motor;
    runtime_config.control_feedback_source = cfg.control_feedback_source;
    runtime_config.allow_truth_heading_aid = cfg.allow_truth_heading_aid;
    runtime_config.nominal_dt_s =
        1.0 / static_cast<double>(effective_rate_hz);
    runtime_config.mission_radius_m = cfg.mission_radius;
    runtime_config.setpoint_timeout_us =
        runtime::duration_us_from_seconds(cfg.mission_timeout_s);

    runtime::HilRuntime flight_runtime(
        runtime_config,
        std::move(stack.controller),
        std::move(stack.allocator)
#ifdef HYDROX_ENABLE_RESIDUAL_RL
            ,
        &residual_augmentor
#endif
    );
    if (!flight_runtime.valid())
    {
        std::fprintf(stderr, "[FC] FATAL: invalid controller/allocation stack\n");
        return 2;
    }
#ifdef HYDROX_DDS_ENABLED
    // Micro XRCE-DDS is isolated from the real-time control loop. The worker
    // owns all network/session calls; this thread only uses non-blocking
    // latest-value mailboxes.
    sitl::DdsWorker dds_worker(
        {
            cfg.dds_host,
            cfg.dds_port,
            cfg.ros_domain_id,
            cfg.vehicle,
            static_cast<uint32_t>(cfg.ue5_port),
            cfg.publish_truth_state,
        },
        monotonic_clock);
    uint64_t last_dds_setpoint_sequence = 0;
    uint64_t last_dds_status_sequence = 0;
    sitl::DdsControlLinkState dds_control_link;

    const auto setpoint_from_dds = [](const hydrox::GNCSetpointDds &isp)
    {
        GNCSetpoint sp;
        sp.depth_ref = isp.depth_ref;
        sp.heading_ref = isp.heading_ref;
        sp.surge_ref = isp.surge_ref;
        sp.use_yaw_rate_ref = isp.use_yaw_rate_ref;
        sp.yaw_rate_ref = isp.yaw_rate_ref;
        sp.wp_n = isp.wp_n;
        sp.wp_e = isp.wp_e;
        sp.wp_d = isp.wp_d;
        return sp;
    };

    const auto revoke_dds_setpoint = [&](platform::MonotonicTimeUs now_us,
                                         const char *reason)
    {
        (void)flight_runtime.revoke_setpoint(now_us);
        std::fprintf(stderr, "[FC] %s; control set to DISABLED\n", reason);
    };
#endif

    // ── QGC UDP Socket (Broadcast) ─────────────────────────────────────────────
    sitl::UdpSender qgc_sender(cfg.qgc_host, cfg.qgc_port, true);
    if (qgc_sender.is_open())
        std::printf("[FC] QGC UDP ready → %s:%d\n",
                    cfg.qgc_host.c_str(), cfg.qgc_port);

    const double expected_dt = 1.0 / static_cast<double>(effective_rate_hz);
    xlog_recorder.start_session_clock();

    // ── Disconnect Reconnection Outer Loop ────────────────────────────────────────────────────
    while (g_running && parent_guard.is_parent_alive())
    {
        std::printf("[FC] Connecting to UE5 %s:%d ...\n",
                    cfg.ue5_host.c_str(), cfg.ue5_port);
        if (!transport.connect())
        {
            std::printf("[FC] Connection failed, retrying in 1s\n");
            monotonic_sleeper.sleep_for_us(1'000'000);
            continue;
        }
        const auto ue_connected_at_us = monotonic_clock.now_us();
        const uint64_t ue_session_generation =
            flight_runtime.on_connected(ue_connected_at_us);
        std::printf(
            "[FC] Connected! timestamp-driven GNC loop nominal=%dHz\n",
            cfg.rate_hz);
        std::printf("[FC] UE control session=%llu waiting for a valid HIL_SENSOR\n",
                    static_cast<unsigned long long>(ue_session_generation));
        if (qgc_sender.is_open())
        {
            const auto packet =
                codec.encode_statustext(6, "HydroX SITL connected");
            qgc_sender.send(packet.data(), packet.size());
        }

        sensor_adapter.reset();

        bool gps_valid = false;
        HilGpsMsg last_gps{};
        HilDvlMsg last_dvl{};
        bool startup_setpoint_pending =
            cfg.parent_pid == 0 && startup_mode != GNCMode::DISABLED;

        auto t_last_status = std::chrono::steady_clock::now();
        auto t_last_heartbeat = t_last_status;
        auto t_last_imu = t_last_status;
        auto t_last_no_imu_warning = t_last_status - 2s;
        std::deque<NavigationInput> pending_control_inputs;
        uint64_t last_queued_imu_time_us = 0;
        uint32_t total_bytes_without_imu = 0;
        bool simulator_paused = false;

        // ── 100Hz GNC Inner Loop ──────────────────────────────────────────────
        while (g_running && parent_guard.is_parent_alive() && transport.is_connected())
        {
            // ── Read UE5 HIL data ─────────────────────────────────────────
            int n = 0;
            if (pending_control_inputs.empty())
            {
                const int wait_result = transport.wait_readable(10);
                if (wait_result < 0)
                    break;

                uint8_t buf[1024];
                n = transport.read(buf, sizeof(buf));

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
                        if (f.msg_id == MSGID_HIL_SENSOR)
                        {
                            NavigationInput input = sensor_adapter.build();
                            const uint64_t timestamp_us = input.imu.time_usec;
                            if (input.got_imu && timestamp_us > last_queued_imu_time_us)
                            {
                                pending_control_inputs.push_back(std::move(input));
                                last_queued_imu_time_us = timestamp_us;
                            }
                            else if (timestamp_us > 0)
                            {
                                std::fprintf(
                                    stderr,
                                    "[FC:%d] rejected non-increasing sensor timestamp current=%llu previous=%llu\n",
                                    cfg.ue5_port,
                                    static_cast<unsigned long long>(timestamp_us),
                                    static_cast<unsigned long long>(last_queued_imu_time_us));
                            }

                            // An IMU sample closes one control input epoch.
                            // Auxiliary frames that follow belong to the next
                            // timestamped control input.
                            sensor_adapter.begin_cycle();
                        }
                    }
                }
                else if (n < 0)
                {
                    break;
                }
            }

            NavigationInput nav;
            if (!pending_control_inputs.empty())
            {
                nav = std::move(pending_control_inputs.front());
                pending_control_inputs.pop_front();
            }
            else
            {
                nav = sensor_adapter.build();
            }

            const auto t_now = std::chrono::steady_clock::now();
            const auto monotonic_now_us = monotonic_clock.now_us();
            gps_valid = nav.gps_valid;
            last_gps = nav.last_gps;
            last_dvl = nav.last_dvl;

            // Evaluate wall-clock loss before a newly arrived sample refreshes
            // the sensor timestamp. A late packet cannot erase an outage.
            const runtime::RuntimeEvent maintenance_event =
                flight_runtime.maintain(monotonic_now_us);
            if (maintenance_event != runtime::RuntimeEvent::NONE)
            {
                std::fprintf(
                    stderr,
                    "[FC] %s; control set to DISABLED\n",
                    runtime::runtime_event_name(maintenance_event));
            }

            // A new simulator byte stream is a new state epoch. Only a real
            // timestamped IMU sample opens the gate for post-reconnect commands;
            // the default value produced by a malformed short packet cannot.
            if (nav.got_imu && nav.imu.time_usec > 0 &&
                flight_runtime.observe_valid_sensor(monotonic_now_us) ==
                    runtime::RuntimeEvent::SENSOR_READY)
            {
                std::printf(
                    "[FC] UE control session=%llu sensor-ready; waiting for a fresh Setpoint\n",
                    static_cast<unsigned long long>(ue_session_generation));
                if (startup_setpoint_pending)
                {
                    startup_setpoint_pending = false;
                    if (flight_runtime.accept_setpoint(
                            runtime_config.initial_setpoint,
                            startup_mode,
                            monotonic_now_us))
                    {
                        std::printf(
                            "[FC] standalone startup Setpoint accepted for mode=%s\n",
                            sitl::gnc_mode_name(startup_mode));
                    }
                }
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
                    revoke_dds_setpoint(
                        monotonic_now_us, "DDS connection lost");
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
                else if (!flight_runtime.accept_setpoint(
                             setpoint_from_dds(incoming_setpoint.setpoint),
                             static_cast<GNCMode>(
                                 incoming_setpoint.setpoint.mode),
                             incoming_setpoint.received_at_us))
                {
                    std::fprintf(
                        stderr,
                        "[FC][DDS] pre-session Setpoint rejected dds_session=%llu "
                        "ue_session=%llu phase=%s\n",
                        static_cast<unsigned long long>(
                            incoming_setpoint.session_generation),
                        static_cast<unsigned long long>(
                            ue_session_generation),
                        runtime::control_session_phase_name(
                            flight_runtime.control_session().phase()));
                }
            }
#endif

            if (simulator_paused)
            {
                t_last_imu = t_now;
                t_last_no_imu_warning = t_now - 2s;
                total_bytes_without_imu = 0;
                pending_control_inputs.clear();
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
                continue;
            }

            t_last_imu = t_now;
            t_last_no_imu_warning = t_now - 2s;
            total_bytes_without_imu = 0;

            const runtime::StepStatus step_status =
                flight_runtime.step(nav, monotonic_now_us);
            if (step_status != runtime::StepStatus::OK)
            {
                std::fprintf(
                    stderr,
                    "[FC:%d] common runtime rejected sensor tick: %s\n",
                    cfg.ue5_port,
                    runtime::step_status_name(step_status));
                continue;
            }

            // Everything below consumes one immutable result from the common
            // SITL/HITL estimator -> GNC -> allocator -> safety pipeline.
            const runtime::HilRuntimeTick &runtime_tick =
                flight_runtime.last_tick();
            const NavigationState &state = runtime_tick.estimated_state;
            const NavigationState &control_state = runtime_tick.control_state;
            const Wrench &tau = runtime_tick.wrench;
            const ActuatorCmd &cmd = runtime_tick.actuator;
            const auto &norm = cmd.ch;
            const MotorState &ms = runtime_tick.motor;
            const EnergyState &es = runtime_tick.energy;
            const GNCSetpoint &sp = flight_runtime.setpoint();
            const GNCMode cur_mode = runtime_tick.mode;
            const runtime::MissionState mission_state =
                runtime_tick.mission_state;
            const uint32_t tick = runtime_tick.tick;
            const bool ekf_init = runtime_tick.ekf_initialized;
            const bool have_external_setpoint =
                runtime_tick.have_external_setpoint;
            const double waypoint_distance_m =
                runtime_tick.waypoint_distance_m;
            const double dt = runtime_tick.dt;

            // Send an explicit zero/unarmed frame in safe states. This removes
            // ambiguity at both the simulator and the hardware HIL router.
            const auto act_pkt = codec.encode_hil_actuator_controls(
                norm,
                runtime_tick.sensor_time_us,
                runtime_tick.actuator_mode,
                0);
            transport.write(act_pkt.data(), act_pkt.size());

            sitl::XLogTickData xlog_tick;
            xlog_tick.state = &control_state;
            xlog_tick.setpoint = &sp;
            xlog_tick.wrench = &tau;
            xlog_tick.actuator = &cmd;
            xlog_tick.navigation = &nav;
            xlog_tick.ekf = &flight_runtime.ekf();
            xlog_tick.tick = tick;
            xlog_tick.gnc_mode = cur_mode;
            xlog_tick.mission_state = static_cast<uint8_t>(mission_state);
            xlog_tick.gps_valid = gps_valid;
            xlog_tick.ekf_initialized = ekf_init;
            xlog_tick.have_external_setpoint = have_external_setpoint;
            xlog_tick.setpoint_age_s =
                runtime_tick.setpoint_age_s;
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
                const double surge_kn = control_state.nu[0] * kMsToKn;
                const double gs_ms = std::sqrt(control_state.nu[0]*control_state.nu[0] + control_state.nu[1]*control_state.nu[1]);
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
                            "tau=(X%.1f M%.1f N%.1f) act=(%.2f %.2f %.2f %.2f T%.2f) dvl=%s accel=%s feedback=%s "
                            "| rpm=%.0f T=%.1fN P=%.1fW SOC=%.1f%%%s\n",
                            cfg.ue5_port,
                            control_state.depth_m,
                            control_state.eta[5] * 180.0 / 3.14159265358979,
                            control_state.nu[0], surge_kn,
                            gs_ms, gs_kn,
                            control_state.nu[3],
                            control_state.nu[4],
                            control_state.nu[5],
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
                            runtime_tick.used_truth ? "TRUTH_DEBUG" : "EKF",
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
                    fs.eta[i] = control_state.eta[i];
                    fs.nu[i] = control_state.nu[i];
                    fs.truth_eta[i] = nav.truth.eta[i];
                    fs.truth_nu[i] = nav.truth.nu[i];
                }
                fs.depth_m = control_state.depth_m;
                fs.dvl_valid = control_state.dvl_valid ? 1u : 0u;
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
                for (size_t i = 0; i < norm.size(); ++i)
                    fs.normalized[i] = norm[i];
                if (use_vehicle_bundle && vehicle_bundle.logical_actuator_count > 0)
                    fs.actuator_channel_count = static_cast<uint8_t>(
                        std::min<size_t>(vehicle_bundle.logical_actuator_count, norm.size()));
                else
                {
                    switch (vehicle_params.archetype)
                    {
                    case VehicleArchetype::DifferentialDrive:
                    case VehicleArchetype::Surface: fs.actuator_channel_count = 2; break;
                    case VehicleArchetype::Multirotor:
                    case VehicleArchetype::FixedWing: fs.actuator_channel_count = 4; break;
                    case VehicleArchetype::SlenderBodyFin: fs.actuator_channel_count = 5; break;
                    case VehicleArchetype::Thruster:
                    case VehicleArchetype::VTOL: fs.actuator_channel_count = 8; break;
                    }
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
                               "%s", runtime::mission_state_name(mission_state));
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

                // Encode map_ned pose and BodyFRD twist covariance under the
                // shared nav_msgs/Odometry contract.
                const auto &P = flight_runtime.ekf().covariance();
                fs.odometry_covariance_valid =
                    odometry_contract::encode_covariances(
                        P, fs.pose_cov, fs.twist_cov)
                        ? 1u
                        : 0u;

                const char *mode_str = sitl::gnc_mode_name(cur_mode);
                dds_sample.gnc_mode = mode_str;
                dds_sample.hil_connected = true;
                dds_sample.ekf_initialized = ekf_init;
                dds_sample.actuator_authorized = runtime_tick.actuator_authorized;
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

        }

        // ── Disconnect Handling ──────────────────────────────────────────────────────
        (void)flight_runtime.on_disconnected(monotonic_clock.now_us());
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
