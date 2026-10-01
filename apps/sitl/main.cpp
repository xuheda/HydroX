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
 *                  [--mavlink-system-id 1]
 *                  [--vehicle vehicle0] [--vehicle-type EcaA9]
 *                  [--vehicle-bundle profiles/generic-auv-fin/vehicle-bundle.json]
 *                  [--dds-host 127.0.0.1] [--dds-port 8888] [--ros-domain-id 0]
 *                  [--mavlink-signing-key-file D:\secure\hil.key]
 *                  [--mavlink-signing-link-id 0]
 *                  [--ekf-accel auto|off|on]
 *                  [--xlog auto|path.xlog]
 *                  [--log-directory path] [--run-id id]
 *                  [--parent-pid UE_PROCESS_ID]
 *                  [--publish-truth-state true|false]
 *                  [--allow-truth-heading-aid true|false]
 *                  [--control-feedback-source estimated_state|truth_debug]
 *                  [--rate 100]
 *                  [--residual-policy-mode disabled|shadow|active]
 *                  [--residual-policy-host 127.0.0.1]
 *                  [--residual-policy-port 14750]
 *                  [--residual-policy-local-port 14751]
 *                  [--residual-policy-nonce NONZERO_UINT64]
 *                  [--depth 5.0] [--heading 0.0] [--surge 0.0]
 *                  [--mission-radius 3.0] [--mission-timeout 2.0]
 *                  [--mode DISABLED]
 */
#include "control_parameters.h"
#include "vehicle_bundle.h"
#include "mavlink_signing.h"
#include "gnc/control_factory.h"
#include "gnc/control_interfaces.h"
#ifdef HYDROX_ENABLE_RESIDUAL_RL
#include "learning/residual_rl.h"
#include "sitl/residual_policy_udp.h"
#endif
#include "hydrox/platform/host/host_clock.h"
#include "hydrox/platform/host/host_sleeper.h"
#include "hydrox/runtime/hil_session_driver.h"
#include "hydrox/runtime/hil_session_mapping.h"
#include "hydrox/runtime/hil_runtime.h"
#include "mavlink_hil.h"
#include "odometry_contract.h"
#include "sensor_adapter.h"
#ifdef HYDROX_DDS_ENABLED
#include "sitl/dds_worker.h"
#endif
#include "hydrox/runtime/control_feedback.h"
#include "sitl/gcs_runtime_bridge.h"
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
#include <limits>
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
    ResidualAugmentor(
        const learning::ResidualSafetyFilter::Params &params,
        std::unique_ptr<learning::IResidualPolicy> policy)
        : module_(params, std::move(policy))
    {
    }

    void reset() override { module_.reset(); }

    const sitl::UdpResidualPolicyStats *policy_stats() const
    {
        const auto *policy = dynamic_cast<const sitl::UdpResidualPolicy *>(
            module_.policy());
        return policy != nullptr ? &policy->stats() : nullptr;
    }

    Wrench update(const NavigationState &estimated_state,
                  const GNCSetpoint &setpoint,
                  const Wrench &base_wrench,
                  const ActuatorCmd &previous_actuator,
                  double dt) override
    {
        return module_.update(
            {estimated_state, setpoint, base_wrench,
             Wrench::Zero(), previous_actuator}, dt);
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
    std::printf("[FC] MAVLink: sysid=%u compid=1\n",
                static_cast<unsigned>(cfg.mavlink_system_id));
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
        runtime::ControlFeedbackSource::TruthDebug)
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
    const VehicleBundle vehicle_bundle = load_vehicle_bundle(cfg.vehicle_bundle, &params_error);
    const ControlParameters& vehicle_params = vehicle_bundle.control;
    if (!vehicle_bundle.valid)
    {
        std::fprintf(stderr,
                     "[FC] FATAL: vehicle params unavailable for type '%s': %s\n",
                     cfg.vehicle_type.c_str(), params_error.c_str());
        return 2;
    }
    std::printf("[FC] Params: %s\n", vehicle_params.source_path.c_str());
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
    SensorAdapter::Params sensor_params(estimation_profile);
    // Build the {controller, allocator} stack for this vehicle's archetype.
    // Slender-body torpedoes get the cascade + fin allocator; thruster ROVs get
    // the 6-DOF controller + thruster-matrix allocator. main loop talks only the
    // IController/IAllocator interfaces below.
    ControlStack stack = build_control_stack(vehicle_bundle);
#ifdef HYDROX_ENABLE_RESIDUAL_RL
    std::unique_ptr<ResidualAugmentor> residual_augmentor;
    if (cfg.residual_policy_mode != sitl::ResidualPolicyMode::Disabled)
    {
        sitl::UdpResidualPolicyConfig policy_config;
        policy_config.host = cfg.residual_policy_host;
        policy_config.remote_port = cfg.residual_policy_port;
        policy_config.local_port = cfg.residual_policy_local_port;
        policy_config.nonce = cfg.residual_policy_nonce;
        policy_config.control_hz = effective_rate_hz;
        policy_config.policy_hz = cfg.residual_policy_hz;
        policy_config.timeout_ms = cfg.residual_policy_timeout_ms;
        auto policy = std::make_unique<sitl::UdpResidualPolicy>(
            policy_config);
        if (!policy->ready())
        {
            std::fprintf(stderr,
                         "[FC] FATAL: local residual policy transport could not bind/open\n");
            return 2;
        }
        learning::ResidualSafetyFilter::Params safety;
        safety.enabled = true;
        safety.blend = cfg.residual_blend;
        safety.min_confidence = cfg.residual_min_confidence;
        safety.max_delta[0] = cfg.residual_max_delta[0];
        safety.max_delta[4] = cfg.residual_max_delta[1];
        safety.max_delta[5] = cfg.residual_max_delta[2];
        safety.max_rate[0] = cfg.residual_max_rate[0];
        safety.max_rate[4] = cfg.residual_max_rate[1];
        safety.max_rate[5] = cfg.residual_max_rate[2];
        residual_augmentor = std::make_unique<ResidualAugmentor>(
            safety, std::move(policy));
        std::printf(
            "[FC] Residual policy: %s loopback=%s:%u local=%u rate=%dHz timeout=%.3fms nonce=%llu\n",
            sitl::residual_policy_mode_name(cfg.residual_policy_mode),
            cfg.residual_policy_host.c_str(),
            static_cast<unsigned>(cfg.residual_policy_port),
            static_cast<unsigned>(cfg.residual_policy_local_port),
            cfg.residual_policy_hz,
            cfg.residual_policy_timeout_ms,
            static_cast<unsigned long long>(cfg.residual_policy_nonce));
    }
#else
    if (cfg.residual_policy_mode != sitl::ResidualPolicyMode::Disabled)
    {
        std::fprintf(
            stderr,
            "[FC] FATAL: residual policy requested but this binary was built without HYDROX_ENABLE_RESIDUAL_RL\n");
        return 2;
    }
#endif

    const GNCMode startup_mode = sitl::gnc_mode_from_string(initial_mode);
    runtime::HilRuntimeConfig runtime_config;
    runtime_config.estimation_profile = estimation_profile;
    runtime_config.safety_profile = safety::safety_profile_for(vehicle_params);
    // Shared runtime safety supervision is mandatory on every target.
    runtime_config.wrench_augmentor_active_enabled =
        cfg.residual_policy_mode == sitl::ResidualPolicyMode::Active;
    runtime::HilSessionConfigValues session_values;
    if (!runtime::hil_session_period_from_rate(
            effective_rate_hz, session_values.nominal_period_us))
    {
        std::fprintf(
            stderr,
            "[FC] --rate must produce an exact 1 ms..100 ms period\n");
        return 2;
    }
    session_values.initial_n_m = cfg.init_n;
    session_values.initial_e_m = cfg.init_e;
    session_values.initial_down_m = cfg.init_depth;
    session_values.initial_heading_rad = cfg.init_heading;
    session_values.initial_surge_mps =
        startup_mode != GNCMode::DISABLED ? cfg.init_surge : 0.0;
    session_values.gps_origin_lat_deg = cfg.gps_origin_lat_deg;
    session_values.gps_origin_lon_deg = cfg.gps_origin_lon_deg;
    session_values.gps_origin_altitude_msl_m =
        cfg.gps_origin_altitude_msl_m;
    session_values.gps_max_radius_m = cfg.gps_max_radius_m;
    session_values.mission_radius_m = cfg.mission_radius;
    const auto setpoint_timeout_us =
        runtime::duration_us_from_seconds(cfg.mission_timeout_s);
    if (setpoint_timeout_us >
        std::numeric_limits<uint32_t>::max())
    {
        std::fprintf(
            stderr,
            "[FC] --mission-timeout exceeds Session V1 wire capacity\n");
        return 2;
    }
    session_values.setpoint_timeout_us =
        static_cast<uint32_t>(setpoint_timeout_us);
    session_values.allow_truth_heading_aid =
        cfg.allow_truth_heading_aid;
    session_values.feedback_source =
        cfg.control_feedback_source ==
                runtime::ControlFeedbackSource::TruthDebug
            ? runtime::HilSessionFeedbackSource::TruthDebug
            : runtime::HilSessionFeedbackSource::EstimatedState;
    switch (accel_mode)
    {
    case AccelMode::Off:
        session_values.accel_mode = runtime::HilSessionAccelMode::Off;
        break;
    case AccelMode::On:
        session_values.accel_mode = runtime::HilSessionAccelMode::On;
        break;
    default:
        session_values.accel_mode = runtime::HilSessionAccelMode::Auto;
        break;
    }
    runtime::HilSessionConfigV1 session_config;
    runtime::HilSessionField session_field = runtime::HilSessionField::None;
    if (!runtime::resolve_hil_session_config(
            session_values, session_config, session_field))
    {
        std::fprintf(
            stderr,
            "[FC] Invalid HIL session configuration field %u\n",
            static_cast<unsigned int>(session_field));
        return 2;
    }
    runtime::apply_hil_session_config(
        session_config, runtime_config, sensor_params);
    sitl::XLogIdentity xlog_identity;
    xlog_identity.profile_id = vehicle_bundle.id;
    xlog_identity.control_contract = vehicle_bundle.control_contract;
    xlog_identity.profile_fingerprint = vehicle_bundle.fingerprint;
    xlog_identity.profile_bound = true;
    xlog_identity.session_config = session_config;
    xlog_identity.session_digest = runtime::hil_session_digest(
        xlog_identity.profile_fingerprint, session_config);
    sitl::XLogRecorder xlog_recorder(
        cfg, vehicle_params, accel_mode, effective_rate_hz, xlog_identity);
    runtime_config.initial_setpoint.depth_ref = cfg.init_depth;
    runtime_config.initial_setpoint.heading_ref = cfg.init_heading;
    runtime_config.initial_setpoint.surge_ref = cfg.init_surge;
    runtime_config.motor = vehicle_params.motor;

    runtime::HilRuntime flight_runtime(
        runtime_config,
        std::move(stack.controller),
        std::move(stack.allocator)
#ifdef HYDROX_ENABLE_RESIDUAL_RL
            ,
        residual_augmentor.get()
#endif
    );
    if (!flight_runtime.valid())
    {
        std::fprintf(stderr, "[FC] FATAL: invalid controller/allocation stack\n");
        return 2;
    }
    uint64_t last_safety_transition_sequence = 0;
    bool last_navigation_degraded = false;
    runtime::HilSessionDriver hil_session(
        codec, flight_runtime, sensor_params);
    const auto send_last_actuator = [&]()
    {
        const runtime::HilRuntimeTick &tick = flight_runtime.last_tick();
        const auto packet = codec.encode_hil_actuator_controls(
            tick.actuator.ch,
            tick.sensor_time_us,
            tick.actuator_mode,
            0);
        return transport.write(packet.data(), packet.size());
    };
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
        sp.use_path_segment = isp.use_path_segment;
        sp.path_start_n = isp.path_start_n;
        sp.path_start_e = isp.path_start_e;
        sp.lookahead_m = isp.lookahead_m;
        sp.arrival_radius_m = isp.arrival_radius_m;
        sp.hold_heading = isp.hold_heading;
        return sp;
    };

    const auto revoke_dds_setpoint = [&](platform::MonotonicTimeUs now_us,
                                         const char *reason)
    {
        const runtime::HilSessionResult revoked =
            hil_session.revoke_setpoint(now_us);
        if (revoked.actuator_frame_required)
            (void)send_last_actuator();
        std::fprintf(stderr, "[FC] %s; control set to DISABLED\n", reason);
    };
#endif

    // ── QGC UDP Socket (Broadcast) ─────────────────────────────────────────────
    sitl::UdpSender qgc_sender(cfg.qgc_host, cfg.qgc_port, true);
    sitl::GcsRuntimeBridge gcs_bridge(
        qgc_sender, cfg.mavlink_system_id, 1, mav_type);
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
        gcs_bridge.set_link_connected(false);
        gcs_bridge.service(flight_runtime, monotonic_clock.now_us());
        if (!transport.connect())
        {
            std::printf("[FC] Connection failed, retrying in 1s\n");
            monotonic_sleeper.sleep_for_us(1'000'000);
            continue;
        }
        const auto ue_connected_at_us = monotonic_clock.now_us();
        uint64_t ue_session_generation =
            hil_session.on_connected(ue_connected_at_us).generation;
        std::printf(
            "[FC] Connected! timestamp-driven GNC loop nominal=%dHz\n",
            cfg.rate_hz);
        std::printf("[FC] UE control session=%llu waiting for a valid HIL_SENSOR\n",
                    static_cast<unsigned long long>(ue_session_generation));
        gcs_bridge.set_link_connected(true);
        (void)gcs_bridge.send_statustext(
            6, "HydroX SITL connected");

        bool gps_valid = false;
        HilGpsMsg last_gps{};
        HilDvlMsg last_dvl{};
        bool startup_setpoint_pending =
            cfg.parent_pid == 0 && startup_mode != GNCMode::DISABLED;
        bool accepted_external_setpoint_logged = false;

        auto t_last_status = std::chrono::steady_clock::now();
        auto t_last_heartbeat = t_last_status;
        auto t_last_imu = t_last_status;
        auto t_last_no_imu_warning = t_last_status - 2s;
        std::deque<NavigationInput> pending_control_inputs;
        uint32_t total_bytes_without_imu = 0;

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
                        const runtime::HilSessionResult frame_result =
                            hil_session.ingest_frame(
                                f, monotonic_clock.now_us());
                        if (frame_result.pause_changed)
                        {
                            std::printf(
                                "[FC:%d] simulator control chain %s\n",
                                cfg.ue5_port,
                                frame_result.simulator_paused
                                    ? "paused" : "resuming");
                        }
                        if (frame_result.actuator_frame_required)
                            (void)send_last_actuator();
                        if (frame_result.has_sensor_input)
                        {
                            pending_control_inputs.push_back(
                                frame_result.sensor_input);
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
                nav = hil_session.navigation_snapshot();
            }

            const auto t_now = std::chrono::steady_clock::now();
            const auto monotonic_now_us = monotonic_clock.now_us();
            gcs_bridge.service(flight_runtime, monotonic_now_us);
            gps_valid = nav.gps_valid;
            last_gps = nav.last_gps;
            last_dvl = nav.last_dvl;

            // Evaluate wall-clock loss before a newly arrived sample refreshes
            // the sensor timestamp. A late packet cannot erase an outage.
            const runtime::HilSessionResult maintenance =
                hil_session.maintain(monotonic_now_us);
            xlog_recorder.record_safety(flight_runtime.last_tick(), monotonic_now_us, maintenance.runtime_event);
            if (maintenance.runtime_event != runtime::RuntimeEvent::NONE)
            {
                std::fprintf(
                    stderr,
                    "[FC] %s; control set to DISABLED\n",
                    runtime::runtime_event_name(maintenance.runtime_event));
                if (maintenance.actuator_frame_required)
                    (void)send_last_actuator();
            }

            const runtime::HilSessionResult prepared =
                nav.got_imu && nav.imu.time_usec > 0 &&
                        !hil_session.simulator_paused()
                    ? hil_session.prepare_sensor(nav, monotonic_now_us)
                    : runtime::HilSessionResult{};
            xlog_recorder.record_safety(flight_runtime.last_tick(), monotonic_now_us, prepared.runtime_event);
            if (prepared.sensor_epoch_restarted)
            {
                ue_session_generation = prepared.generation;
                std::printf(
                    "[FC] UE sensor timestamp epoch restarted; control session=%llu\n",
                    static_cast<unsigned long long>(ue_session_generation));
            }
            // A new simulator byte stream is a new state epoch. Only a real
            // timestamped IMU sample opens the gate for post-reconnect commands;
            // the default value produced by a malformed short packet cannot.
            if (prepared.sensor_ready)
            {
                std::printf(
                    "[FC] UE control session=%llu sensor-ready; waiting for a fresh Setpoint\n",
                    static_cast<unsigned long long>(ue_session_generation));
                if (startup_setpoint_pending)
                {
                    startup_setpoint_pending = false;
                    if (hil_session.accept_setpoint(
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
                else if (!hil_session.accept_setpoint(
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
                else if (!accepted_external_setpoint_logged &&
                         static_cast<GNCMode>(incoming_setpoint.setpoint.mode) !=
                             GNCMode::DISABLED)
                {
                    accepted_external_setpoint_logged = true;
                    std::printf(
                        "[FC][DDS] external Setpoint accepted; control armed mode=%s\n",
                        sitl::gnc_mode_name(static_cast<GNCMode>(
                            incoming_setpoint.setpoint.mode)));
                }
            }
#endif

            if (hil_session.simulator_paused())
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
            if (!prepared.sensor_prepared)
            {
                if (prepared.sensor_rejected &&
                    prepared.step_status != runtime::StepStatus::NO_SENSOR)
                {
                    std::fprintf(
                        stderr,
                        "[FC:%d] common session rejected sensor tick: %s\n",
                        cfg.ue5_port,
                        runtime::step_status_name(prepared.step_status));
                }
                continue;
            }

            t_last_imu = t_now;
            t_last_no_imu_warning = t_now - 2s;
            total_bytes_without_imu = 0;

            const runtime::HilSessionResult stepped =
                hil_session.step_sensor(monotonic_now_us);
            xlog_recorder.record_safety(flight_runtime.last_tick(), monotonic_now_us, stepped.runtime_event);
            if (stepped.step_status != runtime::StepStatus::OK)
            {
                std::fprintf(
                    stderr,
                    "[FC:%d] common runtime rejected sensor tick: %s\n",
                    cfg.ue5_port,
                    runtime::step_status_name(stepped.step_status));
                if (stepped.actuator_frame_required)
                    (void)send_last_actuator();
                continue;
            }

            // Everything below consumes one immutable result from the common
            // SITL/HITL estimator -> GNC -> allocator -> safety pipeline.
            const runtime::HilRuntimeTick &runtime_tick =
                flight_runtime.last_tick();
            if (runtime_tick.safety_status.transition_sequence !=
                last_safety_transition_sequence)
            {
                last_safety_transition_sequence =
                    runtime_tick.safety_status.transition_sequence;
                std::printf(
                    "[FC][SAFETY] seq=%llu mode=%s action=%s reason=%s source=%s active=1\n",
                    static_cast<unsigned long long>(
                        last_safety_transition_sequence),
                    safety::vehicle_mode_name(
                        runtime_tick.safety_status.mode),
                    safety::safety_action_name(
                        runtime_tick.safety_status.action),
                    safety::safety_reason_name(
                        runtime_tick.safety_status.reason),
                    safety::control_source_name(
                        runtime_tick.safety_control_source));
            }
            if (runtime_tick.safety_navigation_degraded != last_navigation_degraded)
            {
                last_navigation_degraded = runtime_tick.safety_navigation_degraded;
                std::printf("[FC][SAFETY] navigation=%s (position/vertical aid acceptance)\n",
                    last_navigation_degraded ? "DEGRADED" : "AVAILABLE");
            }
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
            gcs_bridge.update_telemetry(
                runtime_tick, gps_valid, last_gps);

            // Send an explicit zero/unarmed frame in safe states. This removes
            // ambiguity at both the simulator and the hardware HIL router.
            (void)send_last_actuator();

            xlog::HydroxResidualPolicyRecord residual_policy_record;
            if (runtime_tick.wrench_augmentor_observed)
            {
                for (int axis = 0; axis < 6; ++axis)
                {
                    residual_policy_record.base_tau[axis] =
                        runtime_tick.base_wrench[axis];
                    residual_policy_record.candidate_tau[axis] =
                        runtime_tick.augmentor_candidate_wrench[axis];
                    residual_policy_record.applied_tau[axis] = tau[axis];
                }
                residual_policy_record.observed = 1;
                residual_policy_record.candidate_valid =
                    runtime_tick.wrench_augmentor_candidate_valid ? 1u : 0u;
                residual_policy_record.active =
                    runtime_tick.wrench_augmentor_active ? 1u : 0u;
                residual_policy_record.mode = static_cast<uint8_t>(
                    cfg.residual_policy_mode);
#ifdef HYDROX_ENABLE_RESIDUAL_RL
                if (residual_augmentor != nullptr)
                {
                    const sitl::UdpResidualPolicyStats *stats =
                        residual_augmentor->policy_stats();
                    if (stats != nullptr)
                    {
                        residual_policy_record.inference_ms =
                            stats->last_inference_ms;
                        residual_policy_record.pinn_ood_feature =
                            stats->last_pinn_ood_feature;
                        residual_policy_record.selector_fraction_x_n[0] =
                            stats->last_selector_fraction_x;
                        residual_policy_record.selector_fraction_x_n[1] =
                            stats->last_selector_fraction_n;
                        residual_policy_record.requests = stats->requests;
                        residual_policy_record.accepted = stats->accepted;
                        residual_policy_record.timeouts = stats->timeouts;
                        residual_policy_record.rejected_packets =
                            stats->rejected_packets;
                        residual_policy_record.transport_errors =
                            stats->transport_errors;
                    }
                }
#endif
            }

            sitl::XLogTickData xlog_tick;
            xlog_tick.state = &control_state;
            xlog_tick.setpoint = &sp;
            xlog_tick.wrench = &tau;
            xlog_tick.actuator = &cmd;
            xlog_tick.residual_policy =
                runtime_tick.wrench_augmentor_observed
                    ? &residual_policy_record
                    : nullptr;
            xlog_tick.navigation = &nav;
            xlog_tick.ekf = &flight_runtime.ekf();
            xlog_tick.tick = tick;
            xlog_tick.gnc_mode = cur_mode;
            xlog_tick.mission_state = static_cast<uint8_t>(mission_state);
            xlog_tick.gps_valid = gps_valid;
            xlog_tick.ekf_initialized = ekf_init;
            xlog_tick.controller_reset = runtime_tick.controller_reset;
            xlog_tick.actuator_authorized = runtime_tick.actuator_authorized;
            xlog_tick.used_truth = runtime_tick.used_truth;
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
                fs.actuator_channel_count = static_cast<uint8_t>(
                    std::min<size_t>(vehicle_bundle.logical_actuator_count, norm.size()));
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
                dds_sample.armed = runtime_tick.armed;
                dds_sample.actuator_authorized = runtime_tick.actuator_authorized;
                dds_sample.passive_sonar = nav.passive_sonar;
                dds_sample.acoustic_neighbors = nav.acoustic_neighbors;
                dds_sample.rangefinder_scan = nav.rangefinder_scan;
                (void)dds_worker.try_submit_telemetry(std::move(dds_sample));
            }
#endif

        }

        // ── Disconnect Handling ──────────────────────────────────────────────────────
        (void)hil_session.on_disconnected(monotonic_clock.now_us());
        xlog_recorder.record_safety(flight_runtime.last_tick(), monotonic_clock.now_us(), runtime::RuntimeEvent::DISCONNECTED);
        transport.disconnect();
        if (!parent_guard.is_parent_alive())
        {
            std::printf("[FC] Parent process exited; stopping HydroX SITL.\n");
            break;
        }
        std::printf("[FC] Connection lost, preparing to reconnect...\n");
        gcs_bridge.set_link_connected(false);
        (void)gcs_bridge.send_statustext(
            4, "HydroX SITL disconnected");
    }

    std::printf("[FC] Exited.\n");
    return 0;
}
