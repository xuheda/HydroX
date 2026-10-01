#include "hydrox/runtime/hitl_supervisor.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <utility>

namespace hydrox::runtime
{
    namespace
    {
        constexpr uint8_t kUploaderSystemId = 255;
        constexpr uint8_t kUploaderComponentId = 0;
        constexpr uint8_t kAutopilotSystemId = 1;
        constexpr platform::MonotonicTimeUs kUploaderPairWindowUs = 1'000'000;
    }

    bool validate_hitl_vehicle_profile(
        const HitlVehicleProfile &profile,
        const char *&error) noexcept
    {
        error = nullptr;
        const void *profile_terminator = std::memchr(
            profile.profile_id.data(), '\0', profile.profile_id.size());
        if (profile.profile_id[0] == '\0' || profile_terminator == nullptr ||
            profile.bundle_fingerprint == 0 || !profile.control.valid)
        {
            error = "missing profile identity, fingerprint, or valid flag";
            return false;
        }
        const auto profile_length = static_cast<std::size_t>(
            static_cast<const char *>(profile_terminator) -
            profile.profile_id.data());
        if (profile_length >= HIL_PROFILE_ID_LEN)
        {
            error = "profile identity exceeds HIL handshake capacity";
            return false;
        }
        if (profile.runtime.estimation_profile.vehicle_class !=
                profile.control.vehicle_class ||
            profile.sensors.estimation_profile.vehicle_class !=
                profile.control.vehicle_class)
        {
            error = "estimator/sensor/control vehicle classes disagree";
            return false;
        }
        if (!profile.runtime.safety_profile.valid() ||
            profile.runtime.safety_profile.vehicle_class != profile.control.vehicle_class ||
            profile.runtime.safety_profile.vehicle_kind !=
                safety::safety_profile_for(profile.control).vehicle_kind)
        {
            error = "safety profile does not match the control bundle";
            return false;
        }
        if (!vehicle_class_matches_archetype(
                profile.control.vehicle_class,
                profile.control.archetype))
        {
            error = "vehicle class does not match control archetype";
            return false;
        }
        if (!std::isfinite(profile.runtime.nominal_dt_s) ||
            !(profile.runtime.nominal_dt_s > 0.0) ||
            profile.runtime.nominal_dt_s > 0.1 ||
            !std::isfinite(profile.runtime.max_sensor_dt_s) ||
            profile.runtime.max_sensor_dt_s < profile.runtime.nominal_dt_s ||
            profile.runtime.max_sensor_dt_s > 1.0)
        {
            error = "invalid control period or sensor time-gap bound";
            return false;
        }
        if (profile.runtime.sensor_timeout_us == 0 ||
            profile.runtime.safety_profile.external_command_loss_us == 0)
        {
            error = "sensor and command freshness timeouts must be non-zero";
            return false;
        }
        HilSessionField session_field = HilSessionField::None;
        if (!validate_hil_session_config(
                profile.session_config, session_field))
        {
            error = "invalid resolved HIL session configuration";
            return false;
        }
        if (profile.session_configured)
        {
            if (profile.session_nonce == 0 ||
                hil_session_digest_is_zero(profile.session_digest) ||
                !hil_session_digest_equal(
                    profile.session_digest,
                    hil_session_digest(
                        profile.bundle_fingerprint,
                        profile.session_config)))
            {
                error = "active HIL session identity is invalid";
                return false;
            }
        }
        else if (profile.session_nonce != 0 ||
                 !hil_session_digest_is_zero(profile.session_digest))
        {
            error = "inactive HIL session carries an identity";
            return false;
        }
        const MotorModel::Params &motor = profile.control.motor;
        if (!std::isfinite(motor.tau_m) || !(motor.tau_m > 0.0) ||
            !std::isfinite(motor.rpm_max) || motor.rpm_max < 0.0)
        {
            error = "invalid motor time constant or speed bound";
            return false;
        }

        const ControlParameters &control = profile.control;
        switch (control.archetype)
        {
        case VehicleArchetype::SlenderBodyFin:
            if (!(control.allocator.D_prop > 0.0) ||
                !(control.allocator.n_max_rpm > 0.0) ||
                !(control.allocator.max_thrust_N > 0.0))
            {
                error = "fin vehicle has invalid propeller authority";
                return false;
            }
            break;
        case VehicleArchetype::Thruster:
            if (!(control.max_thrust_per_thruster_N > 0.0))
            {
                error = "thruster vehicle has no actuator authority";
                return false;
            }
            break;
        case VehicleArchetype::Surface:
            if (!(control.surface_channel_surge_limit_N > 0.0) &&
                !(control.max_thrust_per_thruster_N > 0.0))
            {
                error = "surface vehicle has no channel authority";
                return false;
            }
            break;
        case VehicleArchetype::Multirotor:
        case VehicleArchetype::VTOL:
            if (!(control.mass_total > 0.0) &&
                !(control.max_total_lift_N > 0.0))
            {
                error = "lift vehicle has no mass or lift authority";
                return false;
            }
            break;
        case VehicleArchetype::DifferentialDrive:
        {
            const auto &ground = control.ground_allocator;
            if (!(control.mass_total > 0.0) ||
                !(ground.wheel_radius_m > 0.0) ||
                !(ground.track_width_m > 0.0) ||
                !(ground.max_wheel_angular_speed_radps > 0.0) ||
                !(ground.longitudinal_speed_gain_N_per_mps > 0.0))
            {
                error = "differential-drive vehicle has invalid authority";
                return false;
            }
            break;
        }
        case VehicleArchetype::FixedWing:
            break;
        }
        return true;
    }

    HitlSupervisor::HitlSupervisor(
        HitlBoard &board,
        const HitlVehicleProfile &profile,
        ControlStack control_stack)
        : board_(board),
          codec_(1, 1),
          runtime_(profile.runtime,
                   std::move(control_stack.controller),
                   std::move(control_stack.allocator)),
          session_(codec_, runtime_, profile.sensors),
          profile_id_(profile.profile_id),
          profile_fingerprint_(profile.bundle_fingerprint),
          selection_nonce_(profile.selection_nonce),
          mav_type_(profile.mav_type),
          session_config_(profile.session_config),
          session_digest_(profile.session_digest),
          session_nonce_(profile.session_nonce),
          session_configured_(profile.session_configured)
    {
        switch (profile.control.archetype)
        {
        case VehicleArchetype::DifferentialDrive:
        case VehicleArchetype::Surface:
            actuator_channel_count_ = 2;
            break;
        case VehicleArchetype::Multirotor:
        case VehicleArchetype::FixedWing:
            actuator_channel_count_ = 4;
            break;
        case VehicleArchetype::SlenderBodyFin:
            actuator_channel_count_ = 5;
            break;
        case VehicleArchetype::Thruster:
            actuator_channel_count_ = profile.control.direct_body_wrench
                ? 6 : static_cast<uint8_t>(profile.control.thrusters.size());
            break;
        case VehicleArchetype::VTOL:
            actuator_channel_count_ = 8;
            break;
        }
    }

    void HitlSupervisor::visit_frame(void *context, const MavFrame &frame)
    {
        static_cast<HitlSupervisor *>(context)->on_frame(frame);
    }

    void HitlSupervisor::handle_bootloader_reboot_command(
        const MavFrame &frame,
        platform::MonotonicTimeUs now_us)
    {
        const CommandLongMsg request = codec_.parse_command_long(frame);
        const auto reset_pair = [this]()
        {
            bootloader_pair_started_us_ = 0;
            bootloader_broadcast_seen_ = false;
        };

        // Match the two fixed COMMAND_LONG variants sent by the official PX4
        // uploader. A lone generic reboot command is deliberately insufficient.
        bool exact_request = request.valid && frame.mavlink_version == 1 &&
            frame.sysid == kUploaderSystemId &&
            frame.compid == kUploaderComponentId &&
            request.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN &&
            request.params[0] == 3.0F && request.confirmation == 0 &&
            request.target_component == 0;
        for (std::size_t index = 1;
             exact_request && index < request.params.size(); ++index)
        {
            exact_request = request.params[index] == 0.0F;
        }
        if (!exact_request ||
            (request.target_system != 0 &&
             request.target_system != kAutopilotSystemId))
        {
            reset_pair();
            return;
        }

        // This is a local maintenance path, not runtime command authority.
        // It is available only before Session activation and while the
        // independent command link has no live peer.
        if (!board_.bootloader_reboot_supported() ||
            !board_.physical_actuators_inhibited() ||
            session_configured_ || board_.command_link_connected())
        {
            reset_pair();
            board_.notify(
                HitlHealthEvent::COMMAND_REJECTED,
                "bootloader reboot denied outside maintenance window");
            return;
        }

        if (request.target_system == 0)
        {
            bootloader_pair_started_us_ = now_us;
            bootloader_broadcast_seen_ = true;
            return;
        }

        const bool pair_is_fresh = bootloader_broadcast_seen_ &&
            now_us >= bootloader_pair_started_us_ &&
            now_us - bootloader_pair_started_us_ <= kUploaderPairWindowUs;
        reset_pair();
        if (!pair_is_fresh)
            return;

        // Revoke every runtime epoch and make the final simulator-visible
        // actuator frame neutral before the entry point commits the reset.
        (void)session_.on_disconnected(now_us);
        sender_.reset();
        if (!send_last_actuator(now_us))
            stream_failed_ = true;
        bootloader_reboot_requested_ = true;
        board_.notify(
            HitlHealthEvent::BOOTLOADER_REBOOT_REQUESTED,
            "official PX4 uploader requested bootloader maintenance");
    }

    bool HitlSupervisor::send_packet(
        const MavlinkPacket &packet,
        platform::MonotonicTimeUs now_us)
    {
        if (packet.empty())
            return false;
        const FixedFrameSendStatus status = sender_.write_frame(
            board_.hil_stream(), packet.data(), packet.size(), now_us);
        if (status == FixedFrameSendStatus::FRAME_DROPPED ||
            status == FixedFrameSendStatus::TAIL_PENDING)
        {
            board_.notify(
                HitlHealthEvent::STREAM_BACKPRESSURE,
                "HIL transmit backpressure");
            return true;
        }
        return status == FixedFrameSendStatus::COMPLETE;
    }

    bool HitlSupervisor::send_last_actuator(
        platform::MonotonicTimeUs now_us)
    {
        MavlinkPacket packet;
        const HilRuntimeTick &tick = runtime_.last_tick();
        if (!codec_.encode_hil_actuator_controls(
                packet,
                tick.actuator.ch,
                tick.sensor_time_us,
                tick.actuator_mode,
                0))
        {
            return false;
        }
        return send_packet(packet, now_us);
    }

    bool HitlSupervisor::send_heartbeat(
        platform::MonotonicTimeUs now_us)
    {
        MavlinkPacket packet;
        return codec_.encode_heartbeat(packet, mav_type_) &&
               send_packet(packet, now_us);
    }

    bool HitlSupervisor::send_fc_state(
        platform::MonotonicTimeUs now_us)
    {
        const HilRuntimeTick &tick = runtime_.last_tick();
        if (!tick.ekf_initialized || tick.tick == 0 || tick.sensor_time_us == 0)
            return true;

        const NavigationInput navigation = session_.navigation_snapshot();
        HilFcStateMsg state;
        state.time_usec = tick.sensor_time_us;
        for (std::size_t index = 0; index < 6; ++index)
        {
            state.eta[index] = tick.control_state.eta[static_cast<int>(index)];
            state.nu[index] = tick.control_state.nu[static_cast<int>(index)];
        }
        state.depth_m = tick.control_state.depth_m;
        for (std::size_t index = 0; index < tick.actuator.ch.size(); ++index)
            state.normalized[index] = tick.actuator.ch[index];
        state.dvl_vel[0] = navigation.last_dvl.vx;
        state.dvl_vel[1] = navigation.last_dvl.vy;
        state.dvl_vel[2] = navigation.last_dvl.vz;
        state.acc[0] = navigation.imu.xacc;
        state.acc[1] = navigation.imu.yacc;
        state.acc[2] = navigation.imu.zacc;
        state.gyro[0] = navigation.imu.xgyro;
        state.gyro[1] = navigation.imu.ygyro;
        state.gyro[2] = navigation.imu.zgyro;
        state.gps_lat = navigation.last_gps.lat;
        state.gps_lon = navigation.last_gps.lon;
        state.gps_alt = navigation.last_gps.alt;
        state.gps_vel_ned[0] = static_cast<float>(navigation.last_gps.vn) * 0.01f;
        state.gps_vel_ned[1] = static_cast<float>(navigation.last_gps.ve) * 0.01f;
        state.gps_vel_ned[2] = static_cast<float>(navigation.last_gps.vd) * 0.01f;
        state.motor_rpm_actual = static_cast<float>(tick.motor.rpm_actual);
        state.motor_thrust_N = static_cast<float>(tick.motor.thrust_N);
        state.motor_power_W = static_cast<float>(tick.motor.power_W);
        state.motor_current_A = static_cast<float>(tick.motor.current_A);
        state.power_total_W = static_cast<float>(tick.energy.power_total_W);
        state.energy_Wh = static_cast<float>(tick.energy.energy_Wh);
        state.battery_soc = static_cast<float>(tick.energy.soc);
        state.voltage_terminal = static_cast<float>(tick.energy.V_terminal);
        state.runtime_remaining_s = static_cast<float>(
            tick.energy.runtime_rem_s > 0.0 ? tick.energy.runtime_rem_s : 0.0);
        state.commanded_rpm = static_cast<float>(tick.actuator.rpm);
        state.tick = tick.tick;
        state.mode = static_cast<uint8_t>(tick.mode);
        state.mission_state = static_cast<uint8_t>(tick.mission_state);
        state.actuator_channel_count = actuator_channel_count_;
        state.gps_fix = navigation.last_gps.fix_type;
        state.gps_satellites = navigation.last_gps.satellites_visible;
        state.flags = HIL_FC_STATE_FLAG_EKF_INITIALIZED;
        if (tick.control_state.dvl_valid)
            state.flags |= HIL_FC_STATE_FLAG_DVL_VALID;
        if (tick.armed)
            state.flags |= HIL_FC_STATE_FLAG_ARMED;
        if (tick.actuator_authorized)
            state.flags |= HIL_FC_STATE_FLAG_ACTUATOR_AUTHORIZED;
        if (tick.have_external_setpoint)
            state.flags |= HIL_FC_STATE_FLAG_EXTERNAL_SETPOINT;
        state.valid = true;

        MavlinkPacket packet;
        return codec_.encode_hil_fc_state(packet, state) &&
               send_packet(packet, now_us);
    }

    bool HitlSupervisor::send_control_trace(
        platform::MonotonicTimeUs now_us)
    {
        const HilRuntimeTick &tick = runtime_.last_tick();
        if (!tick.ekf_initialized || tick.tick == 0 || tick.sensor_time_us == 0)
            return true;

        const GNCSetpoint &setpoint = runtime_.setpoint();
        HilControlTraceMsg trace;
        trace.time_usec = tick.sensor_time_us;
        trace.tick = tick.tick;
        trace.dt_s = tick.dt;
        for (std::size_t index = 0; index < 6; ++index)
        {
            trace.eta[index] = tick.control_state.eta[static_cast<int>(index)];
            trace.nu[index] = tick.control_state.nu[static_cast<int>(index)];
            trace.wrench[index] = static_cast<float>(
                tick.wrench[static_cast<int>(index)]);
        }
        trace.depth_m = tick.control_state.depth_m;
        trace.depth_ref = setpoint.depth_ref;
        trace.heading_ref = setpoint.heading_ref;
        trace.surge_ref = setpoint.surge_ref;
        trace.yaw_rate_ref = setpoint.yaw_rate_ref;
        trace.wp_n = setpoint.wp_n;
        trace.wp_e = setpoint.wp_e;
        trace.wp_d = setpoint.wp_d;
        trace.path_start_n = setpoint.path_start_n;
        trace.path_start_e = setpoint.path_start_e;
        trace.lookahead_m = setpoint.lookahead_m;
        trace.arrival_radius_m = setpoint.arrival_radius_m;
        trace.setpoint_age_s = static_cast<float>(tick.setpoint_age_s);
        trace.mode = static_cast<uint8_t>(tick.mode);
        trace.mission_state = static_cast<uint8_t>(tick.mission_state);
        trace.actuator_channel_count = actuator_channel_count_;
        if (setpoint.use_yaw_rate_ref)
            trace.flags |= HIL_CONTROL_TRACE_FLAG_USE_YAW_RATE;
        if (setpoint.use_path_segment)
            trace.flags |= HIL_CONTROL_TRACE_FLAG_USE_PATH_SEGMENT;
        if (setpoint.hold_heading)
            trace.flags |= HIL_CONTROL_TRACE_FLAG_HOLD_HEADING;
        if (tick.controller_reset)
            trace.flags |= HIL_CONTROL_TRACE_FLAG_CONTROLLER_RESET;
        if (tick.actuator_authorized)
            trace.flags |= HIL_CONTROL_TRACE_FLAG_ACTUATOR_AUTHORIZED;
        if (tick.used_truth)
            trace.flags |= HIL_CONTROL_TRACE_FLAG_USED_TRUTH;
        if (tick.have_external_setpoint)
            trace.flags |= HIL_CONTROL_TRACE_FLAG_EXTERNAL_SETPOINT;
        if (tick.control_state.dvl_valid)
            trace.flags |= HIL_CONTROL_TRACE_FLAG_DVL_VALID;
        if (tick.ekf_initialized)
            trace.flags |= HIL_CONTROL_TRACE_FLAG_EKF_INITIALIZED;
        trace.valid = true;

        MavlinkPacket packet;
        return codec_.encode_hil_control_trace(packet, trace) &&
               send_packet(packet, now_us);
    }

    bool HitlSupervisor::send_profile_status(
        HilProfileOperation operation,
        const char *profile_id,
        uint64_t fingerprint,
        uint32_t nonce,
        uint8_t mav_type,
        platform::MonotonicTimeUs now_us)
    {
        if (profile_id == nullptr)
            return false;
        HilProfileMsg message;
        message.fingerprint = fingerprint;
        message.nonce = nonce;
        message.operation = operation;
        message.mav_type = mav_type;
        std::size_t length = 0;
        while (length < message.profile_id.size() && profile_id[length] != '\0')
            ++length;
        if (length == 0 || length >= message.profile_id.size())
            return false;
        std::memcpy(message.profile_id.data(), profile_id, length);
        message.profile_id[length] = '\0';
        message.valid = fingerprint != 0 && nonce != 0;
        MavlinkPacket packet;
        return codec_.encode_hil_profile(packet, message) &&
               send_packet(packet, now_us);
    }

    bool HitlSupervisor::send_session_status(
        HilSessionStatusOperation operation,
        const HilSessionDigest &digest,
        uint32_t nonce,
        HilSessionRejectReason reason,
        HilSessionField field,
        platform::MonotonicTimeUs now_us)
    {
        HilSessionStatusMsg message;
        message.profile_fingerprint = profile_fingerprint_;
        message.nonce = nonce;
        message.digest = digest;
        message.operation = operation;
        message.reason = reason;
        message.field = field;
        message.valid = true;
        MavlinkPacket packet;
        return codec_.encode_hil_session_status(packet, message) &&
               send_packet(packet, now_us);
    }

    void HitlSupervisor::on_frame(const MavFrame &frame)
    {
        const platform::MonotonicTimeUs now_us = board_.clock().now_us();
        if (bootloader_reboot_requested_)
            return;
        if (frame.msg_id == MSGID_COMMAND_LONG)
        {
            handle_bootloader_reboot_command(frame, now_us);
            return;
        }
        if (frame.msg_id == MSGID_HIL_PROFILE)
        {
            const HilProfileMsg request = codec_.parse_hil_profile(frame);
            if (!request.valid ||
                request.operation != HilProfileOperation::Select)
                return;

            const bool is_active =
                request.fingerprint == profile_fingerprint_ &&
                std::strcmp(request.profile_id.data(), profile_id_.data()) == 0;
            if (is_active)
            {
                if (!send_profile_status(
                        HilProfileOperation::Ready,
                        profile_id_.data(),
                        profile_fingerprint_,
                        request.nonce,
                        mav_type_,
                        now_us))
                    stream_failed_ = true;
                return;
            }

            if (!board_.request_vehicle_profile(
                    request.profile_id.data(),
                    request.fingerprint,
                    request.nonce))
            {
                if (!send_profile_status(
                        HilProfileOperation::Rejected,
                        request.profile_id.data(),
                        request.fingerprint,
                        request.nonce,
                        request.mav_type,
                        now_us))
                    stream_failed_ = true;
                return;
            }

            // The next profile is staged but never runs in this control
            // object. Revoke the old epoch and make its final observable
            // command neutral before the entry point rebuilds every stateful
            // estimator/controller/allocator object.
            const HilSessionResult disconnected =
                session_.on_disconnected(now_us);
            sender_.reset();
            if (disconnected.actuator_frame_required &&
                !send_last_actuator(now_us))
                stream_failed_ = true;
            profile_switch_requested_ = true;
            board_.notify(HitlHealthEvent::HIL_DISCONNECTED,
                          "vehicle profile switch requested");
            return;
        }

        if (frame.msg_id == MSGID_HIL_SESSION_CONFIG)
        {
            const HilSessionConfigMsg request =
                codec_.parse_hil_session_config(frame);
            if (!request.valid)
                return;

            const auto reject =
                [&](HilSessionRejectReason reason, HilSessionField field)
                {
                    if (!send_session_status(
                            HilSessionStatusOperation::Rejected,
                            request.digest,
                            request.nonce,
                            reason,
                            field,
                            now_us))
                        stream_failed_ = true;
                };

            if (request.profile_fingerprint != profile_fingerprint_)
            {
                reject(HilSessionRejectReason::ProfileMismatch,
                       HilSessionField::None);
                return;
            }
            if (!request.digest_valid)
            {
                reject(HilSessionRejectReason::HashMismatch,
                       HilSessionField::None);
                return;
            }
            HilSessionField validation_field = HilSessionField::None;
            if (!validate_hil_session_config(
                    request.config, validation_field))
            {
                reject(
                    request.config.schema_version !=
                            kHilSessionConfigVersion
                        ? HilSessionRejectReason::UnsupportedVersion
                        : HilSessionRejectReason::InvalidRange,
                    validation_field);
                return;
            }

            if (session_configured_ &&
                hil_session_digest_equal(
                    request.digest, session_digest_))
            {
                if (!send_session_status(
                        HilSessionStatusOperation::Ready,
                        session_digest_,
                        request.nonce,
                        HilSessionRejectReason::None,
                        HilSessionField::None,
                        now_us))
                    stream_failed_ = true;
                return;
            }

            const HitlSessionRequestResult staged =
                board_.request_session_config(
                    request.profile_fingerprint,
                    request.config,
                    request.digest,
                    request.nonce);
            if (!staged.accepted())
            {
                reject(staged.reason, staged.field);
                return;
            }

            const HilSessionResult disconnected =
                session_.on_disconnected(now_us);
            sender_.reset();
            if (disconnected.actuator_frame_required &&
                !send_last_actuator(now_us))
                stream_failed_ = true;
            session_config_restart_requested_ = true;
            board_.notify(
                HitlHealthEvent::HIL_DISCONNECTED,
                "HIL session configuration restart requested");
            return;
        }

        // Profile identity alone never authorizes runtime data. The Router
        // must complete the versioned Session handshake first.
        if (!session_configured_)
            return;

        const HilSessionResult frame_result =
            session_.ingest_frame(frame, now_us);
        if (frame_result.pause_changed && frame_result.simulator_paused)
        {
            board_.notify(HitlHealthEvent::FAILSAFE,
                          "simulator paused");
        }
        if (frame_result.actuator_frame_required &&
            !send_last_actuator(now_us))
        {
            stream_failed_ = true;
        }
        if (!frame_result.has_sensor_input)
            return;

        const HilSessionResult prepared =
            session_.prepare_sensor(frame_result.sensor_input, now_us);
        if (prepared.egress_reset_required)
            sender_.reset();
        if (prepared.sensor_epoch_restarted)
        {
            board_.notify(HitlHealthEvent::HIL_CONNECTED,
                          "HIL sensor timestamp epoch restarted");
        }
        if (prepared.sensor_ready)
        {
            board_.notify(HitlHealthEvent::SENSOR_READY,
                          "HIL sensor epoch ready");
        }
        if (!prepared.sensor_prepared)
        {
            if (prepared.sensor_rejected)
            {
                board_.notify(HitlHealthEvent::FAILSAFE,
                              step_status_name(prepared.step_status));
            }
            return;
        }

        const HilSessionResult stepped = session_.step_sensor(now_us);
        if (stepped.step_status != StepStatus::OK)
        {
            board_.notify(HitlHealthEvent::FAILSAFE,
                          step_status_name(stepped.step_status));
        }
        if (stepped.actuator_frame_required &&
            !send_last_actuator(now_us))
        {
            stream_failed_ = true;
        }
        if (!stream_failed_ && stepped.actuator_frame_required &&
            !send_control_trace(now_us))
        {
            stream_failed_ = true;
        }
        if (!stream_failed_ && now_us >= next_fc_state_us_)
        {
            if (!send_fc_state(now_us))
                stream_failed_ = true;
            next_fc_state_us_ = now_us + 50'000;
        }
    }

    void HitlSupervisor::service_command_link(
        platform::MonotonicTimeUs now_us)
    {
        const bool connected = board_.command_link_connected();
        const uint64_t generation = board_.command_link_generation();
        if (generation != command_generation_ ||
            (command_connected_ && !connected))
        {
            const HilSessionResult revoked =
                session_.revoke_setpoint(now_us);
            board_.notify(HitlHealthEvent::FAILSAFE,
                          "command link epoch changed");
            if (revoked.actuator_frame_required &&
                !send_last_actuator(now_us))
            {
                stream_failed_ = true;
            }
        }
        command_generation_ = generation;
        command_connected_ = connected;
        if (!connected)
            return;

        // Bound command work per supervisor pass so HIL receive cannot starve.
        for (int i = 0; i < 8; ++i)
        {
            HitlSetpointSample sample;
            if (!board_.poll_setpoint(sample))
                break;
            const bool accepted =
                sample.command_link_generation == command_generation_ &&
                sample.received_at_us > 0 &&
                session_.accept_setpoint(
                    sample.setpoint,
                    sample.mode,
                    sample.received_at_us);
            board_.notify(
                accepted ? HitlHealthEvent::COMMAND_ACCEPTED
                         : HitlHealthEvent::COMMAND_REJECTED,
                accepted ? "fresh command accepted"
                         : "stale or pre-sensor command rejected");
        }
    }

    void HitlSupervisor::notify_safety_status_transition()
    {
        const auto &tick = runtime_.last_tick();
        const auto &status = tick.safety_status;
        if (status.transition_sequence == last_safety_transition_sequence_)
            return;
        last_safety_transition_sequence_ = status.transition_sequence;
        char message[384]{};
        std::snprintf(message, sizeof(message),
            "mode=%s action=%s reason=%s source=%s cause=%s observed_s=%.3f threshold_s=%.3f suspended=%u",
            safety::vehicle_mode_name(status.mode), safety::safety_action_name(status.action),
            safety::safety_reason_name(status.reason), safety::control_source_name(tick.safety_control_source),
            tick.safety_cause, tick.safety_observed_s, tick.safety_threshold_s,
            status.session_suspended ? 1u : 0u);
        board_.notify(HitlHealthEvent::SAFETY_TRANSITION, message);
    }

    int HitlSupervisor::run()
    {
        board_.notify(HitlHealthEvent::STARTING,
                      "HydroX HITL supervisor starting");
        if (!board_.physical_actuators_inhibited())
        {
            board_.notify(
                HitlHealthEvent::CONFIGURATION_ERROR,
                "physical actuator outputs are not inhibited");
            return 20;
        }
        if (!runtime_.valid())
        {
            board_.notify(HitlHealthEvent::CONFIGURATION_ERROR,
                          "invalid controller/allocation stack");
            return 21;
        }
        if (!board_.watchdog().start(1000))
        {
            board_.notify(HitlHealthEvent::CONFIGURATION_ERROR,
                          "hardware watchdog failed to start");
            return 22;
        }

        std::array<uint8_t, 512> receive_buffer{};
        while (!board_.should_exit())
        {
            platform::ByteStream &stream = board_.hil_stream();
            if (!stream.open())
            {
                board_.watchdog().kick();
                board_.sleeper().sleep_for_us(100'000);
                continue;
            }

            const platform::MonotonicTimeUs connected_at =
                board_.clock().now_us();
            const HilSessionResult connected =
                session_.on_connected(connected_at);
            if (connected.egress_reset_required)
                sender_.reset();
            stream_failed_ = false;
            profile_switch_requested_ = false;
            session_config_restart_requested_ = false;
            bootloader_pair_started_us_ = 0;
            bootloader_broadcast_seen_ = false;
            bootloader_reboot_requested_ = false;
            command_generation_ = board_.command_link_generation();
            command_connected_ = board_.command_link_connected();
            next_heartbeat_us_ = connected_at;
            next_fc_state_us_ = connected_at;
            last_safety_transition_sequence_ = 0;
            notify_safety_status_transition();
            board_.notify(HitlHealthEvent::HIL_CONNECTED,
                          "HIL byte stream connected");

            if (selection_nonce_ != 0 &&
                !send_profile_status(
                    HilProfileOperation::Ready,
                    profile_id_.data(),
                    profile_fingerprint_,
                    selection_nonce_,
                    mav_type_,
                    connected_at))
            {
                stream_failed_ = true;
            }

            if (session_configured_ && session_nonce_ != 0 &&
                !send_session_status(
                    HilSessionStatusOperation::Ready,
                    session_digest_,
                    session_nonce_,
                    HilSessionRejectReason::None,
                    HilSessionField::None,
                    connected_at))
            {
                stream_failed_ = true;
            }

            while (!board_.should_exit() && stream.is_open() &&
                   !stream_failed_ && !profile_switch_requested_ &&
                   !session_config_restart_requested_ &&
                   !bootloader_reboot_requested_)
            {
                const platform::MonotonicTimeUs now_us =
                    board_.clock().now_us();
                if (session_configured_)
                    service_command_link(now_us);

                if (session_configured_)
                {
                    const HilSessionResult maintenance =
                        session_.maintain(now_us);
                    if (maintenance.runtime_event != RuntimeEvent::NONE)
                    {
                        board_.notify(HitlHealthEvent::FAILSAFE,
                                      runtime_event_name(
                                          maintenance.runtime_event));
                        if (maintenance.actuator_frame_required &&
                            !send_last_actuator(now_us))
                        {
                            stream_failed_ = true;
                            break;
                        }
                    }
                }
                notify_safety_status_transition();

                const FixedFrameSendStatus flush_status =
                    sender_.flush(stream, now_us);
                if (flush_status == FixedFrameSendStatus::FATAL ||
                    flush_status == FixedFrameSendStatus::TIMED_OUT)
                {
                    stream_failed_ = true;
                    break;
                }

                if (now_us >= next_heartbeat_us_)
                {
                    if (!send_heartbeat(now_us))
                    {
                        stream_failed_ = true;
                        break;
                    }
                    next_heartbeat_us_ = now_us + 1'000'000;
                }

                const platform::IoResult read = stream.read(
                    receive_buffer.data(), receive_buffer.size());
                if (read.status == platform::IoStatus::Ok)
                {
                    if (read.size == 0 || read.size > receive_buffer.size())
                    {
                        stream_failed_ = true;
                        break;
                    }
                    codec_.feed_each(
                        receive_buffer.data(),
                        read.size,
                        this,
                        &HitlSupervisor::visit_frame);
                }
                else if (read.status == platform::IoStatus::WouldBlock)
                {
                    board_.sleeper().sleep_for_us(1'000);
                }
                else
                {
                    stream_failed_ = true;
                    break;
                }
                board_.watchdog().kick();
            }

            const platform::MonotonicTimeUs disconnected_at =
                board_.clock().now_us();
            const HilSessionResult disconnected =
                session_.on_disconnected(disconnected_at);
            notify_safety_status_transition();
            if (stream.is_open() && !stream_failed_ &&
                disconnected.actuator_frame_required)
                (void)send_last_actuator(disconnected_at);
            stream.close();
            board_.notify(HitlHealthEvent::HIL_DISCONNECTED,
                          "HIL byte stream disconnected");
            board_.watchdog().kick();
            if (profile_switch_requested_)
                return kHitlProfileSwitchRequested;
            if (session_config_restart_requested_)
                return kHitlSessionConfigRestartRequested;
            if (bootloader_reboot_requested_)
                return kHitlBootloaderRestartRequested;
            if (!board_.should_exit())
                board_.sleeper().sleep_for_us(100'000);
        }
        return 0;
    }
} // namespace hydrox::runtime
