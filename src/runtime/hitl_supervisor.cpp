#include "hydrox/runtime/hitl_supervisor.h"

#include <array>
#include <cmath>
#include <utility>

namespace hydrox::runtime
{
    bool validate_hitl_vehicle_profile(
        const HitlVehicleProfile &profile,
        const char *&error) noexcept
    {
        error = nullptr;
        if (profile.profile_id[0] == '\0' ||
            profile.bundle_fingerprint == 0 ||
            !profile.control.valid)
        {
            error = "missing profile identity, fingerprint, or valid flag";
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
            profile.runtime.setpoint_timeout_us == 0)
        {
            error = "sensor and command freshness timeouts must be non-zero";
            return false;
        }
        const MotorModel::Params &motor = profile.control.motor;
        if (!std::isfinite(motor.tau_m) || !(motor.tau_m > 0.0) ||
            !std::isfinite(motor.rpm_max) || motor.rpm_max < 0.0)
        {
            error = "invalid motor time constant or speed bound";
            return false;
        }

        const FossenControlParams &control = profile.control;
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
          sensor_adapter_(profile.sensors),
          runtime_(profile.runtime,
                   std::move(control_stack.controller),
                   std::move(control_stack.allocator)),
          mav_type_(profile.mav_type)
    {
    }

    void HitlSupervisor::visit_frame(void *context, const MavFrame &frame)
    {
        static_cast<HitlSupervisor *>(context)->on_frame(frame);
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

    void HitlSupervisor::on_frame(const MavFrame &frame)
    {
        const platform::MonotonicTimeUs now_us = board_.clock().now_us();
        if (frame.msg_id == MSGID_HEARTBEAT)
        {
            const HeartbeatMsg heartbeat = codec_.parse_heartbeat(frame);
            if (heartbeat.valid)
            {
                const bool paused =
                    heartbeat.system_status == MAV_STATE_STANDBY;
                if (paused && !simulator_paused_)
                {
                    (void)runtime_.revoke_setpoint(now_us);
                    if (!send_last_actuator(now_us))
                        stream_failed_ = true;
                    board_.notify(
                        HitlHealthEvent::FAILSAFE,
                        "simulator paused");
                }
                simulator_paused_ = paused;
            }
        }

        sensor_adapter_.ingest_frame(frame, codec_);
        if (frame.msg_id != MSGID_HIL_SENSOR)
            return;

        NavigationInput input = sensor_adapter_.build();
        sensor_adapter_.begin_cycle();
        if (!input.got_imu || input.imu.time_usec == 0 || simulator_paused_)
            return;
        if (input.imu.time_usec <= last_sensor_time_us_)
        {
            board_.notify(
                HitlHealthEvent::FAILSAFE,
                "non-increasing HIL sensor timestamp");
            return;
        }
        last_sensor_time_us_ = input.imu.time_usec;

        if (runtime_.observe_valid_sensor(now_us) ==
            RuntimeEvent::SENSOR_READY)
        {
            board_.notify(HitlHealthEvent::SENSOR_READY,
                          "HIL sensor epoch ready");
        }

        const StepStatus status = runtime_.step(input, now_us);
        if (status != StepStatus::OK)
        {
            board_.notify(HitlHealthEvent::FAILSAFE,
                          step_status_name(status));
            if (status == StepStatus::SENSOR_TIME_GAP &&
                !send_last_actuator(now_us))
            {
                stream_failed_ = true;
            }
            return;
        }

        if (!send_last_actuator(now_us))
            stream_failed_ = true;
    }

    void HitlSupervisor::service_command_link(
        platform::MonotonicTimeUs now_us)
    {
        const bool connected = board_.command_link_connected();
        const uint64_t generation = board_.command_link_generation();
        if (generation != command_generation_ ||
            (command_connected_ && !connected))
        {
            (void)runtime_.revoke_setpoint(now_us);
            board_.notify(HitlHealthEvent::FAILSAFE,
                          "command link epoch changed");
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
                runtime_.accept_setpoint(
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
            (void)runtime_.on_connected(connected_at);
            sensor_adapter_.reset();
            sender_.reset();
            last_sensor_time_us_ = 0;
            simulator_paused_ = false;
            stream_failed_ = false;
            command_generation_ = board_.command_link_generation();
            command_connected_ = board_.command_link_connected();
            next_heartbeat_us_ = connected_at;
            board_.notify(HitlHealthEvent::HIL_CONNECTED,
                          "HIL byte stream connected");

            while (!board_.should_exit() && stream.is_open() &&
                   !stream_failed_)
            {
                const platform::MonotonicTimeUs now_us =
                    board_.clock().now_us();
                service_command_link(now_us);

                const RuntimeEvent event = runtime_.maintain(now_us);
                if (event != RuntimeEvent::NONE)
                {
                    board_.notify(HitlHealthEvent::FAILSAFE,
                                  runtime_event_name(event));
                    if (!send_last_actuator(now_us))
                    {
                        stream_failed_ = true;
                        break;
                    }
                }

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
            (void)runtime_.on_disconnected(disconnected_at);
            if (stream.is_open() && !stream_failed_)
                (void)send_last_actuator(disconnected_at);
            stream.close();
            board_.notify(HitlHealthEvent::HIL_DISCONNECTED,
                          "HIL byte stream disconnected");
            board_.watchdog().kick();
            if (!board_.should_exit())
                board_.sleeper().sleep_for_us(100'000);
        }
        return 0;
    }
} // namespace hydrox::runtime
