#include "hydrox/runtime/hil_session_driver.h"

namespace hydrox::runtime
{
    HilSessionDriver::HilSessionDriver(
        MavlinkHIL &codec,
        HilRuntime &runtime,
        const SensorAdapter::Params &sensor_params)
        : codec_(codec), runtime_(runtime), sensor_adapter_(sensor_params)
    {
    }

    void HilSessionDriver::reset_link_state()
    {
        sensor_adapter_.reset();
        prepared_input_ = NavigationInput{};
        last_sensor_time_us_ = 0;
        last_sensor_arrival_us_ = 0;
        prepared_sensor_ = false;
        simulator_paused_ = false;
    }

    HilSessionResult HilSessionDriver::on_connected(
        platform::MonotonicTimeUs now_us)
    {
        reset_link_state();
        HilSessionResult result;
        result.generation = runtime_.on_connected(now_us);
        result.egress_reset_required = true;
        return result;
    }

    HilSessionResult HilSessionDriver::on_disconnected(
        platform::MonotonicTimeUs now_us)
    {
        HilSessionResult result;
        result.runtime_event = runtime_.on_disconnected(now_us);
        result.generation = runtime_.control_session().generation();
        result.actuator_frame_required = true;
        result.egress_reset_required = true;
        reset_link_state();
        return result;
    }

    HilSessionResult HilSessionDriver::ingest_frame(
        const MavFrame &frame,
        platform::MonotonicTimeUs now_us)
    {
        HilSessionResult result;
        result.generation = runtime_.control_session().generation();

        if (frame.msg_id == MSGID_HEARTBEAT)
        {
            const HeartbeatMsg heartbeat = codec_.parse_heartbeat(frame);
            if (heartbeat.valid)
            {
                const bool paused =
                    heartbeat.system_status == MAV_STATE_STANDBY;
                if (paused != simulator_paused_)
                {
                    result.pause_changed = true;
                    result.simulator_paused = paused;
                    if (paused)
                    {
                        result.runtime_event =
                            runtime_.on_simulator_paused(now_us);
                        result.actuator_frame_required = true;
                        prepared_sensor_ = false;
                    }
                    else
                    {
                        result.generation =
                            runtime_.on_simulator_resumed(now_us);
                    }
                    simulator_paused_ = paused;
                }
            }
        }

        sensor_adapter_.ingest_frame(frame, codec_);
        if (frame.msg_id != MSGID_HIL_SENSOR)
            return result;

        result.sensor_input = sensor_adapter_.build();
        sensor_adapter_.begin_cycle();
        result.has_sensor_input = result.sensor_input.got_imu &&
                                  result.sensor_input.imu.time_usec > 0 &&
                                  !simulator_paused_;
        return result;
    }

    HilSessionResult HilSessionDriver::prepare_sensor(
        const NavigationInput &input,
        platform::MonotonicTimeUs now_us)
    {
        HilSessionResult result;
        result.generation = runtime_.control_session().generation();
        if (prepared_sensor_)
        {
            result.sensor_rejected = true;
            result.step_status = StepStatus::CONFIGURATION_ERROR;
            return result;
        }
        if (!input.got_imu || input.imu.time_usec == 0 || simulator_paused_)
        {
            result.sensor_rejected = true;
            result.step_status = StepStatus::NO_SENSOR;
            return result;
        }

        if (last_sensor_time_us_ != 0 &&
            input.imu.time_usec <= last_sensor_time_us_)
        {
            const platform::MonotonicTimeUs timeout_us =
                runtime_.config().sensor_timeout_us;
            const bool prior_epoch_is_stale =
                timeout_us > 0 && last_sensor_arrival_us_ > 0 &&
                now_us > last_sensor_arrival_us_ &&
                now_us - last_sensor_arrival_us_ > timeout_us;
            if (!prior_epoch_is_stale)
            {
                result.sensor_rejected = true;
                result.step_status = StepStatus::NON_INCREASING_SENSOR_TIME;
                return result;
            }

            (void)runtime_.on_disconnected(now_us);
            result.generation = runtime_.on_connected(now_us);
            sensor_adapter_.reset();
            last_sensor_time_us_ = 0;
            last_sensor_arrival_us_ = 0;
            result.sensor_epoch_restarted = true;
            result.egress_reset_required = true;
        }

        last_sensor_time_us_ = input.imu.time_usec;
        if (runtime_.observe_valid_sensor(now_us) == RuntimeEvent::SENSOR_READY)
            result.sensor_ready = true;

        prepared_input_ = input;
        prepared_sensor_ = true;
        result.sensor_prepared = true;
        return result;
    }

    HilSessionResult HilSessionDriver::step_sensor(
        platform::MonotonicTimeUs now_us)
    {
        HilSessionResult result;
        result.generation = runtime_.control_session().generation();
        if (!prepared_sensor_)
        {
            result.sensor_rejected = true;
            result.step_status = StepStatus::NO_SENSOR;
            return result;
        }

        result.step_status = runtime_.step(prepared_input_, now_us);
        prepared_sensor_ = false;
        if (result.step_status == StepStatus::OK)
            last_sensor_arrival_us_ = now_us;
        if (result.step_status == StepStatus::OK ||
            result.step_status == StepStatus::SENSOR_TIME_GAP)
        {
            result.actuator_frame_required = true;
        }
        return result;
    }

    HilSessionResult HilSessionDriver::process_sensor(
        const NavigationInput &input,
        platform::MonotonicTimeUs now_us)
    {
        HilSessionResult prepared = prepare_sensor(input, now_us);
        if (!prepared.sensor_prepared)
            return prepared;

        HilSessionResult stepped = step_sensor(now_us);
        stepped.sensor_ready = prepared.sensor_ready;
        stepped.sensor_epoch_restarted = prepared.sensor_epoch_restarted;
        stepped.egress_reset_required = prepared.egress_reset_required;
        stepped.generation = prepared.generation;
        return stepped;
    }

    HilSessionResult HilSessionDriver::maintain(
        platform::MonotonicTimeUs now_us)
    {
        HilSessionResult result;
        result.runtime_event = runtime_.maintain(now_us);
        result.generation = runtime_.control_session().generation();
        result.actuator_frame_required =
            result.runtime_event != RuntimeEvent::NONE;
        return result;
    }

    HilSessionResult HilSessionDriver::revoke_setpoint(
        platform::MonotonicTimeUs now_us)
    {
        HilSessionResult result;
        result.runtime_event = runtime_.revoke_setpoint(now_us);
        result.generation = runtime_.control_session().generation();
        result.actuator_frame_required = true;
        return result;
    }

    bool HilSessionDriver::accept_setpoint(
        const GNCSetpoint &setpoint,
        GNCMode mode,
        platform::MonotonicTimeUs received_at_us)
    {
        return runtime_.accept_setpoint(setpoint, mode, received_at_us);
    }

    NavigationInput HilSessionDriver::navigation_snapshot() const
    {
        return sensor_adapter_.build();
    }
} // namespace hydrox::runtime
