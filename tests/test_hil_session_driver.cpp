#include "hydrox/runtime/hil_session_driver.h"

#include <cmath>
#include <cstdio>
#include <memory>

namespace
{
    int fail(const char *message)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        return 1;
    }

    class ConstantController final : public hydrox::IController
    {
    public:
        void reset(const hydrox::NavigationState &) override {}
        void set_mode(hydrox::GNCMode) override {}
        void set_setpoint(const hydrox::GNCSetpoint &) override {}
        hydrox::Wrench update(const hydrox::NavigationState &, double) override
        {
            hydrox::Wrench wrench = hydrox::Wrench::Zero();
            wrench[0] = 10.0;
            return wrench;
        }
    };

    class ConstantAllocator final : public hydrox::IAllocator
    {
    public:
        hydrox::ActuatorCmd allocate(
            const hydrox::Wrench &wrench, double) const override
        {
            hydrox::ActuatorCmd command;
            command.layout = hydrox::ActuatorLayout::DirectBodyWrench;
            command.set_body_force(0, wrench[0], 20.0);
            command.set_body_force(1, 0.0, 20.0);
            command.set_body_force(2, 0.0, 20.0);
            command.set_body_moment(3, 0.0, 20.0);
            command.set_body_moment(4, 0.0, 20.0);
            command.set_body_moment(5, 0.0, 20.0);
            return command;
        }
    };

    hydrox::NavigationInput sensor(uint64_t timestamp_us)
    {
        hydrox::NavigationInput input;
        input.got_imu = true;
        input.imu.time_usec = timestamp_us;
        input.measurements.depth.meta.valid = true;
        input.measurements.depth.meta.source = hydrox::NavMeasurementSource::Depth;
        input.measurements.depth.variance = 0.01;
        return input;
    }

    hydrox::MavFrame heartbeat(uint8_t system_status)
    {
        hydrox::MavFrame frame;
        frame.msg_id = hydrox::MSGID_HEARTBEAT;
        frame.payload.resize(9, 0);
        frame.payload[4] = 12;
        frame.payload[7] = system_status;
        frame.payload[8] = 3;
        return frame;
    }

    bool safe_tick(const hydrox::runtime::HilRuntime &runtime)
    {
        const hydrox::runtime::HilRuntimeTick &tick = runtime.last_tick();
        return !tick.actuator_authorized &&
               (tick.actuator_mode &
                hydrox::runtime::kMavModeFlagSafetyArmed) == 0 &&
               std::abs(tick.actuator.ch[0]) < 1e-6f;
    }
}

int main()
{
    using namespace hydrox;
    using namespace hydrox::runtime;

    HilRuntimeConfig config;
    config.nominal_dt_s = 0.01;
    config.max_sensor_dt_s = 0.25;
    config.safety_profile.external_command_loss_us = 200'000;
    config.safety_profile.external_command_warn_us = 100'000;
    config.safety_profile.command_hold_max_us = 100'000;
    config.sensor_timeout_us = 100'000;
    HilRuntime runtime(
        config,
        std::make_unique<ConstantController>(),
        std::make_unique<ConstantAllocator>());
    MavlinkHIL codec;
    HilSessionDriver session(codec, runtime, SensorAdapter::Params{});

    constexpr uint64_t start = 10'000'000;
    const HilSessionResult connected = session.on_connected(start);
    if (connected.generation != 1 || !connected.egress_reset_required)
        return fail("connect did not create and reset a shared HIL epoch");

    GNCSetpoint setpoint;
    setpoint.surge_ref = 1.0;
    if (session.accept_setpoint(setpoint, GNCMode::DEPTH_HOLD, start + 1))
        return fail("command crossed the first-sensor gate");

    HilSessionResult prepared = session.prepare_sensor(
        sensor(1'000'000), start + 10'000);
    if (!prepared.sensor_prepared || !prepared.sensor_ready)
        return fail("first sensor did not open the shared command gate");
    const uint64_t command_time = start + 11'000;
    if (!session.accept_setpoint(
            setpoint, GNCMode::DEPTH_HOLD, command_time))
        return fail("fresh command was rejected after sensor preparation");
    HilSessionResult stepped = session.step_sensor(start + 20'000);
    if (stepped.step_status != StepStatus::OK ||
        !stepped.actuator_frame_required ||
        !runtime.last_tick().actuator_authorized)
    {
        return fail("normal sensor tick did not require an active actuator frame");
    }

    const HilSessionResult timed_out = session.maintain(
        command_time + config.safety_profile.external_command_loss_us + 1);
    if (timed_out.runtime_event != RuntimeEvent::SENSOR_TIMEOUT ||
        !timed_out.actuator_frame_required || !safe_tick(runtime))
    {
        return fail("sensor timeout did not require an immediate inhibited frame");
    }

    session.on_disconnected(start + 1'000'000);
    session.on_connected(start + 1'010'000);
    prepared = session.prepare_sensor(sensor(2'000'000), start + 1'020'000);
    if (!prepared.sensor_prepared ||
        !session.accept_setpoint(
            setpoint, GNCMode::DEPTH_HOLD, start + 1'021'000) ||
        session.step_sensor(start + 1'030'000).step_status != StepStatus::OK)
    {
        return fail("second epoch did not become active");
    }
    prepared = session.prepare_sensor(sensor(3'000'000), start + 1'040'000);
    stepped = session.step_sensor(start + 1'041'000);
    if (!prepared.sensor_prepared ||
        stepped.step_status != StepStatus::SENSOR_TIME_GAP ||
        !stepped.actuator_frame_required || !safe_tick(runtime))
    {
        return fail("sensor time gap did not require an immediate safe frame");
    }

    const uint64_t stale_arrival = start + 1'030'000;
    prepared = session.prepare_sensor(
        sensor(1'000), stale_arrival + config.sensor_timeout_us + 1);
    if (!prepared.sensor_prepared || !prepared.sensor_epoch_restarted ||
        !prepared.egress_reset_required || prepared.generation != 3)
    {
        return fail("stale sensor timestamp epoch was not restarted uniformly");
    }
    if (!session.accept_setpoint(
            setpoint, GNCMode::DEPTH_HOLD,
            stale_arrival + config.sensor_timeout_us + 2) ||
        session.step_sensor(
            stale_arrival + config.sensor_timeout_us + 3).step_status !=
            StepStatus::OK)
    {
        return fail("restarted sensor epoch did not reopen cleanly");
    }

    const HilSessionResult paused = session.ingest_frame(
        heartbeat(MAV_STATE_STANDBY), start + 2'000'000);
    if (!paused.pause_changed || !paused.simulator_paused ||
        paused.runtime_event != RuntimeEvent::SIMULATOR_PAUSED ||
        !paused.actuator_frame_required || !safe_tick(runtime))
    {
        return fail("pause did not immediately publish the shared safe state");
    }
    const HilSessionResult resumed = session.ingest_frame(
        heartbeat(MAV_STATE_ACTIVE), start + 2'010'000);
    if (!resumed.pause_changed || resumed.simulator_paused ||
        resumed.generation != 4 ||
        session.accept_setpoint(
            setpoint, GNCMode::DEPTH_HOLD, start + 2'011'000))
    {
        return fail("resume did not create a fresh sensor-gated epoch");
    }

    prepared = session.prepare_sensor(sensor(2'000), start + 2'020'000);
    if (!prepared.sensor_prepared ||
        !session.accept_setpoint(
            setpoint, GNCMode::DEPTH_HOLD, start + 2'021'000) ||
        session.step_sensor(start + 2'030'000).step_status != StepStatus::OK)
    {
        return fail("post-resume epoch did not become active");
    }
    if (!safe_tick(runtime) || !runtime.safety_status().operator_ack_required)
        return fail("session restart or pause/resume cleared a hard fault");
    const HilSessionResult revoked = session.revoke_setpoint(start + 2'040'000);
    if (!revoked.actuator_frame_required || !safe_tick(runtime))
        return fail("command-link revoke did not require an immediate safe frame");

    std::puts("PASS: shared HIL session driver");
    return 0;
}
