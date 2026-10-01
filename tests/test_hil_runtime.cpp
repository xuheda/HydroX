#include "hydrox/runtime/hil_runtime.h"

#include <cmath>
#include <cstdio>
#include <limits>
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
        void reset(const hydrox::NavigationState &) override { ++reset_count; }
        void set_mode(hydrox::GNCMode next) override { mode = next; }
        void set_setpoint(const hydrox::GNCSetpoint &next) override
        {
            setpoint = next;
        }
        hydrox::Wrench update(const hydrox::NavigationState &, double) override
        {
            ++update_count;
            hydrox::Wrench wrench = hydrox::Wrench::Zero();
            wrench[0] = 10.0;
            return wrench;
        }

        int reset_count = 0;
        int update_count = 0;
        hydrox::GNCMode mode = hydrox::GNCMode::DISABLED;
        hydrox::GNCSetpoint setpoint{};
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
            command.rpm = 100.0;
            return command;
        }
    };
    class InvalidAllocator final : public hydrox::IAllocator
    {
    public:
        hydrox::ActuatorCmd allocate(
            const hydrox::Wrench &, double) const override
        {
            hydrox::ActuatorCmd command;
            // Simulates a buggy allocator that bypasses the physical-unit
            // setters and provides no layout or quantity metadata.
            command.ch[0] = 0.5f;
            return command;
        }
    };

    class OffsetAugmentor final : public hydrox::runtime::IWrenchAugmentor
    {
    public:
        explicit OffsetAugmentor(bool invalid = false) : invalid_(invalid) {}

        void reset() override { ++reset_count; }

        hydrox::Wrench update(
            const hydrox::NavigationState &,
            const hydrox::GNCSetpoint &,
            const hydrox::Wrench &base,
            const hydrox::ActuatorCmd &,
            double) override
        {
            ++update_count;
            hydrox::Wrench candidate = base;
            candidate[0] = invalid_
                ? std::numeric_limits<double>::quiet_NaN()
                : base[0] + 5.0;
            return candidate;
        }

        int reset_count = 0;
        int update_count = 0;

    private:
        bool invalid_ = false;
    };


    hydrox::NavigationInput sensor(uint64_t timestamp_us)
    {
        hydrox::NavigationInput input;
        input.got_imu = true;
        input.imu.time_usec = timestamp_us;
        input.measurements.depth.meta.valid = true;
        input.measurements.depth.meta.age_s = 0.0;
        input.measurements.depth.meta.timestamp_s = timestamp_us * 1.e-6;
        input.measurements.depth.meta.source = hydrox::NavMeasurementSource::Depth;
        input.measurements.depth.variance = 0.01;
        return input;
    }
}

int main()
{
    using namespace hydrox;
    using namespace hydrox::runtime;

    auto controller = std::make_unique<ConstantController>();
    ConstantController *controller_probe = controller.get();
    auto allocator = std::make_unique<ConstantAllocator>();

    HilRuntimeConfig config;
    config.nominal_dt_s = 0.01;
    config.max_sensor_dt_s = 0.25;
    config.safety_profile.external_command_loss_us = 2'000'000;
    config.sensor_timeout_us = 500'000;
    HilRuntime runtime(
        config, std::move(controller), std::move(allocator));
    if (!runtime.valid())
        return fail("valid controller stack was rejected");

    constexpr uint64_t start = 10'000'000;
    if (runtime.on_connected(start) != 1)
        return fail("connection generation did not start at one");
    if (runtime.safety_status().mode !=
            hydrox::safety::VehicleMode::ArmedIdle ||
        !runtime.safety_status().armed)
    {
        return fail("safety supervisor did not mirror initial armed-idle state");
    }

    GNCSetpoint setpoint;
    setpoint.surge_ref = 1.0;
    if (runtime.accept_setpoint(setpoint, GNCMode::DEPTH_HOLD, start + 1))
        return fail("command was accepted before a sensor from this epoch");

    if (runtime.observe_valid_sensor(start + 10'000) !=
        RuntimeEvent::SENSOR_READY)
    {
        return fail("first valid sensor did not open the command gate");
    }
    GNCSetpoint overbound = setpoint;
    overbound.surge_ref = GncSetpointLimits::max_abs_surge_mps + 1.0;
    if (runtime.accept_setpoint(
            overbound, GNCMode::DEPTH_HOLD, start + 11'000))
        return fail("shared runtime accepted an over-bound command");

    const uint64_t command_time = start + 20'000;
    if (!runtime.accept_setpoint(
            setpoint, GNCMode::DEPTH_HOLD, command_time))
    {
        return fail("fresh post-sensor command was rejected");
    }
    if (runtime.safety_status().mode !=
            hydrox::safety::VehicleMode::ExternalControl ||
        !runtime.safety_status().external_authorized)
    {
        return fail("safety supervisor did not authorize fresh external control");
    }

    if (runtime.step(sensor(1'000'000), start + 30'000) != StepStatus::OK)
        return fail("valid first sensor tick was rejected");
    const HilRuntimeTick active = runtime.last_tick();
    if (!active.actuator_authorized ||
        (active.actuator_mode & kMavModeFlagSafetyArmed) == 0 ||
        std::abs(active.actuator.ch[0] - 0.5f) > 1e-6f ||
        controller_probe->update_count != 1)
    {
        return fail("active pipeline did not run controller/allocation and arm");
    }

    if (runtime.maintain(command_time + 2'000'001) !=
        RuntimeEvent::SENSOR_TIMEOUT)
    {
        return fail("sensor loss must take priority over simultaneous command loss");
    }
    const HilRuntimeTick timed_out = runtime.last_tick();
    if (timed_out.actuator_authorized ||
        (timed_out.actuator_mode & kMavModeFlagSafetyArmed) != 0 ||
        timed_out.actuator.ch[0] != 0.0f ||
        runtime.mode() != GNCMode::DISABLED)
    {
        return fail("timeout did not publish a zero, unarmed safe state");
    }
    if (timed_out.safety_status.mode !=
            hydrox::safety::VehicleMode::OutputDisabled ||
        timed_out.safety_status.actuator_authorized)
    {
        return fail("supervisor must inhibit output after IMU loss");
    }

    if (runtime.step(sensor(1'010'000), command_time + 2'010'000) !=
        StepStatus::OK)
    {
        return fail("safe runtime did not continue estimator ticks");
    }
    if (controller_probe->update_count != 1 ||
        runtime.last_tick().actuator.ch[0] != 0.0f)
    {
        return fail("controller retained actuator authority after timeout");
    }

    runtime.on_disconnected(start + 3'000'000);
    const uint64_t reconnect_time = start + 4'000'000;
    if (runtime.on_connected(reconnect_time) != 2)
        return fail("reconnect did not advance the control epoch");
    runtime.observe_valid_sensor(reconnect_time + 10'000);
    if (runtime.accept_setpoint(setpoint, GNCMode::DEPTH_HOLD, command_time))
        return fail("a command from the previous epoch crossed reconnect");

    const uint64_t second_command = reconnect_time + 20'000;
    if (!runtime.accept_setpoint(
            setpoint, GNCMode::DEPTH_HOLD, second_command))
    {
        return fail("new epoch command was rejected");
    }
    if (runtime.step(sensor(2'000'000), reconnect_time + 30'000) != StepStatus::OK)
        return fail("new epoch sensor tick was rejected");
    if (runtime.last_tick().actuator_authorized || !runtime.safety_status().operator_ack_required)
        return fail("reconnect cleared the hard fault");
    runtime.set_armed(false, reconnect_time + 31'000);
    if (!runtime.acknowledge_safety_fault(reconnect_time + 32'000) ||
        !runtime.set_armed(true, reconnect_time + 33'000) ||
        !runtime.accept_setpoint(setpoint, GNCMode::DEPTH_HOLD, reconnect_time + 34'000))
        return fail("explicit hard-fault recovery failed");
    if (runtime.step(sensor(3'000'000), reconnect_time + 40'000) !=
        StepStatus::SENSOR_TIME_GAP)
    {
        return fail("unsafe sensor time discontinuity was not rejected");
    }
    if (runtime.last_tick().actuator_authorized ||
        runtime.mode() != GNCMode::DISABLED)
    {
        return fail("sensor time discontinuity did not revoke actuator authority");
    }
    if (runtime.safety_status().mode !=
            hydrox::safety::VehicleMode::OutputDisabled ||
        runtime.safety_status().actuator_authorized)
    {
        return fail("safety supervisor did not disable output after estimator timing loss");
    }
    if (std::string(runtime.last_tick().safety_cause) != "SENSOR_TIME_DISCONTINUITY" ||
        runtime.last_tick().safety_observed_s != 1.0 ||
        runtime.last_tick().safety_threshold_s != config.max_sensor_dt_s)
        return fail("first rejected sensor dt diagnostic lost");
    const uint64_t pause_time = reconnect_time + 50'000;
    if (runtime.on_simulator_paused(pause_time) !=
        RuntimeEvent::SIMULATOR_PAUSED)
        return fail("pause did not enter the shared simulator failsafe");
    if (runtime.last_tick().actuator_authorized ||
        runtime.mode() != GNCMode::DISABLED ||
        runtime.control_session().phase() != ControlSessionPhase::DISCONNECTED)
    {
        return fail("pause retained command or actuator authority");
    }

    const uint64_t resume_time = pause_time + 10'000;
    if (runtime.on_simulator_resumed(resume_time) != 3 ||
        runtime.control_session().phase() !=
            ControlSessionPhase::WAITING_FOR_SENSOR)
    {
        return fail("resume did not create a fresh sensor-gated epoch");
    }
    if (runtime.accept_setpoint(
            setpoint, GNCMode::DEPTH_HOLD, resume_time + 1'000))
        return fail("post-resume command was accepted before a new sensor");
    if (runtime.observe_valid_sensor(resume_time + 10'000) !=
        RuntimeEvent::SENSOR_READY)
        return fail("post-resume sensor did not reopen the command gate");
    const uint64_t resumed_command_time = resume_time + 20'000;
    if (!runtime.accept_setpoint(
            setpoint, GNCMode::DEPTH_HOLD, resumed_command_time))
        return fail("fresh post-resume command was rejected");
    if (runtime.step(sensor(3'010'000), resume_time + 30'000) !=
        StepStatus::OK)
        return fail("post-resume sensor tick was rejected");
    if (runtime.last_tick().actuator_authorized || !runtime.safety_status().operator_ack_required)
        return fail("pause/resume bypassed a latched timing fault");

    auto active_controller = std::make_unique<ConstantController>();
    auto active_allocator = std::make_unique<ConstantAllocator>();
    HilRuntimeConfig active_config = config;
    active_config.safety_profile.external_command_loss_us = 500'000;
    active_config.sensor_timeout_us = 10'000'000;
    HilRuntime active_runtime(
        active_config,
        std::move(active_controller),
        std::move(active_allocator));
    const uint64_t active_start = start + 10'000'000;
    active_runtime.on_connected(active_start);
    active_runtime.observe_valid_sensor(active_start + 10'000);

    GNCSetpoint active_setpoint;
    active_setpoint.depth_ref = 20.0;
    active_setpoint.heading_ref = 1.0;
    active_setpoint.surge_ref = 5.0;
    active_setpoint.use_yaw_rate_ref = true;
    active_setpoint.yaw_rate_ref = 1.0;
    const uint64_t active_command_time = active_start + 20'000;
    if (!active_runtime.accept_setpoint(
            active_setpoint,
            GNCMode::DEPTH_HOLD,
            active_command_time))
    {
        return fail("active-safety runtime rejected initial external command");
    }
    if (active_runtime.step(
            sensor(10'000'000),
            active_start + 30'000) != StepStatus::OK ||
        !active_runtime.last_tick().actuator_authorized ||
        active_runtime.last_tick().safety_control_source !=
            hydrox::safety::ControlSource::External)
    {
        return fail("active-safety runtime did not authorize external source");
    }

    if (active_runtime.maintain(
            active_command_time + 200'001) != RuntimeEvent::NONE ||
        active_runtime.safety_status().mode !=
            hydrox::safety::VehicleMode::CommandHold)
    {
        return fail("active safety did not enter command hold before hard timeout");
    }
    if (active_runtime.step(
            sensor(10'010'000),
            active_command_time + 210'000) != StepStatus::OK ||
        !active_runtime.last_tick().actuator_authorized ||
        active_runtime.last_tick().safety_control_source !=
            hydrox::safety::ControlSource::Failsafe ||
        active_runtime.last_tick().actuator.ch[0] == 0.0f)
    {
        return fail("command hold did not transfer authority without zero output");
    }
    if (active_runtime.step(
            sensor(10'020'000),
            active_command_time + 220'000) != StepStatus::OK ||
        !(active_runtime.setpoint().yaw_rate_ref < 1.0) ||
        !(active_runtime.setpoint().yaw_rate_ref > 0.0))
    {
        return fail("command-hold yaw-rate reference was not slewed");
    }

    if (active_runtime.maintain(
            active_command_time + 500'001) !=
            RuntimeEvent::SETPOINT_TIMEOUT)
    {
        return fail("active-safety runtime did not report command-loss timeout");
    }
    const HilRuntimeTick active_timeout = active_runtime.last_tick();
    if (active_timeout.safety_status.mode !=
            hydrox::safety::VehicleMode::FailsafeStabilize ||
        !active_timeout.actuator_authorized ||
        (active_timeout.actuator_mode & kMavModeFlagSafetyArmed) == 0 ||
        active_timeout.actuator.ch[0] == 0.0f)
    {
        return fail("hard timeout inserted a zero frame instead of onboard fallback");
    }

    if (active_runtime.step(
            sensor(10'030'000),
            active_command_time + 510'000) != StepStatus::OK ||
        active_runtime.last_tick().safety_control_source !=
            hydrox::safety::ControlSource::Failsafe ||
        !active_runtime.last_tick().actuator_authorized)
    {
        return fail("stabilization did not retain failsafe control authority");
    }
    if (active_runtime.step(
            sensor(10'040'000),
            active_command_time + 610'000) != StepStatus::OK ||
        !(active_runtime.setpoint().surge_ref < 5.0) ||
        active_runtime.setpoint().surge_ref <
            active_config.safety_profile.minimum_control_surge_mps)
    {
        return fail("stabilization surge reference violated slew/authority bounds");
    }

    if (active_runtime.maintain(
            active_command_time + 3'600'000) != RuntimeEvent::NONE ||
        active_runtime.safety_status().mode !=
            hydrox::safety::VehicleMode::FailsafeSurface)
    {
        return fail("active safety did not escalate stabilization to surfacing");
    }
    if (active_runtime.step(
            sensor(10'050'000),
            active_command_time + 3'610'000) != StepStatus::OK ||
        !active_runtime.last_tick().actuator_authorized ||
        active_runtime.last_tick().safety_control_source !=
            hydrox::safety::ControlSource::Failsafe)
    {
        return fail("controlled surfacing did not retain onboard authority");
    }

    if (!active_runtime.accept_setpoint(
            active_setpoint,
            GNCMode::DEPTH_HOLD,
            active_command_time + 3'620'000))
    {
        return fail("fresh external packet could not enter revoked session gate");
    }
    if (active_runtime.step(
            sensor(10'060'000),
            active_command_time + 3'630'000) != StepStatus::OK ||
        active_runtime.safety_status().mode !=
            hydrox::safety::VehicleMode::FailsafeSurface ||
        active_runtime.last_tick().safety_control_source !=
            hydrox::safety::ControlSource::Failsafe)
    {
        return fail("fresh packets automatically stole authority from latched failsafe");
    }

    active_runtime.set_armed(false, active_command_time + 3'640'000);
    if (active_runtime.last_tick().actuator_authorized ||
        active_runtime.safety_status().mode !=
            hydrox::safety::VehicleMode::FaultLocked)
    {
        return fail("operator disarm did not end fallback in locked zero output");
    }
    if (active_runtime.set_armed(
            true, active_command_time + 3'650'000))
    {
        return fail("latched fallback rearmed without explicit acknowledgement");
    }
    if (!active_runtime.acknowledge_safety_fault(
            active_command_time + 3'660'000) ||
        !active_runtime.set_armed(
            true, active_command_time + 3'670'000) ||
        active_runtime.safety_status().mode !=
            hydrox::safety::VehicleMode::ArmedIdle)
    {
        return fail("acknowledge/rearm sequence did not return to armed idle");
    }

    auto invalid_controller = std::make_unique<ConstantController>();
    auto invalid_allocator = std::make_unique<InvalidAllocator>();
    HilRuntime invalid_runtime(
        config, std::move(invalid_controller), std::move(invalid_allocator));
    const uint64_t invalid_start = start + 20'000'000;
    invalid_runtime.on_connected(invalid_start);
    if (invalid_runtime.observe_valid_sensor(invalid_start + 10'000) !=
        RuntimeEvent::SENSOR_READY)
    {
        return fail("invalid-contract test did not open its sensor gate");
    }
    if (!invalid_runtime.accept_setpoint(
            setpoint, GNCMode::DEPTH_HOLD, invalid_start + 20'000))
    {
        return fail("invalid-contract test did not accept its fresh setpoint");
    }
    if (invalid_runtime.step(
            sensor(4'000'000), invalid_start + 30'000) != StepStatus::OK)
    {
        return fail("invalid-contract test sensor tick was rejected");
    }
    const HilRuntimeTick invalid_tick = invalid_runtime.last_tick();
    if (!invalid_tick.safety_status.operator_ack_required ||
        invalid_tick.safety_status.mode != safety::VehicleMode::OutputDisabled ||
        std::string(invalid_tick.safety_cause) != "CONTROL_OUTPUT_INVALID" ||
        invalid_tick.actuator_authorized ||
        (invalid_tick.actuator_mode & kMavModeFlagSafetyArmed) != 0 ||
        invalid_tick.actuator.ch[0] != 0.0f)
    {
        return fail("invalid actuator contract was not cleared and disarmed");
    }

    const auto run_augmentor_case = [&](bool active, bool invalid,
                                        uint64_t time_offset,
                                        float expected_actuator)
    {
        auto case_controller = std::make_unique<ConstantController>();
        auto case_allocator = std::make_unique<ConstantAllocator>();
        OffsetAugmentor augmentor(invalid);
        HilRuntimeConfig case_config = config;
        case_config.wrench_augmentor_active_enabled = active;
        HilRuntime case_runtime(
            case_config, std::move(case_controller),
            std::move(case_allocator), &augmentor);
        const uint64_t case_start = start + time_offset;
        case_runtime.on_connected(case_start);
        if (case_runtime.observe_valid_sensor(case_start + 10'000) !=
            RuntimeEvent::SENSOR_READY)
        {
            return false;
        }
        if (!case_runtime.accept_setpoint(
                setpoint, GNCMode::DEPTH_HOLD, case_start + 20'000))
        {
            return false;
        }
        if (case_runtime.step(
                sensor(20'000'000 + time_offset), case_start + 30'000) !=
            StepStatus::OK)
        {
            return false;
        }
        const HilRuntimeTick tick = case_runtime.last_tick();
        const bool candidate_valid = !invalid;
        const bool candidate_active = active && candidate_valid;
        return augmentor.update_count == 1 &&
            tick.wrench_augmentor_observed &&
            tick.wrench_augmentor_candidate_valid == candidate_valid &&
            tick.wrench_augmentor_active == candidate_active &&
            std::abs(tick.base_wrench[0] - 10.0) < 1e-12 &&
            std::abs(tick.wrench[0] - (candidate_active ? 15.0 : 10.0)) < 1e-12 &&
            std::abs(tick.actuator.ch[0] - expected_actuator) < 1e-6f;
    };

    if (!run_augmentor_case(false, false, 30'000'000, 0.5f))
        return fail("shadow augmentor changed the classical actuator output");
    if (!run_augmentor_case(true, false, 40'000'000, 0.75f))
        return fail("explicit augmentor authority did not select its valid candidate");
    if (!run_augmentor_case(true, true, 50'000'000, 0.5f))
        return fail("non-finite active augmentor candidate did not fail closed");

    HilRuntime timeout_runtime(config, std::make_unique<ConstantController>(),
                               std::make_unique<ConstantAllocator>());
    timeout_runtime.on_connected(1000000);
    timeout_runtime.observe_valid_sensor(1010000);
    if (!timeout_runtime.accept_setpoint(setpoint, GNCMode::DEPTH_HOLD, 1020000))
        return fail("timeout diagnostic setup");
    if (timeout_runtime.maintain(1620000) != RuntimeEvent::SENSOR_TIMEOUT ||
        std::string(timeout_runtime.last_tick().safety_cause) != "SENSOR_TIMEOUT" ||
        std::abs(timeout_runtime.last_tick().safety_observed_s - .61) > 1e-9 ||
        timeout_runtime.last_tick().safety_threshold_s != .5)
        return fail("IMU silent timeout diagnostic missing trigger age");

    // Even after command authority is revoked, sensor watchdogs must keep running.
    HilRuntime watchdog_runtime(config, std::make_unique<ConstantController>(),
                                std::make_unique<ConstantAllocator>());
    watchdog_runtime.on_connected(60'000'000);
    watchdog_runtime.step(sensor(60'010'000), 60'010'000);
    watchdog_runtime.accept_setpoint(setpoint, GNCMode::DEPTH_HOLD, 60'020'000);
    watchdog_runtime.step(sensor(60'030'000), 60'030'000);
    if (!watchdog_runtime.accept_setpoint(GNCSetpoint{}, GNCMode::DISABLED, 60'040'000))
        return fail("explicit external release was rejected");
    if (watchdog_runtime.safety_status().mode != safety::VehicleMode::FailsafeStabilize)
        return fail("disabled external command must release authority, not create an invalid reference");
    if (!watchdog_runtime.last_tick().actuator_authorized)
        return fail("command-link release cut output before onboard takeover");
    watchdog_runtime.step(sensor(60'050'000), 60'050'000);
    if (watchdog_runtime.last_tick().safety_control_source != safety::ControlSource::Failsafe)
        return fail("command-link release did not transfer to onboard control");
    if (watchdog_runtime.maintain(60'550'001) != RuntimeEvent::SENSOR_TIMEOUT ||
        watchdog_runtime.last_tick().actuator_authorized ||
        watchdog_runtime.safety_status().mode != safety::VehicleMode::OutputDisabled)
        return fail("IMU watchdog stopped when external authority was revoked");
    if (watchdog_runtime.resume_external_control(60'550'002))
        return fail("continue bypassed IMU loss");

    HilRuntime pause_runtime(config, std::make_unique<ConstantController>(),
                             std::make_unique<ConstantAllocator>());
    pause_runtime.on_connected(70'000'000);
    pause_runtime.step(sensor(70'010'000), 70'010'000);
    pause_runtime.accept_setpoint(setpoint, GNCMode::DEPTH_HOLD, 70'020'000);
    pause_runtime.step(sensor(70'030'000), 70'030'000);
    pause_runtime.on_simulator_paused(70'040'000);
    if (pause_runtime.last_tick().actuator_authorized ||
        !pause_runtime.safety_status().session_suspended)
        return fail("pause did not inhibit the sole safety output gate");
    const auto paused_tick = pause_runtime.last_tick().tick;
    if (pause_runtime.step(sensor(75'000'000), 75'000'000) != StepStatus::NO_SENSOR)
        return fail("runtime integrated a sensor while the simulation was paused");
    if (pause_runtime.maintain(80'000'000) != RuntimeEvent::NONE ||
        pause_runtime.safety_status().operator_ack_required ||
        pause_runtime.last_tick().tick != paused_tick)
        return fail("paused simulation fabricated control steps or a sensor fault");
    pause_runtime.on_simulator_resumed(80'010'000);
    pause_runtime.step(sensor(80'020'000), 80'020'000);
    if (pause_runtime.last_tick().actuator_authorized ||
        pause_runtime.accept_setpoint(setpoint, GNCMode::DEPTH_HOLD, 70'020'000))
        return fail("old command crossed the resumed epoch");
    pause_runtime.accept_setpoint(setpoint, GNCMode::DEPTH_HOLD, 80'030'000);
    pause_runtime.step(sensor(80'040'000), 80'040'000);
    if (!pause_runtime.last_tick().actuator_authorized)
        return fail("healthy pause/resume did not recover with both fresh inputs");

    // An incoming packet must not hide an already elapsed command-loss deadline.
    HilRuntime late_runtime(config, std::make_unique<ConstantController>(),
                            std::make_unique<ConstantAllocator>());
    late_runtime.on_connected(90'000'000);
    late_runtime.step(sensor(90'010'000), 90'010'000);
    late_runtime.accept_setpoint(setpoint, GNCMode::DEPTH_HOLD, 90'020'000);
    late_runtime.step(sensor(90'030'000), 90'030'000);
    late_runtime.observe_valid_sensor(92'030'000);
    late_runtime.accept_setpoint(setpoint, GNCMode::DEPTH_HOLD, 92'040'000);
    if (!late_runtime.safety_status().operator_ack_required ||
        late_runtime.safety_status().external_authorized)
        return fail("late packet silently cleared sustained command loss");

    std::puts("PASS: shared SITL/HITL runtime safety pipeline");
    return 0;
}
