#include "hydrox/runtime/hil_runtime.h"

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
            command.ch[0] = static_cast<float>(wrench[0] / 20.0);
            command.rpm = 100.0;
            return command;
        }
    };

    hydrox::NavigationInput sensor(uint64_t timestamp_us)
    {
        hydrox::NavigationInput input;
        input.got_imu = true;
        input.imu.time_usec = timestamp_us;
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
    config.setpoint_timeout_us = 2'000'000;
    config.sensor_timeout_us = 500'000;
    HilRuntime runtime(
        config, std::move(controller), std::move(allocator));
    if (!runtime.valid())
        return fail("valid controller stack was rejected");

    constexpr uint64_t start = 10'000'000;
    if (runtime.on_connected(start) != 1)
        return fail("connection generation did not start at one");

    GNCSetpoint setpoint;
    setpoint.surge_ref = 1.0;
    if (runtime.accept_setpoint(setpoint, GNCMode::DEPTH_HOLD, start + 1))
        return fail("command was accepted before a sensor from this epoch");

    if (runtime.observe_valid_sensor(start + 10'000) !=
        RuntimeEvent::SENSOR_READY)
    {
        return fail("first valid sensor did not open the command gate");
    }
    const uint64_t command_time = start + 20'000;
    if (!runtime.accept_setpoint(
            setpoint, GNCMode::DEPTH_HOLD, command_time))
    {
        return fail("fresh post-sensor command was rejected");
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
        RuntimeEvent::SETPOINT_TIMEOUT)
    {
        return fail("freshness timeout did not enter failsafe");
    }
    const HilRuntimeTick timed_out = runtime.last_tick();
    if (timed_out.actuator_authorized ||
        (timed_out.actuator_mode & kMavModeFlagSafetyArmed) != 0 ||
        timed_out.actuator.ch[0] != 0.0f ||
        runtime.mode() != GNCMode::DISABLED)
    {
        return fail("timeout did not publish a zero, unarmed safe state");
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
    if (runtime.step(sensor(2'000'000), reconnect_time + 30'000) !=
        StepStatus::OK)
    {
        return fail("new epoch sensor tick was rejected");
    }
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

    std::puts("PASS: shared SITL/HITL runtime safety pipeline");
    return 0;
}
