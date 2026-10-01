#include "hydrox/runtime/control_session.h"

#include <cstdio>

namespace
{
    int fail(const char *message)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        return 1;
    }
}

int main()
{
    using hydrox::runtime::ControlSessionGate;
    using hydrox::runtime::ControlSessionPhase;

    constexpr uint64_t kMillisecond = 1000;
    constexpr uint64_t kSecond = 1000 * kMillisecond;
    constexpr uint64_t start = 10 * kSecond;

    ControlSessionGate gate;

    if (gate.accept_setpoint(start))
        return fail("disconnected gate accepted a Setpoint");

    if (gate.on_connected(start) != 1 ||
        gate.phase() != ControlSessionPhase::WAITING_FOR_SENSOR)
    {
        return fail("first connection did not create a waiting session");
    }

    const auto command_before_sensor = start + 10 * kMillisecond;
    if (gate.accept_setpoint(command_before_sensor))
        return fail("Setpoint was accepted before a valid sensor sample");

    const auto sensor_ready = start + 20 * kMillisecond;
    if (!gate.observe_valid_sensor(sensor_ready) ||
        gate.phase() != ControlSessionPhase::WAITING_FOR_SETPOINT)
    {
        return fail("valid sensor did not open the Setpoint gate");
    }
    if (gate.accept_setpoint(command_before_sensor))
        return fail("pre-sensor Setpoint was replayed after sensor readiness");

    const auto first_command = start + 30 * kMillisecond;
    if (!gate.accept_setpoint(first_command) || !gate.is_active())
        return fail("fresh post-sensor Setpoint was rejected");
    if (gate.setpoint_timed_out(
            first_command + 1999 * kMillisecond, 2 * kSecond))
    {
        return fail("Setpoint timed out too early");
    }
    if (!gate.setpoint_timed_out(
            first_command + 2001 * kMillisecond, 2 * kSecond))
    {
        return fail("Setpoint did not time out on monotonic time");
    }

    gate.revoke_setpoint(first_command + 2001 * kMillisecond);
    if (gate.phase() != ControlSessionPhase::WAITING_FOR_SETPOINT ||
        gate.accept_setpoint(first_command))
    {
        return fail("revocation allowed an older Setpoint to reactivate control");
    }

    gate.on_disconnected(start + 3 * kSecond);
    if (gate.phase() != ControlSessionPhase::DISCONNECTED ||
        gate.setpoint_age_s(start + 4 * kSecond) >= 0.0)
    {
        return fail("disconnect did not clear active Setpoint state");
    }

    const auto reconnect = start + 5 * kSecond;
    if (gate.on_connected(reconnect) != 2)
        return fail("session generation did not advance on reconnect");
    const auto second_sensor = start + 5010 * kMillisecond;
    if (!gate.observe_valid_sensor(second_sensor))
        return fail("reconnected session did not accept its first sensor");
    if (gate.accept_setpoint(first_command))
        return fail("previous session Setpoint crossed reconnect boundary");

    const auto second_command = start + 5020 * kMillisecond;
    if (!gate.accept_setpoint(second_command) || !gate.is_active())
        return fail("fresh Setpoint did not activate the reconnected session");

    if (hydrox::runtime::duration_us_from_seconds(2.5) !=
        2500 * kMillisecond)
    {
        return fail("seconds-to-microseconds conversion is incorrect");
    }

    std::puts("PASS: runtime control session gate");
    return 0;
}
