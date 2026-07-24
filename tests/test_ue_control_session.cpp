#include "ue_control_session.h"

#include <chrono>
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
    using namespace std::chrono_literals;
    using hydrox::sitl::UeControlSessionGate;
    using hydrox::sitl::UeControlSessionPhase;

    UeControlSessionGate gate;
    const auto start = UeControlSessionGate::Clock::now();

    if (gate.accept_setpoint(start))
        return fail("disconnected gate accepted a Setpoint");

    if (gate.on_connected(start) != 1 ||
        gate.phase() != UeControlSessionPhase::WAITING_FOR_SENSOR)
    {
        return fail("first UE connection did not create a waiting session");
    }

    const auto command_before_sensor = start + 10ms;
    if (gate.accept_setpoint(command_before_sensor))
        return fail("Setpoint was accepted before a valid sensor sample");

    const auto sensor_ready = start + 20ms;
    if (!gate.observe_valid_sensor(sensor_ready) ||
        gate.phase() != UeControlSessionPhase::WAITING_FOR_SETPOINT)
    {
        return fail("valid sensor did not open the Setpoint gate");
    }
    if (gate.accept_setpoint(command_before_sensor))
        return fail("pre-sensor Setpoint was replayed after sensor readiness");

    const auto first_command = start + 30ms;
    if (!gate.accept_setpoint(first_command) || !gate.is_active())
        return fail("fresh post-sensor Setpoint was rejected");
    if (gate.setpoint_timed_out(first_command + 1999ms, 2s))
        return fail("Setpoint timed out too early");
    if (!gate.setpoint_timed_out(first_command + 2001ms, 2s))
        return fail("Setpoint did not time out on steady-clock time");

    gate.revoke_setpoint(first_command + 2001ms);
    if (gate.phase() != UeControlSessionPhase::WAITING_FOR_SETPOINT ||
        gate.accept_setpoint(first_command))
    {
        return fail("revocation allowed an older Setpoint to reactivate control");
    }

    gate.on_disconnected(start + 3s);
    if (gate.phase() != UeControlSessionPhase::DISCONNECTED ||
        gate.setpoint_age_s(start + 4s) >= 0.0)
    {
        return fail("disconnect did not clear active Setpoint state");
    }

    const auto reconnect = start + 5s;
    if (gate.on_connected(reconnect) != 2)
        return fail("UE session generation did not advance on reconnect");
    const auto second_sensor = start + 5010ms;
    if (!gate.observe_valid_sensor(second_sensor))
        return fail("reconnected session did not accept its first sensor");
    if (gate.accept_setpoint(first_command))
        return fail("previous UE session Setpoint crossed reconnect boundary");

    const auto second_command = start + 5020ms;
    if (!gate.accept_setpoint(second_command) || !gate.is_active())
        return fail("fresh Setpoint did not activate the reconnected session");

    std::puts("PASS: UE control session gate");
    return 0;
}
