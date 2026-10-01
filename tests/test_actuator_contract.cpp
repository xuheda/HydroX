#include "gnc/control_interfaces.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
int expect(bool condition, const char* message)
{
    if (condition)
        return 0;
    std::fprintf(stderr, "FAIL: %s\n", message);
    return 1;
}
}

int main()
{
    using namespace hydrox;
    int failures = 0;

    ActuatorCmd safe_zero;
    failures += expect(safe_zero.valid(),
                       "zero inactive command is a valid safe state");

    ActuatorCmd motor;
    motor.layout = ActuatorLayout::QuadX;
    motor.set_thrust(0, 5.0, 10.0);
    motor.set_thrust(1, 0.0, 10.0);
    motor.set_thrust(2, 0.0, 10.0);
    motor.set_thrust(3, 0.0, 10.0);
    failures += expect(
        motor.valid() && std::abs(motor.ch[0] - 0.5f) < 1.0e-6f &&
            motor.contract[0].quantity == ActuatorQuantity::Thrust &&
            !motor.contract[0].reversible,
        "physical motor thrust maps once to an unidirectional normalized channel");

    motor.set_thrust(0, -5.0, 10.0);
    failures += expect(
        motor.valid() && motor.ch[0] == 0.0f,
        "unidirectional motor rejects negative thrust");

    ActuatorCmd surface;
    surface.layout = ActuatorLayout::ConventionalFixedWing;
    surface.set_position(0, -0.39, 0.78, 0.78);
    surface.set_position(1, 0.0, 0.78, 0.78);
    surface.set_position(2, 0.0, 0.78, 0.78);
    surface.set_thrust(3, 0.0, 10.0);
    failures += expect(
        surface.valid() && std::abs(surface.ch[0] + 0.5f) < 1.0e-6f &&
            surface.contract[0].quantity == ActuatorQuantity::Position &&
            surface.contract[0].reversible,
        "physical surface deflection maps to a signed normalized position");

    ActuatorCmd wheel;
    wheel.layout = ActuatorLayout::DifferentialDrive;
    wheel.set_angular_velocity(0, -10.0, 20.0, 20.0);
    wheel.set_angular_velocity(1, 0.0, 20.0, 20.0);
    failures += expect(
        wheel.valid() && std::abs(wheel.ch[0] + 0.5f) < 1.0e-6f,
        "typed wheel angular velocity remains reversible");

    ActuatorCmd raw;
    raw.ch[0] = 0.2f;
    failures += expect(
        !raw.valid(),
        "raw channel values without quantity and layout metadata are rejected");

    ActuatorCmd gap;
    gap.layout = ActuatorLayout::QuadX;
    gap.set_thrust(1, 1.0, 10.0);
    failures += expect(
        !gap.valid(),
        "active actuator channels must form a complete declared layout prefix");

    ActuatorCmd invalid_limit;
    invalid_limit.layout = ActuatorLayout::QuadX;
    invalid_limit.set_thrust(0, 1.0, 0.0);
    invalid_limit.set_thrust(1, 0.0, 10.0);
    invalid_limit.set_thrust(2, 0.0, 10.0);
    invalid_limit.set_thrust(3, 0.0, 10.0);
    failures += expect(
        !invalid_limit.valid(),
        "zero physical actuator authority cannot be normalized");

    ActuatorCmd invalid_value;
    invalid_value.layout = ActuatorLayout::QuadX;
    invalid_value.set_thrust(
        0, std::numeric_limits<double>::quiet_NaN(), 10.0);
    invalid_value.set_thrust(1, 0.0, 10.0);
    invalid_value.set_thrust(2, 0.0, 10.0);
    invalid_value.set_thrust(3, 0.0, 10.0);
    failures += expect(
        !invalid_value.valid(),
        "non-finite physical actuator requests are rejected");

    if (failures == 0)
        std::puts("test_actuator_contract: all checks passed");
    return failures == 0 ? 0 : 1;
}
