#include "gnc/thruster_controller.h"

#include <cmath>
#include <iostream>

namespace
{
int expect(bool condition, const char* message)
{
    if (condition)
        return 0;
    std::cerr << "FAIL: " << message << '\n';
    return 1;
}
}

int main()
{
    int failures = 0;

    hydrox::ThrusterVehicleController::Params params;
    params.mass = 10.0;
    params.surge.kp = 1.0;
    params.surge.ki = 0.5;
    params.surge.kd = 100.0;
    params.surge.integral_limit = 2.0;
    params.surge.accel_max = 10.0;
    hydrox::ThrusterVehicleController controller(params);

    hydrox::GNCSetpoint speed;
    speed.surge_ref = 1.0;
    controller.set_mode(hydrox::GNCMode::DEPTH_HOLD);
    controller.set_setpoint(speed);
    auto state = hydrox::NavigationState::zeros();
    state.nu[0] = 0.5;

    const auto first = controller.update(state, 1.0);
    const auto second = controller.update(state, 1.0);
    failures += expect(std::abs(first[0] - 7.5) < 1.0e-9,
                       "thruster speed loop uses P+I without steady velocity damping");
    failures += expect(second[0] > first[0],
                       "thruster speed integral compensates persistent drag deficit");

    speed.surge_ref = 0.0;
    controller.set_setpoint(speed);
    const auto terminal = controller.update(state, 0.1);
    failures += expect(std::abs(terminal[0] + 5.0) < 1.0e-9,
                       "zero-speed terminal hold clears residual integral thrust");

    controller.set_mode(hydrox::GNCMode::DISABLED);
    (void)controller.update(state, 1.0);
    controller.set_mode(hydrox::GNCMode::DEPTH_HOLD);
    speed.surge_ref = 1.0;
    controller.set_setpoint(speed);
    const auto restarted = controller.update(state, 1.0);
    failures += expect(std::abs(restarted[0] - first[0]) < 1.0e-9,
                       "disabled transition resets the speed integral");

    if (failures == 0)
        std::cout << "test_thruster_controller: all checks passed\n";
    return failures == 0 ? 0 : 1;
}
