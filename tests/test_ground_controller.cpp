#include "gnc/control_factory.h"
#include "vehicle_bundle.h"

#include <cmath>
#include <cstdio>

namespace
{
int expect(bool condition, const char *message)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        return 1;
    }
    return 0;
}
}

int main()
{
    using namespace hydrox;
    int failures = 0;
    std::string error;
    const auto bundle = load_vehicle_bundle(
        "profiles/r1-rover/vehicle-bundle.json", &error);
    failures += expect(bundle.valid && error.empty(), "R1Rover bundle validates");
    failures += expect(bundle.control.vehicle_class == VehicleClass::UGV_DIFFERENTIAL,
                       "R1Rover selects UGV vehicle class");
    failures += expect(bundle.logical_actuator_count == 2,
                       "R1Rover exposes exactly two logical wheel channels");

    auto stack = build_control_stack(bundle);
    failures += expect(stack.controller && stack.allocator,
                       "R1Rover control stack builds");

    NavigationState state = NavigationState::zeros();
    GNCSetpoint sp;
    sp.surge_ref = 1.0;
    sp.use_yaw_rate_ref = true;
    sp.yaw_rate_ref = 0.8;
    stack.controller->set_mode(GNCMode::DEPTH_HOLD);
    stack.controller->set_setpoint(sp);
    const auto wrench = stack.controller->update(state, 0.02);
    const auto command = stack.allocator->allocate(wrench, state.nu[0]);
    failures += expect(command.ch[1] > command.ch[0],
                       "positive NED yaw commands left wheels faster than right wheels");
    failures += expect(command.ch[0] >= -1.0f && command.ch[1] <= 1.0f,
                       "wheel commands remain normalized");

    state.eta[5] = 3.14159265358979323846;
    sp.wp_n = 10.0;
    sp.wp_e = 0.0;
    sp.surge_ref = 1.0;
    stack.controller->set_mode(GNCMode::WAYPOINT_3D);
    stack.controller->set_setpoint(sp);
    const auto turn_wrench = stack.controller->update(state, 0.02);
    failures += expect(std::abs(turn_wrench[0]) < 1.0e-9,
                       "waypoint behind rover produces turn-in-place command");

    stack.controller->set_mode(GNCMode::DISABLED);
    const auto safe = stack.allocator->allocate(
        stack.controller->update(state, 0.02), 0.0);
    failures += expect(std::abs(safe.ch[0]) < 1.0e-9 &&
                       std::abs(safe.ch[1]) < 1.0e-9,
                       "disabled mode is a zero-wheel-speed safe stop");

    if (failures == 0)
        std::printf("test_ground_controller: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
