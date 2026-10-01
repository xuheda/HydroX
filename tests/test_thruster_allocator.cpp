#include "gnc/thruster_allocator.h"
#include "gnc/control_factory.h"

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
    hydrox::ControlParameters params;
    params.archetype = hydrox::VehicleArchetype::Thruster;
    params.direct_body_wrench = true;
    params.max_thrust_per_thruster_N = 30.0;
    auto stack = hydrox::build_control_stack(params);

    hydrox::Wrench tau = hydrox::Wrench::Zero();
    tau[0] = 12.0;
    tau[2] = -6.0;
    tau[5] = 4.8;
    const auto command = stack.allocator->allocate(tau, 0.0);

    int failures = 0;
    failures += expect(command.layout == hydrox::ActuatorLayout::DirectBodyWrench,
                       "factory selects the dedicated body-wrench allocator");
    hydrox::ThrusterMatrixAllocator::Params thrusters;
    hydrox::Thruster thruster;
    thruster.max_thrust_N = 100.0;
    thrusters.thrusters.push_back(thruster);
    hydrox::ThrusterMatrixAllocator physical_allocator(thrusters);
    const auto physical_command = physical_allocator.allocate(tau, 0.0);
    failures += expect(physical_command.layout == hydrox::ActuatorLayout::ThrusterArray,
                       "matrix allocator always emits individual thruster commands");
    failures += expect(std::abs(physical_command.ch[0] - 0.12f / 1.001f) < 1e-5f,
                       "matrix allocator retains damped allocation and normalization");
    failures += expect(std::abs(command.ch[0] - 0.05f) < 1.0e-6f,
                       "direct ROV bridge maps surge wrench to channel 0");
    failures += expect(std::abs(command.ch[1]) < 1.0e-6f,
                       "direct ROV bridge does not reinterpret surge as sway");
    failures += expect(std::abs(command.ch[2] + 0.025f) < 1.0e-6f,
                       "direct ROV bridge maps heave wrench to channel 2");
    failures += expect(std::abs(command.ch[5] - 0.02f) < 1.0e-6f,
                       "direct ROV bridge maps yaw wrench to channel 5");

    if (failures == 0)
        std::cout << "test_thruster_allocator: all checks passed\n";
    return failures == 0 ? 0 : 1;
}
