#include "gnc/thruster_allocator.h"

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
    hydrox::ThrusterMatrixAllocator::Params params;
    params.direct_wrench_output = true;
    params.direct_force_limit_N = 240.0;
    params.direct_moment_limit_Nm = 240.0;
    hydrox::ThrusterMatrixAllocator allocator(params);

    hydrox::Wrench tau = hydrox::Wrench::Zero();
    tau[0] = 12.0;
    tau[2] = -6.0;
    tau[5] = 4.8;
    const auto command = allocator.allocate(tau, 0.0);

    int failures = 0;
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
