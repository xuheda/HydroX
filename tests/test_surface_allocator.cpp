#include "gnc/surface_allocator.h"
#include "gnc/surface_control_demand.h"

#include <cmath>
#include <iostream>

namespace
{
int expect(bool condition, const char *message)
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

    hydrox::TwinScrewAllocator::Params params;
    params.max_thrust_N = 100.0;
    params.lever_arm_m = 2.0;
    hydrox::TwinScrewAllocator allocator(params);

    hydrox::SurfaceControlDemand demand;
    demand.surge_force_N = 100.0;
    demand.yaw_moment_Nm = 40.0;
    const auto command = allocator.allocate(demand.to_wrench(), 0.0);
    failures += expect(std::abs(command.ch[0] - 0.4f) < 1.0e-6f,
                       "port command combines surge and yaw demand");
    failures += expect(std::abs(command.ch[1] - 0.6f) < 1.0e-6f,
                       "starboard command combines surge and yaw demand");

    demand.surge_force_N = 200.0;
    demand.yaw_moment_Nm = 200.0;
    const auto saturated = allocator.allocate(demand.to_wrench(), 0.0);
    failures += expect(
        std::abs(saturated.ch[0] - 1.0f / 3.0f) < 1.0e-6f &&
            std::abs(saturated.ch[1] - 1.0f) < 1.0e-6f,
        "joint desaturation preserves the surge-to-yaw wrench ratio");

    hydrox::Wrench unsupported = hydrox::Wrench::Zero();
    unsupported[1] = 100.0;
    unsupported[2] = 100.0;
    unsupported[3] = 100.0;
    unsupported[4] = 100.0;
    const auto ignored = allocator.allocate(unsupported, 0.0);
    failures += expect(
        std::abs(ignored.ch[0]) < 1.0e-9f &&
            std::abs(ignored.ch[1]) < 1.0e-9f,
        "sway heave roll and pitch do not enter twin-screw allocation");

    if (failures == 0)
        std::cout << "test_surface_allocator: all checks passed\n";
    return failures == 0 ? 0 : 1;
}
