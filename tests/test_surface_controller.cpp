#include "gnc/surface_controller.h"

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

    hydrox::SurfaceVesselController::Params params;
    params.surge_kp = 100.0;
    params.surge_kd = 0.0;
    params.waypoint_surge_mps = 3.0;
    hydrox::SurfaceVesselController controller(params);

    hydrox::GNCSetpoint waypoint;
    waypoint.wp_n = 50.0;
    waypoint.wp_e = 0.0;
    waypoint.surge_ref = 0.5;
    controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);
    controller.set_setpoint(waypoint);

    const auto state = hydrox::NavigationState::zeros();
    const auto approach = controller.update(state, 0.01);
    failures += expect(std::abs(approach[0] - 50.0) < 1.0e-9,
                       "surface waypoint honors a planner approach-speed cap");

    auto cross_track_state = hydrox::NavigationState::zeros();
    cross_track_state.eta[5] = 0.5 * 3.14159265358979323846;
    const auto turn_in_place = controller.update(cross_track_state, 0.01);
    failures += expect(std::abs(turn_in_place[0]) < 1.0e-9,
                       "surface waypoint removes forward thrust while broadside to the goal");

    waypoint.surge_ref = 0.0;
    controller.set_setpoint(waypoint);
    const auto hold = controller.update(state, 0.01);
    failures += expect(std::abs(hold[0]) < 1.0e-9,
                       "surface terminal hold commands zero surge at rest");

    auto coasting_state = hydrox::NavigationState::zeros();
    coasting_state.nu[0] = 1.0;
    const auto coasting_hold = controller.update(coasting_state, 0.01);
    failures += expect(std::abs(coasting_hold[0]) < 1.0e-9,
                       "surface terminal hold lets passive hull drag slow a coasting vessel");

    hydrox::SurfaceVesselController::Params disturbance_params;
    disturbance_params.surge_kp = 100.0;
    disturbance_params.surge_ki = 50.0;
    disturbance_params.surge_integral_limit = 1.0;
    disturbance_params.yaw_kp = 100.0;
    disturbance_params.sideslip_compensation_gain = 1.0;
    disturbance_params.max_crab_angle_rad = 0.3;
    disturbance_params.waypoint_surge_mps = 3.0;
    hydrox::SurfaceVesselController disturbance_controller(disturbance_params);
    waypoint.surge_ref = 1.0;
    disturbance_controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);
    disturbance_controller.set_setpoint(waypoint);
    auto disturbed_state = hydrox::NavigationState::zeros();
    disturbed_state.nu[0] = 0.8;
    disturbed_state.nu[1] = -0.4;
    const auto integral_first = disturbance_controller.update(disturbed_state, 0.1);
    const auto integral_second = disturbance_controller.update(disturbed_state, 0.1);
    failures += expect(integral_second[0] > integral_first[0],
                       "surface speed integral compensates persistent drag deficit");
    failures += expect(integral_second[5] > 0.0,
                       "surface sideslip compensation crabs into negative sway");

    if (failures == 0)
        std::cout << "test_surface_controller: all checks passed\n";
    return failures == 0 ? 0 : 1;
}
