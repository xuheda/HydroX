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
    params.max_accel_mps2 = 1.0;
    params.max_decel_mps2 = 1.0;
    hydrox::SurfaceVesselController controller(params);

    hydrox::GNCSetpoint waypoint;
    waypoint.wp_n = 50.0;
    waypoint.wp_e = 0.0;
    waypoint.surge_ref = 0.5;
    controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);
    controller.set_setpoint(waypoint);

    const auto state = hydrox::NavigationState::zeros();
    hydrox::Wrench approach = hydrox::Wrench::Zero();
    for (int i = 0; i < 50; ++i)
        approach = controller.update(state, 0.01);
    failures += expect(std::abs(approach[0] - 50.0) < 1.0e-9,
                       "surface waypoint honors a planner approach-speed cap");

    hydrox::SurfaceVesselController broadside_controller(params);
    broadside_controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);
    broadside_controller.set_setpoint(waypoint);
    auto cross_track_state = hydrox::NavigationState::zeros();
    cross_track_state.eta[5] = 0.5 * 3.14159265358979323846;
    const auto turn_in_place = broadside_controller.update(cross_track_state, 0.01);
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
    failures += expect(coasting_hold[0] < -40.0,
                       "surface terminal hold commands bounded active braking");

    hydrox::SurfaceVesselController::Params drag_params = params;
    drag_params.surge_drag_linear_N_per_mps = 10.0;
    drag_params.surge_drag_quadratic_N_per_mps2 = 20.0;
    hydrox::SurfaceVesselController drag_controller(drag_params);
    hydrox::GNCSetpoint cruise;
    cruise.surge_ref = 1.0;
    drag_controller.set_mode(hydrox::GNCMode::SURFACE);
    drag_controller.set_setpoint(cruise);
    auto cruise_state = hydrox::NavigationState::zeros();
    cruise_state.nu[0] = 1.0;
    const auto drag_feed_forward = drag_controller.update(cruise_state, 0.01);
    failures += expect(std::abs(drag_feed_forward[0] - 30.0) < 1.0e-9,
                       "surface speed loop feeds forward calibrated hull drag");

    hydrox::SurfaceVesselController::Params yaw_params = params;
    yaw_params.yaw_kp = 100.0;
    yaw_params.yaw_kd = 50.0;
    yaw_params.max_yaw_rate_radps = 0.2;
    hydrox::SurfaceVesselController yaw_controller(yaw_params);
    hydrox::GNCSetpoint heading;
    heading.heading_ref = 3.14159265358979323846;
    yaw_controller.set_mode(hydrox::GNCMode::SURFACE);
    yaw_controller.set_setpoint(heading);
    const auto rate_limited_yaw = yaw_controller.update(state, 0.01);
    failures += expect(std::abs(rate_limited_yaw[5] - 10.0) < 1.0e-9,
                       "heading outer loop respects the configured yaw-rate limit");

    hydrox::SurfaceVesselController::Params disturbance_params;
    disturbance_params.surge_kp = 100.0;
    disturbance_params.surge_ki = 50.0;
    disturbance_params.surge_integral_limit = 1.0;
    disturbance_params.yaw_kp = 100.0;
    disturbance_params.sideslip_compensation_gain = 1.0;
    disturbance_params.max_crab_angle_rad = 0.3;
    disturbance_params.waypoint_surge_mps = 3.0;
    disturbance_params.max_accel_mps2 = 1.0;
    disturbance_params.max_decel_mps2 = 1.0;
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

    hydrox::SurfaceVesselController::Params limited_params = params;
    limited_params.surge_kp = 500.0;
    limited_params.surge_kd = 65.0;
    limited_params.surge_feedback_force_limit_N = 180.0;
    hydrox::SurfaceVesselController limited_controller(limited_params);
    hydrox::GNCSetpoint recovery;
    recovery.surge_ref = 0.5;
    limited_controller.set_mode(hydrox::GNCMode::SURFACE);
    limited_controller.set_setpoint(recovery);
    auto reverse_drift = hydrox::NavigationState::zeros();
    reverse_drift.nu[0] = -2.0;
    const auto bounded_recovery = limited_controller.update(reverse_drift, 0.1);
    failures += expect(std::abs(bounded_recovery[0] - 180.0) < 1.0e-9,
                       "surface station recovery respects the inertial force limit");

    hydrox::SurfaceVesselController::Params damping_params = params;
    damping_params.surge_kp = 100.0;
    damping_params.surge_kd = 50.0;
    damping_params.surge_accel_filter_tau_s = 0.001;
    hydrox::SurfaceVesselController damping_controller(damping_params);
    hydrox::GNCSetpoint constant_speed;
    constant_speed.surge_ref = 1.0;
    damping_controller.set_mode(hydrox::GNCMode::SURFACE);
    damping_controller.set_setpoint(constant_speed);
    auto accelerating = hydrox::NavigationState::zeros();
    (void)damping_controller.update(accelerating, 0.1);
    accelerating.nu[0] = 0.5;
    const auto acceleration_damped = damping_controller.update(accelerating, 0.1);
    failures += expect(acceleration_damped[0] < 0.0,
                       "surface derivative term opposes measured surge acceleration");

    if (failures == 0)
        std::cout << "test_surface_controller: all checks passed\n";
    return failures == 0 ? 0 : 1;
}
