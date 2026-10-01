#include "gnc/control_allocator.h"
#include "gnc/gnc_controller.h"
#include "gnc/surge_p.h"

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

    hydrox::SlenderBodyAUVController controller;
    auto state = hydrox::NavigationState::zeros();
    state.depth_m = 10.0;
    state.eta[2] = 10.0;
    controller.reset(state);
    controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);

    hydrox::GNCSetpoint waypoint;
    waypoint.depth_ref = 20.0;
    waypoint.heading_ref = 0.5;
    waypoint.surge_ref = 1.5;
    controller.set_setpoint(waypoint);

    hydrox::Wrench dive = hydrox::Wrench::Zero();
    for (int i = 0; i < 20; ++i)
        dive = controller.update(state, 0.1);
    failures += expect(dive[0] > 0.0,
                       "slender AUV accelerates forward toward cruise speed");
    failures += expect(std::abs(dive[2]) < 1.0e-12,
                       "slender AUV never claims direct heave authority");
    failures += expect(dive[4] < 0.0,
                       "deeper depth demand creates nose-down pitch moment");
    failures += expect(dive[5] > 0.0,
                       "positive heading error creates positive NED yaw moment");

    waypoint.use_yaw_rate_ref = true;
    waypoint.yaw_rate_ref = 0.2;
    controller.set_setpoint(waypoint);
    const auto yaw_rate = controller.update(state, 0.1);
    failures += expect(yaw_rate[5] > 0.0,
                       "positive yaw-rate command creates positive yaw moment");

    controller.set_mode(hydrox::GNCMode::SURFACE);
    controller.set_setpoint(waypoint);
    const auto surface = controller.update(state, 0.1);
    failures += expect(std::abs(surface[2]) < 1.0e-12 &&
                           std::abs(surface[4]) < 1.0e-12,
                       "surface mode suppresses depth and pitch actuation");

    hydrox::FinAllocator::Params allocator_params;
    allocator_params.S_fin = 0.0064;
    allocator_params.CL_s = 3.0;
    allocator_params.CL_r = 3.0;
    allocator_params.x_fin = 0.4;
    allocator_params.D_prop = 0.14;
    allocator_params.KT_0 = 0.4566;
    allocator_params.n_max_rpm = 3000.0;
    allocator_params.max_thrust_N = 50.0;
    allocator_params.delta_max_deg = 25.0;
    allocator_params.fin_angles_deg = {270.0, 0.0, 90.0, 180.0};
    hydrox::FinAllocator allocator(allocator_params);
    hydrox::Wrench requested = hydrox::Wrench::Zero();
    requested[0] = 10.0;
    requested[4] = -5.0;
    hydrox::SurgeP::Params surge_params;
    surge_params.kp = 25.0;
    surge_params.ki = 4.0;
    surge_params.drag_ff = 3.0;
    surge_params.tau_max = 50.0;
    surge_params.forward_min_tau = 5.0;
    surge_params.tau_rate_limit = 30.0;
    hydrox::SurgeP shaped_surge(surge_params);
    double previous_tau = 0.0;
    double shaped_tau = 0.0;
    bool bounded_and_smooth = true;
    for (int i = 0; i < 12; ++i)
    {
        shaped_tau = shaped_surge.step(0.0, 2.0, 0.1);
        bounded_and_smooth =
            bounded_and_smooth && shaped_tau >= 0.0 && shaped_tau <= 50.0 &&
            std::abs(shaped_tau - previous_tau) <= 3.0 + 1.0e-12;
        previous_tau = shaped_tau;
    }
    const double overspeed_tau = shaped_surge.step(2.5, 2.0, 0.1);
    bounded_and_smooth = bounded_and_smooth && overspeed_tau >= 0.0 &&
        std::abs(overspeed_tau - previous_tau) <= 3.0 + 1.0e-12;
    failures += expect(bounded_and_smooth,
                       "LAUV surge loop stays one-way, authority-bounded and slew-limited");
    auto turn_floor_params = surge_params;
    turn_floor_params.tau_rate_limit = 0.0;
    hydrox::SurgeP turn_floor_surge(turn_floor_params);
    const double turn_floor_tau = turn_floor_surge.step(2.1, 1.2, 0.1);
    failures += expect(turn_floor_tau >= 5.0,
                       "positive AUV turn-speed reference preserves propeller wash");
    const auto pitch_actuators = allocator.allocate(requested, 2.0);
    requested[4] = 0.0;
    requested[5] = 5.0;
    const auto yaw_actuators = allocator.allocate(requested, 2.0);
    requested[4] = -5.0;
    const auto actuators = allocator.allocate(requested, 2.0);
    failures += expect(actuators.ch[4] > 0.0f,
                       "slender AUV positive surge drives the stern propeller");
    failures += expect(std::abs(pitch_actuators.ch[0]) < 1.0e-6f &&
                           pitch_actuators.ch[1] > 0.0f &&
                           std::abs(pitch_actuators.ch[2]) < 1.0e-6f &&
                           pitch_actuators.ch[3] < 0.0f,
                       "LAUV nose-down moment moves only its starboard and port fins");
    failures += expect(yaw_actuators.ch[0] > 0.0f &&
                           std::abs(yaw_actuators.ch[1]) < 1.0e-6f &&
                           yaw_actuators.ch[2] < 0.0f &&
                           std::abs(yaw_actuators.ch[3]) < 1.0e-6f,
                       "LAUV positive yaw moves only its top and bottom fins");

    auto xtail_params = allocator_params;
    xtail_params.fin_angles_deg = {315.0, 45.0, 135.0, 225.0};
    hydrox::FinAllocator xtail_allocator(xtail_params);
    requested[4] = 0.0;
    const auto xtail_yaw = xtail_allocator.allocate(requested, 2.0);
    failures += expect(xtail_yaw.ch[0] > 0.0f && xtail_yaw.ch[1] < 0.0f &&
                           xtail_yaw.ch[2] < 0.0f && xtail_yaw.ch[3] > 0.0f,
                       "ECA A9 positive yaw mixes all four X-tail fins by installation angle");

    if (failures == 0)
        std::cout << "test_slender_auv_controller: all checks passed\n";
    return failures == 0 ? 0 : 1;
}
