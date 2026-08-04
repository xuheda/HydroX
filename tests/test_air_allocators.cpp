#include "gnc/multirotor_allocator.h"
#include "gnc/vtol_controller.h"
#include "gnc/vtol_allocator.h"

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

bool approx(double a, double b, double epsilon = 1.0e-6)
{
    return std::abs(a - b) <= epsilon;
}
}

int main()
{
    int fails = 0;

    hydrox::QuadrotorAllocator::Params quad_params;
    quad_params.max_total_thrust_N = 34.19432;
    hydrox::QuadrotorAllocator quad(quad_params);
    hydrox::Wrench tau = hydrox::Wrench::Zero();
    tau[2] = 2.0 * 9.80665;
    const auto hover = quad.allocate(tau, 0.0);
    const double expected_hover = std::sqrt(tau[2] / quad_params.max_total_thrust_N);
    for (int i = 0; i < 4; ++i)
        fails += expect(approx(hover.ch[i], expected_hover, 1.0e-5),
                        "X500 hover uses the physical UE maximum lift");

    tau[3] = 0.5;
    const auto positive_roll = quad.allocate(tau, 0.0);
    fails += expect(positive_roll.ch[0] > positive_roll.ch[1] &&
                    positive_roll.ch[3] > positive_roll.ch[2],
                    "positive roll adds thrust to the left rotor pair");

    tau[3] = 0.0;
    tau[4] = 0.5;
    const auto positive_pitch = quad.allocate(tau, 0.0);
    fails += expect(positive_pitch.ch[0] > positive_pitch.ch[3] &&
                    positive_pitch.ch[1] > positive_pitch.ch[2],
                    "positive pitch adds thrust to the front rotor pair");

    tau[4] = 0.0;
    tau[5] = 0.5;
    const auto positive_yaw = quad.allocate(tau, 0.0);
    fails += expect(positive_yaw.ch[0] > positive_yaw.ch[1] &&
                    positive_yaw.ch[2] > positive_yaw.ch[3],
                    "positive yaw follows the canonical CCW/CW spin order");

    hydrox::VtolAllocator::Params vtol_params;
    vtol_params.max_total_lift_N = 180.0;
    hydrox::VtolAllocator vtol(vtol_params);
    tau.setZero();
    tau[2] = 5.0 * 9.80665;
    tau[0] = 3.0;
    tau[3] = 0.25;
    tau[4] = -0.2;
    tau[5] = 0.1;
    const auto vtol_cmd = vtol.allocate(tau, 0.0);
    fails += expect(vtol_cmd.ch[0] > vtol_cmd.ch[1],
                    "VTOL lift rotors share the quadrotor roll convention");
    fails += expect(vtol_cmd.ch[4] < 0.0f && vtol_cmd.ch[5] > 0.0f && vtol_cmd.ch[6] > 0.0f,
                    "VTOL channels 4..6 map elevator, aileron, rudder");
    fails += expect(vtol_cmd.ch[7] > static_cast<float>(vtol_params.pusher_trim),
                    "VTOL channel 7 drives the forward pusher");

    tau.setZero();
    tau[2] = 5.0 * 9.80665;
    const auto hover_cmd = vtol.allocate(tau, 0.0);
    fails += expect(std::abs(hover_cmd.ch[7]) < 1.0e-6f,
                    "VTOL hover leaves the pusher off without a speed error");

    hydrox::VtolController::Params controller_params;
    controller_params.xy_kp = 0.35;
    controller_params.xy_kd = 1.0;
    controller_params.max_tilt_rad = 0.28;
    hydrox::VtolController controller(controller_params);
    hydrox::GNCSetpoint waypoint;
    waypoint.wp_n = 8.0;
    waypoint.wp_d = -45.0;
    waypoint.surge_ref = 1.0;
    controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);
    controller.set_setpoint(waypoint);
    auto overspeed_state = hydrox::NavigationState::zeros();
    overspeed_state.eta[2] = -45.0;
    overspeed_state.nu[0] = 4.0;
    const auto braking_wrench = controller.update(overspeed_state, 0.01);
    fails += expect(braking_wrench[4] > 0.0,
                    "VTOL forward overspeed commands a nose-up braking pitch");
    fails += expect(std::abs(braking_wrench[0]) < 1.0e-9,
                    "VTOL waypoint tracking keeps the pusher disabled in lift flight");

    auto yaw_hold_state = hydrox::NavigationState::zeros();
    yaw_hold_state.eta[2] = -45.0;
    yaw_hold_state.eta[5] = 0.7;
    controller.reset(yaw_hold_state);
    const auto yaw_hold_wrench = controller.update(yaw_hold_state, 0.01);
    fails += expect(std::abs(yaw_hold_wrench[5]) < 1.0e-9,
                    "VTOL waypoint entry holds its current yaw in lift-rotor flight");

    if (fails == 0)
        std::cout << "test_air_allocators: all checks passed\n";
    return fails == 0 ? 0 : 1;
}
