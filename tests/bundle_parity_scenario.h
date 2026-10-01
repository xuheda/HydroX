#pragma once
#include "gnc/control_factory.h"
#include <cmath>
#include <vector>
namespace hydrox::test
{
inline std::vector<double> bundle_parity_sequence(ControlStack stack)
{
    std::vector<double> result;
    for (int mode : {1, 2, 3})
    {
        NavigationState state = NavigationState::zeros();
        state.depth_m = 12.0;
        state.eta[2] = 12.0;
        state.nu[0] = 1.0;
        stack.controller->reset(state);
        stack.controller->set_mode(static_cast<GNCMode>(mode));
        for (int step = 0; step < 32; ++step)
        {
            const double t = step * 0.02;
            state.eta[0] = 0.7 * t;
            state.eta[1] = 0.2 * t;
            state.eta[3] = 0.03 * std::sin(t);
            state.eta[4] = 0.04 * std::cos(t);
            state.eta[5] = 0.1 + t * 0.02;
            state.nu[0] = 1.0 + t;
            state.nu[4] = 0.01;
            state.nu[5] = 0.02;
            GNCSetpoint sp{};
            sp.depth_ref = 13.0;
            sp.heading_ref = 0.4;
            sp.surge_ref = 1.5;
            sp.wp_n = 20.0;
            sp.wp_e = 5.0;
            sp.wp_d = 13.0;
            sp.use_yaw_rate_ref = step >= 16;
            sp.yaw_rate_ref = -0.08;
            stack.controller->set_setpoint(sp);
            const auto tau = stack.controller->update(state, 0.02);
            const auto cmd = stack.allocator->allocate(tau, state.nu[0]);
            for (int axis = 0; axis < 6; ++axis)
                result.push_back(tau[axis]);
            for (float channel : cmd.ch)
                result.push_back(channel);
        }
    }
    return result;
}
} // namespace hydrox::test
