#include "gnc/fixedwing_controller.h"

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
    hydrox::FixedWingController::Params params;
    params.pitch_trim = 0.0;
    hydrox::FixedWingController controller(params);
    hydrox::GNCSetpoint waypoint;
    waypoint.wp_n = 100.0;
    waypoint.wp_d = -45.0;
    waypoint.surge_ref = 8.0;
    controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);
    controller.set_setpoint(waypoint);

    auto state = hydrox::NavigationState::zeros();
    state.eta[2] = -45.0;
    state.eta[3] = 0.2;
    state.nu[3] = 0.4;
    state.eta[4] = 0.1;
    state.nu[4] = 0.3;
    const auto tau = controller.update(state, 0.01);

    int failures = 0;
    failures += expect(tau[3] < 0.0,
                       "fixed wing roll loop counters positive roll and roll rate");
    failures += expect(tau[4] < 0.0,
                       "fixed wing pitch loop counters positive pitch and pitch rate");
    failures += expect(tau[0] == 8.0,
                       "fixed wing preserves the requested cruise speed");

    // A trimmed airframe can have a persistent altitude bias.  The bounded
    // outer-loop integral must add a climb demand when the aircraft is below
    // its commanded NED altitude, and reset cleanly between missions.
    hydrox::FixedWingController::Params altitude_params;
    altitude_params.altitude_kp = 0.0;
    altitude_params.altitude_kd = 0.0;
    altitude_params.altitude_ki = 1.0;
    altitude_params.altitude_integral_limit = 0.1;
    altitude_params.pitch_limit_rad = 0.5;
    altitude_params.pitch_trim = 0.0;
    altitude_params.pitch_attitude_kp = 1.0;
    altitude_params.max_pitch_command = 0.5;
    hydrox::FixedWingController altitude_controller(altitude_params);
    altitude_controller.set_mode(hydrox::GNCMode::WAYPOINT_3D);
    auto altitude_waypoint = waypoint;
    altitude_waypoint.wp_d = -10.0;
    altitude_controller.set_setpoint(altitude_waypoint);
    auto altitude_state = hydrox::NavigationState::zeros();
    altitude_state.eta[2] = 0.0;
    const auto biased_tau = altitude_controller.update(altitude_state, 1.0);
    failures += expect(biased_tau[4] > 0.0,
                       "altitude integral adds a climb demand for negative NED depth error");
    altitude_controller.reset(altitude_state);
    altitude_waypoint.wp_d = 0.0;
    altitude_controller.set_setpoint(altitude_waypoint);
    const auto reset_tau = altitude_controller.update(altitude_state, 0.01);
    failures += expect(reset_tau[4] == 0.0,
                       "altitude integral resets between fixed-wing missions");
    if (failures == 0)
        std::cout << "test_fixedwing_controller: all checks passed\n";
    return failures == 0 ? 0 : 1;
}
