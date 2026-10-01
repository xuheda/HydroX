#include "hydrox/runtime/ecaa9_diagnostic_mission.h"

#include <cassert>
#include <cmath>

namespace
{
bool near(double lhs, double rhs, double tolerance = 1.0e-6)
{
    return std::abs(lhs - rhs) <= tolerance;
}
}

int main()
{
    hydrox::runtime::EcaA9DiagnosticMission mission;

    const hydrox::runtime::MissionGuidanceState initial{-120.0, -30.0, 100.0, 0.0};
    const hydrox::GNCSetpoint straight = mission.update(initial);
    assert(mission.waypoint_index() == 0);
    assert(near(straight.heading_ref, 0.0));
    assert(near(straight.yaw_rate_ref, 0.0));
    assert(near(straight.surge_ref, 2.0));
    assert(near(straight.depth_ref, 100.0));
    assert(straight.use_yaw_rate_ref);
    assert(straight.use_path_segment);
    assert(near(straight.lookahead_m, 15.0));

    const hydrox::runtime::MissionGuidanceState first_arrival{0.0, -30.0, 100.0, 0.0};
    const hydrox::GNCSetpoint turning = mission.update(first_arrival);
    assert(mission.waypoint_index() == 1);
    assert(near(turning.heading_ref, 3.14159265358979323846 / 4.0));
    assert(near(turning.yaw_rate_ref, 0.35));
    assert(turning.surge_ref > 1.7 && turning.surge_ref < 1.8);
    assert(turning.depth_ref > 101.3 && turning.depth_ref < 101.4);
    assert(near(turning.path_start_n, 0.0));
    assert(near(turning.path_start_e, -30.0));

    mission.reset();
    const hydrox::runtime::MissionGuidanceState reversed{-120.0, -30.0, 100.0,
                                                  3.14159265358979323846};
    const hydrox::GNCSetpoint hard_turn = mission.update(reversed);
    assert(near(hard_turn.yaw_rate_ref, -0.35));
    assert(near(hard_turn.surge_ref, 1.2));
    return 0;
}
