#include "hydrox/runtime/hil_runtime.h"
#include "gnc/control_factory.h"
#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace hydrox;
using namespace hydrox::safety;
using namespace hydrox::runtime;

static void check(bool value, const char *message)
{
    if (!value) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}

static ControlParameters parameters(VehicleArchetype kind, VehicleClass domain)
{
    ControlParameters p{}; p.valid = true; p.archetype = kind; p.vehicle_class = domain;
    return p;
}

static void test_domain_references()
{
    const VehicleArchetype kinds[] = {VehicleArchetype::SlenderBodyFin, VehicleArchetype::Thruster,
        VehicleArchetype::Surface, VehicleArchetype::Multirotor, VehicleArchetype::FixedWing,
        VehicleArchetype::VTOL, VehicleArchetype::DifferentialDrive};
    const VehicleClass domains[] = {VehicleClass::UUV, VehicleClass::UUV, VehicleClass::USV,
        VehicleClass::UAV_MULTIROTOR, VehicleClass::UAV_FIXED_WING, VehicleClass::UAV_VTOL,
        VehicleClass::UGV_DIFFERENTIAL};
    for (int i = 0; i < 7; ++i)
    {
        const auto p = parameters(kinds[i], domains[i]);
        const auto profile = safety_profile_for(p);
        check(profile.valid(), "every physical family selects a valid profile");
        VehicleSupervisor supervisor(profile);
        SupervisorInput in{}; in.health.can_arm = in.health.control_available = true;
        in.boot_complete = true; supervisor.update(in);
        in.arm_requested = true; supervisor.update(in); in.arm_requested = false;
        in.external_control_requested = in.external_command_valid = true;
        in.external_command_age_us = 0; supervisor.update(in);
        in.now_us = 210'000; in.external_command_age_us = in.now_us;
        check(supervisor.update(in).mode == VehicleMode::CommandHold, "all families enter short hold");
        in.now_us = 510'000; in.external_command_age_us = in.now_us;
        check(supervisor.update(in).mode == VehicleMode::FailsafeStabilize, "all families latch on loss");
        in.now_us = 3'600'000; in.external_command_age_us = in.now_us;
        const auto terminal = supervisor.update(in).mode;
        check(terminal == (i < 2 ? VehicleMode::FailsafeSurface : VehicleMode::FailsafeHold),
              "only underwater vehicles escalate to surfacing");
        in.external_command_age_us = 0;
        check(supervisor.update(in).mode == terminal, "fresh packets cannot take over");
        in.resume_requested = true;
        check(supervisor.update(in).mode == VehicleMode::ExternalControl, "deliberate healthy takeover without disarming");

        auto state = NavigationState::zeros();
        state.eta[0] = 100.0; state.eta[1] = 200.0;
        state.depth_m = state.eta[2] = i < 2 ? 30.0 : -40.0;
        state.nu[0] = 2.0;
        FailsafeContext context{true, true, i == 5 ? FlightPhase::Hover : FlightPhase::NotApplicable};
        VehicleFailsafeNavigator nav(profile);
        check(nav.enter(terminal, state, nullptr, 0, context), "enter per-domain fallback");
        ControlCandidate result{};
        for (uint64_t t = 100'000; t <= 20'000'000; t += 100'000)
        {
            result = nav.update(terminal, state, t, context);
            check(result.valid, "bounded fallback continues generating references");
            if (i >= 3 && i <= 5)
                check(result.mode != GNCMode::SURFACE && result.setpoint.depth_ref == -40.0 &&
                    result.setpoint.wp_d == -40.0, "aircraft preserve altitude, never descend to sea datum");
        }
        if (i < 2)
        {
            check(result.setpoint.depth_ref < 30.0 && result.setpoint.depth_ref > 20.0,
                  "both UUV types ascend through a rate-bounded depth reference");
            check(i == 0 ? result.setpoint.surge_ref >= profile.minimum_control_surge_mps
                         : result.setpoint.surge_ref == 0.0, "fin authority versus direct-heave ROV");
            if (i == 1)
            {
                ThrusterVehicleController rov;
                rov.set_mode(result.mode); rov.set_setpoint(result.setpoint);
                check(rov.update(state, 0.1)[2] < 0.0, "ROV surfacing actually commands upward heave thrust");
            }
        }
        if (i == 2) check(result.mode == GNCMode::WAYPOINT_3D && result.setpoint.hold_heading,
                          "USV stops then station-keeps");
        if (i == 3 || i == 5) check(result.mode == GNCMode::WAYPOINT_3D && result.setpoint.wp_n == 100.0 &&
            result.setpoint.wp_e == 200.0 && result.setpoint.surge_ref == 0.0, "hover retains captured point");
        if (i == 4) check(result.mode == GNCMode::WAYPOINT_3D && !result.setpoint.hold_heading &&
            result.setpoint.surge_ref >= p.fixedwing_gnc.min_flight_speed_mps, "fixed wing has a flying loiter, not zero-speed hold");
        if (i == 6) check(result.setpoint.surge_ref == 0.0 && result.setpoint.use_yaw_rate_ref &&
                          result.setpoint.yaw_rate_ref == 0.0, "ground vehicle stops without turning on the spot");
        if (i >= 3 && i <= 5)
        {
            context.position_available = false;
            result = nav.update(terminal, state, 20'100'000, context);
            check(result.valid && result.mode == GNCMode::DEPTH_HOLD,
                  "lost position keeps height/course, not a fictitious geographic hold");
            check(!nav.enter(VehicleMode::FailsafeSurface, state, nullptr, 0, context),
                  "aircraft reject surfacing even if supervisor is misconfigured");
        }
    }
    auto invalid = safety_profile_for(parameters(VehicleArchetype::Thruster, VehicleClass::UAV_MULTIROTOR));
    check(!invalid.valid(), "cross-domain profile mismatch is not accepted");
}

static void test_vtol_phase_handoff()
{
    auto p = parameters(VehicleArchetype::VTOL, VehicleClass::UAV_VTOL);
    VtolController controller(p.vtol_gnc);
    auto state = NavigationState::zeros(); state.depth_m = state.eta[2] = -40.0;
    state.airspeed_valid = true; state.equivalent_airspeed_mps = 12.0; state.nu[0] = 12.0;
    controller.reset(state); controller.set_mode(GNCMode::WAYPOINT_3D);
    GNCSetpoint sp{}; sp.wp_n = 500.0; sp.wp_d = -40.0; sp.surge_ref = 12.0;
    controller.set_setpoint(sp);
    for (int i = 0; i < 10; ++i) controller.update(state, 0.1);
    check(controller.flight_phase() == FlightPhase::Cruise, "VTOL fixture reaches actual cruise");
    controller.set_mode(GNCMode::DEPTH_HOLD);
    check(controller.flight_phase() == FlightPhase::Cruise, "setpoint mode change alone must not reset phase");
    controller.set_mode(GNCMode::WAYPOINT_3D);
    FailsafeContext context{true, true, controller.flight_phase()};
    VehicleFailsafeNavigator nav(safety_profile_for(p));
    check(nav.enter(VehicleMode::FailsafeStabilize, state, nullptr, 0, context), "capture cruise fallback");
    auto cmd = nav.update(VehicleMode::FailsafeStabilize, state, 100'000, context);
    controller.set_mode(cmd.mode); controller.set_setpoint(cmd.setpoint);
    check(controller.update(state, 0.1).allFinite() && controller.flight_phase() == FlightPhase::Cruise,
          "cruise fallback stays wing-borne with finite control");
    context.position_available = false;
    cmd = nav.update(VehicleMode::FailsafeStabilize, state, 200'000, context);
    controller.set_mode(cmd.mode); controller.set_setpoint(cmd.setpoint); controller.update(state, 0.1);
    check(controller.flight_phase() == FlightPhase::Cruise, "GPS loss alone must not force back transition");
    state.airspeed_valid = false; controller.update(state, 0.1);
    context.flight_phase = controller.flight_phase();
    cmd = nav.update(VehicleMode::FailsafeStabilize, state, 300'000, context);
    check(cmd.setpoint.hold_heading && cmd.setpoint.surge_ref == 0.0,
          "pitot loss commits recovery, no oscillation back into cruise");

    controller.reset(state); state.airspeed_valid = true; state.equivalent_airspeed_mps = 0.0;
    state.nu[0] = 0.0; controller.set_mode(GNCMode::WAYPOINT_3D); controller.set_setpoint(sp);
    for (int i = 0; i < 160; ++i) controller.update(state, 0.1);
    check(controller.transition_fault(), "front transition cannot wait forever for airspeed");
    check(controller.flight_phase() == FlightPhase::Hover, "failed transition recovers to hover after slowing");
    check(controller.acknowledge_transition_fault(), "transition latch can be acknowledged after recovery");

    controller.reset(state); controller.set_setpoint(sp); controller.update(state, 0.1);
    state.depth_m += 4.0; controller.update(state, 0.1);
    check(controller.transition_fault() && controller.flight_phase() == FlightPhase::BackTransition,
          "altitude loss aborts front transition");
}

static NavigationInput aerial_sensor(uint64_t t, bool gps = true)
{
    NavigationInput in{}; in.got_imu = true; in.imu.time_usec = t;
    in.measurements.depth.meta.valid = true; in.measurements.depth.value = -15.0;
    in.measurements.depth.meta.source = NavMeasurementSource::Barometer;
    in.measurements.depth.variance = 0.01;
    if (gps)
    {
        GPSMeasurement g{}; g.pos_d = -15.0; g.has_altitude = true; g.timestamp = t * 1.e-6;
        in.gps = g; in.gps_valid = true;
        in.measurements.gps_position_ned.meta.valid = true;
        in.measurements.gps_position_ned.covariance = Eigen::Matrix3d::Identity();
    }
    return in;
}

static void test_air_runtime()
{
    auto p = parameters(VehicleArchetype::Multirotor, VehicleClass::UAV_MULTIROTOR);
    p.max_total_lift_N = 100.0;
    auto stack = build_control_stack(p);
    HilRuntimeConfig cfg{}; cfg.estimation_profile = estimation_profile_for(p.vehicle_class);
    cfg.safety_profile = safety_profile_for(p);
    cfg.initial_state.depth_m = cfg.initial_state.eta[2] = -15.0;
    cfg.sensor_timeout_us = 500'000;
    HilRuntime rt(cfg, std::move(stack.controller), std::move(stack.allocator));
    check(rt.valid(), "air runtime accepts matched profile");
    rt.on_connected(1'000'000);
    rt.step(aerial_sensor(1'010'000), 1'010'000);
    GNCSetpoint sp{}; sp.wp_n = 50.0; sp.wp_d = -15.0; sp.depth_ref = -15.0; sp.surge_ref = 2.0;
    check(!rt.accept_setpoint(sp, GNCMode::SURFACE, 1'020'000), "air runtime rejects even external SURFACE commands");
    check(rt.accept_setpoint(sp, GNCMode::WAYPOINT_3D, 1'020'000), "air external waypoint accepted");
    for (uint64_t t = 1'030'000; t < 5'100'000; t += 10'000)
    {
        check(rt.step(aerial_sensor(t), t) == StepStatus::OK, "air safety step succeeds");
        check(rt.last_tick().actuator_authorized && rt.last_tick().actuator.valid(), "safety handoff keeps bounded lift commands");
        check(rt.mode() != GNCMode::SURFACE && rt.setpoint().wp_d < 0.0, "runtime never substitutes underwater altitude");
    }
    check(rt.safety_status().mode == VehicleMode::FailsafeHold, "air runtime reaches latched hold");
    sp.wp_n = 500.0;
    check(rt.accept_setpoint(sp, GNCMode::WAYPOINT_3D, 5'100'000), "new candidate can be buffered");
    check(rt.setpoint().wp_n != 500.0, "buffered command cannot mutate effective controller target");
    rt.step(aerial_sensor(5'110'000), 5'110'000);
    check(rt.safety_status().mode == VehicleMode::FailsafeHold, "recovered packet does not unlatch");
    check(rt.resume_external_control(5'120'000), "explicit live takeover does not require stopping motors");
    rt.step(aerial_sensor(5'130'000), 5'130'000);
    check(rt.last_tick().actuator_authorized && rt.setpoint().wp_n == 500.0, "deliberate takeover reaches arbiter");
    for (uint64_t t = 5'140'000; t < 9'200'000; t += 10'000)
    {
        if (t % 100'000 == 0) rt.accept_setpoint(sp, GNCMode::WAYPOINT_3D, t);
        rt.step(aerial_sensor(t, false), t);
    }
    check(rt.last_tick().safety_navigation_degraded && rt.mode() == GNCMode::DEPTH_HOLD &&
          rt.last_tick().actuator_authorized, "GPS loss preserves lift/height with degraded navigation");
    check(!rt.resume_external_control(9'200'000), "cannot resume a geographic mission without position");
    check(std::string(rt.last_tick().safety_cause) == "PositionAidUnavailable",
          "runtime health aggregation retains position-loss diagnosis");
    auto missing_vertical = aerial_sensor(10'300'000, false);
    missing_vertical.measurements.depth.meta.valid = false;
    // Advance using normal sensor cadence, but remove every vertical aid.
    for (uint64_t t = 9'200'000; t <= 10'400'000; t += 10'000)
    {
        missing_vertical.imu.time_usec = t;
        rt.step(missing_vertical, t);
    }
    check(rt.safety_status().mode == VehicleMode::OutputDisabled && !rt.last_tick().actuator_authorized,
          "unobservable vertical state must not claim a successful hold");
    check(rt.mission_state() == MissionState::FAILED, "hard protection must not be mislabeled idle");
    check(rt.safety_status().reason == SafetyReason::ControlCoreUnavailable &&
          std::string(rt.last_tick().safety_cause) == "VerticalAidUnavailable",
          "fatal vertical loss wins over simultaneous position loss in the shared health path");
    check(!rt.resume_external_control(10'410'000), "continue cannot clear a hard output-inhibition fault");
}

static void test_vtol_runtime_handoff()
{
    auto p = parameters(VehicleArchetype::VTOL, VehicleClass::UAV_VTOL);
    auto controller = std::make_unique<VtolController>(p.vtol_gnc);
    auto *probe = controller.get();
    HilRuntimeConfig cfg{}; cfg.estimation_profile = estimation_profile_for(p.vehicle_class);
    cfg.safety_profile = safety_profile_for(p);
    // Isolate controller handoff from estimator convergence, explicitly labeled
    // truth-debug (the multirotor test above exercises estimated-state feedback).
    cfg.control_feedback_source = ControlFeedbackSource::TruthDebug;
    cfg.initial_state.depth_m = cfg.initial_state.eta[2] = -40.0;
    HilRuntime rt(cfg, std::move(controller), std::make_unique<VtolAllocator>(p.vtol_allocator));
    rt.on_connected(1'000'000);
    GNCSetpoint sp{}; sp.wp_n = 500.0; sp.wp_d = -40.0; sp.surge_ref = 12.0;
    for (uint64_t t = 1'010'000; t <= 6'000'000; t += 10'000)
    {
        auto in = aerial_sensor(t);
        in.truth_valid = true; in.truth.valid = true;
        in.truth.eta[2] = -40.0; in.truth.nu[0] = 12.0;
        in.measurements.airspeed.meta.valid = true; in.measurements.airspeed.value = 12.0;
        rt.step(in, t);
        if (t < 2'000'000) rt.accept_setpoint(sp, GNCMode::WAYPOINT_3D, t + 1);
        if (t > 1'700'000)
            check(probe->flight_phase() == FlightPhase::Cruise,
                  "runtime authority changes do not reset actual VTOL cruise to hover");
    }
    check(rt.safety_status().mode == VehicleMode::FailsafeHold && rt.last_tick().actuator_authorized,
          "VTOL runtime keeps active cruise loiter after sustained loss");
}

int main()
{
    test_domain_references(); test_vtol_phase_handoff(); test_air_runtime(); test_vtol_runtime_handoff();
    std::cout << "PASS: seven-family failsafe references, VTOL recovery and airborne runtime takeover\n";
}
