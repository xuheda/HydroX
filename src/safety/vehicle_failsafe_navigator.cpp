#include "hydrox/safety/failsafe_navigator.h"
#include "hydrox/runtime/hil_contract.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace hydrox::safety
{
VehicleFailsafeNavigator::VehicleFailsafeNavigator(const SafetyProfile &profile) noexcept
    : profile_(profile) { reset(); }

void VehicleFailsafeNavigator::reset() noexcept
{
    mode_ = VehicleMode::Boot;
    reference_ = GNCSetpoint{};
    initialized_ = position_available_ = vtol_cruise_ = false;
    last_update_us_ = sequence_ = 0;
}

double VehicleFailsafeNavigator::slew(double value, double target, double rate, double dt) noexcept
{ return value + std::clamp(target - value, -rate * dt, rate * dt); }

bool VehicleFailsafeNavigator::supported(VehicleMode mode) noexcept
{
    return mode == VehicleMode::CommandHold || mode == VehicleMode::FailsafeStabilize ||
           mode == VehicleMode::FailsafeHold || mode == VehicleMode::FailsafeSurface ||
           mode == VehicleMode::EmergencyAbort;
}

void VehicleFailsafeNavigator::capture_position(const NavigationState &state) noexcept
{
    north_ = state.eta[0]; east_ = state.eta[1];
    // Clockwise circle tangent to the entry course, rather than flying back home.
    center_n_ = north_ - profile_.loiter_radius_m * std::sin(heading_);
    center_e_ = east_ + profile_.loiter_radius_m * std::cos(heading_);
}

bool VehicleFailsafeNavigator::enter(VehicleMode mode, const NavigationState &state,
    const AuthorizedReference *last, uint64_t now_us, const FailsafeContext &context) noexcept
{
    if (!profile_.valid() || !supported(mode) || !state.eta.allFinite() ||
        !state.nu.allFinite() || !std::isfinite(state.depth_m) ||
        (mode == VehicleMode::FailsafeSurface && !profile_.underwater()))
    { initialized_ = false; return false; }
    if (!initialized_)
    {
        heading_ = state.heading_rad(); depth_ = state.depth_m;
        surge_ = state.surge();
        if (last && last->valid && std::isfinite(last->setpoint.surge_ref))
            surge_ = last->setpoint.surge_ref;
        capture_position(state);
        position_available_ = context.position_available;
        vtol_cruise_ = context.flight_phase == FlightPhase::Cruise;
    }
    mode_ = mode; last_update_us_ = now_us; surface_depth_ = state.depth_m;
    reference_ = GNCSetpoint{};
    // Only a fin AUV briefly holds its last command. Other domains capture a
    // stop/hold/loiter immediately, never reuse an old descent/waypoint target.
    if (mode == VehicleMode::CommandHold && profile_.vehicle_kind == VehicleSafetyKind::FinAuv)
    {
        if (!last || !last->valid || !runtime::valid_gnc_setpoint(last->setpoint, last->mode))
        { initialized_ = false; return false; }
        reference_ = last->setpoint;
        surge_ = reference_.surge_ref;
    }
    initialized_ = true;
    return true;
}

ControlCandidate VehicleFailsafeNavigator::update(VehicleMode mode, const NavigationState &state,
    uint64_t now_us, const FailsafeContext &context) noexcept
{
    ControlCandidate out{};
    out.source = mode == VehicleMode::EmergencyAbort ? ControlSource::EmergencyAbort : ControlSource::Failsafe;
    if (!initialized_ || mode != mode_ || !supported(mode) || !state.eta.allFinite() ||
        !state.nu.allFinite() || !std::isfinite(state.depth_m)) return out;
    const bool vertical_required = profile_.vehicle_class != VehicleClass::USV &&
                                   profile_.vehicle_class != VehicleClass::UGV_DIFFERENTIAL;
    if (vertical_required && !context.vertical_available) return out;
    const double dt = now_us > last_update_us_
        ? std::min(0.2, (now_us - last_update_us_) * 1.e-6) : 0.0;
    last_update_us_ = now_us;
    if (context.position_available && !position_available_)
    {
        heading_ = state.heading_rad();
        capture_position(state); // no jump back to an old point after navigation recovery
    }
    position_available_ = context.position_available;
    GNCMode command_mode = GNCMode::DEPTH_HOLD;
    if (mode == VehicleMode::CommandHold && profile_.vehicle_kind == VehicleSafetyKind::FinAuv)
    {
        if (reference_.use_yaw_rate_ref)
        {
            reference_.yaw_rate_ref = slew(reference_.yaw_rate_ref, 0.0, profile_.yaw_rate_slew_radps2, dt);
            reference_.heading_ref = state.heading_rad();
            if (std::abs(reference_.yaw_rate_ref) < 1.e-6)
            { reference_.use_yaw_rate_ref = false; heading_ = state.heading_rad(); }
        }
    }
    else
    {
        reference_ = GNCSetpoint{};
        reference_.depth_ref = depth_; reference_.wp_d = depth_;
        reference_.heading_ref = heading_; reference_.hold_heading = true;
        reference_.wp_n = north_; reference_.wp_e = east_;
        reference_.arrival_radius_m = profile_.station_radius_m;
        const bool surfacing = profile_.underwater() &&
            (mode == VehicleMode::FailsafeSurface || mode == VehicleMode::EmergencyAbort);
        switch (profile_.vehicle_kind)
        {
        case VehicleSafetyKind::FinAuv:
        case VehicleSafetyKind::ThrusterUuv:
        {
            const bool fins = profile_.vehicle_kind == VehicleSafetyKind::FinAuv;
            const double target_speed = surfacing ? profile_.surface_surge_mps : profile_.safe_surge_mps;
            surge_ = slew(surge_, target_speed, profile_.surge_slew_mps2, dt);
            reference_.surge_ref = fins ? std::max(profile_.minimum_control_surge_mps, surge_) : surge_;
            if (surfacing)
            {
                // Never command a deeper target if already shallower than capture depth.
                surface_depth_ = std::min(surface_depth_, std::max(profile_.surface_capture_depth_m,
                    surface_depth_ - profile_.surface_reference_rate_mps * dt));
                reference_.depth_ref = reference_.wp_d = surface_depth_;
                if (fins && state.depth_m <= profile_.surface_capture_depth_m)
                { command_mode = GNCMode::SURFACE; reference_.surge_ref = 0.0; }
            }
            // ROVs keep vertical control near the surface (SURFACE disables heave).
            if (!fins && profile_.position_hold_capable && context.position_available)
                command_mode = GNCMode::DP;
            break;
        }
        case VehicleSafetyKind::SurfaceVessel:
            surge_ = slew(surge_, 0.0, profile_.surge_slew_mps2, dt);
            reference_.surge_ref = surge_;
            if (std::abs(surge_) < 0.01 && context.position_available && profile_.position_hold_capable)
                command_mode = GNCMode::WAYPOINT_3D;
            break;
        case VehicleSafetyKind::Ground:
            surge_ = slew(surge_, 0.0, profile_.surge_slew_mps2, dt);
            reference_.surge_ref = surge_;
            reference_.use_yaw_rate_ref = true;
            reference_.yaw_rate_ref = 0.0;
            break;
        case VehicleSafetyKind::Multirotor:
            command_mode = context.position_available ? GNCMode::WAYPOINT_3D : GNCMode::DEPTH_HOLD;
            break;
        case VehicleSafetyKind::Vtol:
            // Front/back transition on entry recovers to hover. Cruise stays
            // wing-borne unless the aircraft controller itself loses airspeed.
            if (vtol_cruise_ && context.flight_phase != FlightPhase::Cruise)
                vtol_cruise_ = false;
            if (!vtol_cruise_)
            {
                command_mode = context.position_available ? GNCMode::WAYPOINT_3D : GNCMode::DEPTH_HOLD;
                break;
            }
            [[fallthrough]];
        case VehicleSafetyKind::FixedWing:
        {
            reference_.surge_ref = profile_.loiter_speed_mps;
            if (context.position_available)
            {
                const double theta = std::atan2(state.eta[1] - center_e_, state.eta[0] - center_n_);
                const double lead = std::clamp(profile_.loiter_speed_mps * 3.0 / profile_.loiter_radius_m, 0.15, 0.6);
                reference_.wp_n = center_n_ + profile_.loiter_radius_m * std::cos(theta + lead);
                reference_.wp_e = center_e_ + profile_.loiter_radius_m * std::sin(theta + lead);
                reference_.hold_heading = false;
                command_mode = GNCMode::WAYPOINT_3D;
            }
            else if (profile_.vehicle_kind == VehicleSafetyKind::Vtol)
            {
                // No position fix: altitude/course hold, not a fictitious geographic
                // loiter and not an unintended request to transition to hover.
                reference_.wp_n = state.eta[0] + 100.0 * std::cos(heading_);
                reference_.wp_e = state.eta[1] + 100.0 * std::sin(heading_);
                reference_.hold_heading = false;
                command_mode = GNCMode::WAYPOINT_3D;
            }
            break;
        }
        }
    }
    if (!profile_.accepts_mode(command_mode) || !runtime::valid_gnc_setpoint(reference_, command_mode)) return out;
    out.valid = true; out.mode = command_mode; out.setpoint = reference_;
    out.sequence = ++sequence_; out.received_at_us = now_us;
    out.valid_until_us = now_us > std::numeric_limits<uint64_t>::max() - profile_.internal_reference_valid_us
        ? std::numeric_limits<uint64_t>::max() : now_us + profile_.internal_reference_valid_us;
    return out;
}
}
