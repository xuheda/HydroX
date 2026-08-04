#include "gnc/control_factory.h"
#include "hydrox/runtime/hitl_board.h"
#include "hydrox/runtime/hitl_supervisor.h"

#include <utility>

extern "C" int hydrox_hitl_main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    hydrox::runtime::HitlBoard *board = hydrox_hitl_board();
    if (board == nullptr)
        return 10;

    hydrox::runtime::HitlVehicleProfile profile;
    if (!board->load_vehicle_profile(profile) || !profile.control.valid ||
        profile.profile_id[0] == '\0' || profile.bundle_fingerprint == 0)
    {
        board->notify(
            hydrox::runtime::HitlHealthEvent::CONFIGURATION_ERROR,
            "vehicle profile unavailable, unidentified, or invalid");
        return 11;
    }

    if (profile.runtime.estimation_profile.vehicle_class !=
            profile.control.vehicle_class ||
        profile.sensors.estimation_profile.vehicle_class !=
            profile.control.vehicle_class)
    {
        board->notify(
            hydrox::runtime::HitlHealthEvent::CONFIGURATION_ERROR,
            "estimator/sensor/control vehicle classes disagree");
        return 12;
    }

    const char *profile_error = nullptr;
    if (!hydrox::runtime::validate_hitl_vehicle_profile(
            profile, profile_error))
    {
        board->notify(
            hydrox::runtime::HitlHealthEvent::CONFIGURATION_ERROR,
            profile_error != nullptr ? profile_error
                                     : "vehicle profile validation failed");
        return 13;
    }

    // The actuator and motor parameters have one authority: the board-loaded
    // vehicle profile. No HITL-only tuning fork is permitted here.
    profile.runtime.motor = profile.control.motor;
    hydrox::ControlStack stack =
        hydrox::build_control_stack(profile.control);
    hydrox::runtime::HitlSupervisor supervisor(
        *board, profile, std::move(stack));
    return supervisor.run();
}

extern "C" void HydroX_main(void)
{
    (void)hydrox_hitl_main(0, nullptr);
}
