#include "gnc/control_factory.h"
#include "hydrox/runtime/hitl_board.h"
#include "hydrox/runtime/hitl_supervisor.h"

#include <memory>
#include <new>
#include <utility>

#if defined(__NuttX__)
static_assert(
    EIGEN_STACK_ALLOCATION_LIMIT == 4096,
    "FMUv6C HITL requires a bounded Eigen runtime stack-allocation limit");
#endif

extern "C" int hydrox_hitl_main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    hydrox::runtime::HitlBoard *board = hydrox_hitl_board();
    if (board == nullptr)
        return 10;

    for (;;)
    {
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

        // One profile is one state epoch. Every switch destroys and rebuilds
        // the estimator, controller and allocator before READY is emitted.
        profile.runtime.motor = profile.control.motor;
        hydrox::ControlStack stack =
            hydrox::build_control_stack(profile.control);
        std::unique_ptr<hydrox::runtime::HitlSupervisor> supervisor{
            new (std::nothrow) hydrox::runtime::HitlSupervisor(
                *board, profile, std::move(stack))};
        if (!supervisor)
        {
            board->notify(
                hydrox::runtime::HitlHealthEvent::CONFIGURATION_ERROR,
                "HITL supervisor allocation failed");
            return 14;
        }
        const int result = supervisor->run();
        if (result == hydrox::runtime::kHitlBootloaderRestartRequested)
        {
            // Destroy every stateful control object and close its streams
            // before the BSP writes the bootloader signature and resets.
            supervisor.reset();
            if (!board->reboot_to_bootloader())
            {
                board->notify(
                    hydrox::runtime::HitlHealthEvent::CONFIGURATION_ERROR,
                    "bootloader reboot returned unexpectedly");
                return 15;
            }
            return 15;
        }
        if (result != hydrox::runtime::kHitlProfileSwitchRequested &&
            result != hydrox::runtime::kHitlSessionConfigRestartRequested)
            return result;
    }
}

extern "C" void HydroX_main(void)
{
    (void)hydrox_hitl_main(0, nullptr);
}
