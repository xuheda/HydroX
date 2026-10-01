#include "gnc/control_factory.h"
#include "gnc/control_allocator.h"
#include "gnc/fixedwing_allocator.h"
#include "gnc/fixedwing_controller.h"
#include "gnc/ground_allocator.h"
#include "gnc/ground_controller.h"
#include "gnc/vtol_allocator.h"
#include "gnc/vtol_controller.h"
#include "vehicle_bundle.h"
#include "gnc/gnc_controller.h"
#include "gnc/multirotor_allocator.h"
#include "gnc/multirotor_controller.h"
#include "gnc/surface_allocator.h"
#include "gnc/surface_controller.h"
#include "gnc/thruster_allocator.h"
#include "gnc/thruster_controller.h"

#include <algorithm>
#include <cmath>

namespace hydrox
{
namespace
{
    class DirectBodyWrenchAllocator final : public IAllocator
    {
    public:
        DirectBodyWrenchAllocator(double force_limit_N, double moment_limit_Nm)
            : _force_limit_N(std::max(1.0, force_limit_N)),
              _moment_limit_Nm(std::max(1.0, moment_limit_Nm))
        {
        }

        ActuatorCmd allocate(const Wrench &tau, double) const override
        {
            ActuatorCmd command;
            command.layout = ActuatorLayout::DirectBodyWrench;
            for (std::size_t index = 0; index < 3; ++index)
                command.set_body_force(index, tau[index], _force_limit_N);
            for (std::size_t index = 3; index < 6; ++index)
                command.set_body_moment(index, tau[index], _moment_limit_Nm);
            return command;
        }

    private:
        double _force_limit_N = 1.0;
        double _moment_limit_Nm = 1.0;
    };

} // namespace

ControlStack build_control_stack(const ControlParameters &vp)
{
    ControlStack stack;

    if (vp.archetype == VehicleArchetype::Thruster)
    {
        ThrusterVehicleController::Params cp = vp.thruster_gnc;
        cp.mass = vp.mass_total;
        stack.controller = std::make_unique<ThrusterVehicleController>(cp);

        if (vp.direct_body_wrench)
        {
            // The generic Unreal bluff-body plant exposes six normalized
            // generalized wrench axes, not a fictitious thruster geometry.
            const double wrench_limit =
                8.0 * std::max(1.0, vp.max_thrust_per_thruster_N);
            stack.allocator = std::make_unique<DirectBodyWrenchAllocator>(
                wrench_limit, wrench_limit);
            return stack;
        }
        ThrusterMatrixAllocator::Params tp;
        tp.thrusters = vp.thrusters;

        stack.allocator = std::make_unique<ThrusterMatrixAllocator>(tp);
        return stack;
    }

    if (vp.archetype == VehicleArchetype::Surface)
    {
        SurfaceVesselController::Params cp = vp.surface_gnc;
        const double ChannelForce = vp.surface_channel_surge_limit_N;
        const double LeverArm = vp.surface_channel_lever_arm_m;

        stack.controller = std::make_unique<SurfaceVesselController>(cp);

        TwinScrewAllocator::Params ap;
        ap.max_thrust_N = ChannelForce;
        ap.lever_arm_m = LeverArm;
        stack.allocator = std::make_unique<TwinScrewAllocator>(ap);
        return stack;
    }

    if (vp.archetype == VehicleArchetype::Multirotor)
    {
        MultirotorController::Params cp = vp.multirotor_gnc;
        cp.mass = vp.mass_total;
        stack.controller = std::make_unique<MultirotorController>(cp);

        QuadrotorAllocator::Params ap;
        ap.max_total_thrust_N = vp.max_total_lift_N;
        ap.roll_pitch_moment_arm_m = vp.lift_roll_pitch_moment_arm_m;
        ap.yaw_moment_per_thrust_m = vp.lift_yaw_moment_per_thrust_m;
        stack.allocator = std::make_unique<QuadrotorAllocator>(ap);
        return stack;
    }

    if (vp.archetype == VehicleArchetype::DifferentialDrive)
    {
        stack.controller = std::make_unique<DifferentialDriveController>(vp.ground_gnc);
        stack.allocator = std::make_unique<DifferentialDriveAllocator>(vp.ground_allocator);
        return stack;
    }

    if (vp.archetype == VehicleArchetype::FixedWing)
    {
        stack.controller = std::make_unique<FixedWingController>(vp.fixedwing_gnc);
        stack.allocator = std::make_unique<FixedWingAllocator>(vp.fixedwing_allocator);
        return stack;
    }

    if (vp.archetype == VehicleArchetype::VTOL)
    {
        VtolController::Params cp = vp.vtol_gnc;
        cp.mass = vp.mass_total;
        stack.controller = std::make_unique<VtolController>(cp);
        VtolAllocator::Params ap = vp.vtol_allocator;
        ap.max_total_lift_N = vp.max_total_lift_N;
        ap.roll_pitch_moment_arm_m = vp.lift_roll_pitch_moment_arm_m;
        ap.yaw_moment_per_thrust_m = vp.lift_yaw_moment_per_thrust_m;
        stack.allocator = std::make_unique<VtolAllocator>(ap);
        return stack;
    }

    SlenderBodyAUVController::Params gnc_params = vp.gnc;
    stack.controller = std::make_unique<SlenderBodyAUVController>(gnc_params);
    stack.allocator = std::make_unique<FinAllocator>(vp.allocator);
    return stack;
}

ControlStack build_control_stack(const VehicleBundle &bundle)
{
    return build_control_stack(bundle.control);
}

} // namespace hydrox
