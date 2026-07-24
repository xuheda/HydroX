#include "gnc/control_factory.h"
#include "gnc/control_allocator.h"
#include "gnc/fixedwing_allocator.h"
#include "gnc/fixedwing_controller.h"
#include "gnc/vtol_allocator.h"
#include "gnc/vtol_controller.h"
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
    ThrusterMatrixAllocator::Params vectored_rov_thrusters(double max_thrust_N)
    {
        ThrusterMatrixAllocator::Params p;
        const double s = 0.70710678;
        auto add = [&](double x, double y, double z, double dx, double dy, double dz)
        {
            Thruster t;
            t.pos = Eigen::Vector3d(x, y, z);
            t.dir = Eigen::Vector3d(dx, dy, dz).normalized();
            t.max_thrust_N = max_thrust_N;
            p.thrusters.push_back(t);
        };
        add(0.12, 0.2181, -0.0809, 0.0, 0.0, -1.0);
        add(0.12, -0.2181, -0.0809, 0.0, 0.0, -1.0);
        add(-0.12, -0.2181, -0.0809, 0.0, 0.0, -1.0);
        add(-0.12, 0.2181, -0.0809, 0.0, 0.0, -1.0);
        add(0.1562, 0.0988, 0.0, s, -s, 0.0);
        add(0.1562, -0.0988, 0.0, s, s, 0.0);
        add(-0.1562, -0.0988, 0.0, s, -s, 0.0);
        add(-0.1562, 0.0988, 0.0, s, s, 0.0);
        return p;
    }

    ThrusterMatrixAllocator::Params desistek_thrusters(double max_thrust_N)
    {
        ThrusterMatrixAllocator::Params p;
        auto add = [&](double x, double y, double z, double dx, double dy, double dz)
        {
            Thruster t;
            t.pos = Eigen::Vector3d(x, y, z);
            t.dir = Eigen::Vector3d(dx, dy, dz).normalized();
            t.max_thrust_N = max_thrust_N;
            p.thrusters.push_back(t);
        };
        add(-0.088, 0.1192, 0.00618, -1.0, 0.0, 0.0);
        add(-0.088, -0.1192, 0.00618, -1.0, 0.0, 0.0);
        add(0.067, 0.0, -0.12582, 0.0, 0.0, 1.0);
        return p;
    }

    ThrusterMatrixAllocator::Params rexrov2_thrusters(double max_thrust_N)
    {
        ThrusterMatrixAllocator::Params p;
        auto add = [&](double x, double y, double z, double dx, double dy, double dz)
        {
            Thruster t;
            t.pos = Eigen::Vector3d(x, y, z);
            t.dir = Eigen::Vector3d(dx, dy, dz).normalized();
            t.max_thrust_N = max_thrust_N;
            p.thrusters.push_back(t);
        };
        add(0.4878, 0.0, -0.2373, 0.0, -1.0, 0.0);
        add(-0.8217, 0.0, -0.5890, 0.2588, 0.0, -0.9659);
        add(0.8654, 0.5322, -0.5332, 0.0, -0.3827, -0.9239);
        add(0.8654, -0.5322, -0.5332, 0.0, -0.3827, 0.9239);
        add(-0.7076, -0.5129, -0.2404, 0.9063, -0.4226, 0.0);
        add(-0.7076, 0.5129, -0.2404, 0.9063, 0.4226, 0.0);
        return p;
    }
} // namespace

ControlStack build_control_stack(const FossenControlParams &vp)
{
    ControlStack stack;

    if (vp.archetype == VehicleArchetype::Thruster)
    {
        ThrusterVehicleController::Params cp = vp.thruster_gnc;
        if (vp.mass_total > 1e-6)
            cp.mass = vp.mass_total;
        stack.controller = std::make_unique<ThrusterVehicleController>(cp);

        ThrusterMatrixAllocator::Params tp;
        if (vp.vehicle_type == "DesistekSaga")
            tp = desistek_thrusters(vp.max_thrust_per_thruster_N);
        else if (vp.vehicle_type == "RexROV2")
            tp = rexrov2_thrusters(vp.max_thrust_per_thruster_N);
        else
            tp = vectored_rov_thrusters(vp.max_thrust_per_thruster_N);
        stack.allocator = std::make_unique<ThrusterMatrixAllocator>(tp);
        return stack;
    }

    if (vp.archetype == VehicleArchetype::Surface)
    {
        SurfaceVesselController::Params cp = vp.surface_gnc;
        const double ChannelForce = vp.surface_channel_surge_limit_N > 1e-6
            ? vp.surface_channel_surge_limit_N
            : vp.max_thrust_per_thruster_N;
        const double LeverArm = std::max(std::abs(vp.surface_channel_lever_arm_m), 1e-6);
        if (!vp.archetype_control_loaded_from_json && vp.mass_total > 1e-6)
        {
            cp.surge_kp = 3.0 * vp.mass_total;
            cp.surge_kd = 1.0 * vp.mass_total;
            cp.max_force_N = 2.0 * ChannelForce;
            cp.max_moment_Nm = 2.0 * ChannelForce * LeverArm;
        }
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
        if (vp.mass_total > 1e-6)
            cp.mass = vp.mass_total;
        stack.controller = std::make_unique<MultirotorController>(cp);

        QuadrotorAllocator::Params ap;
        ap.max_total_thrust_N = vp.max_total_lift_N > 1e-6
                                    ? vp.max_total_lift_N
                                    : cp.mass * cp.g * 2.5;
        stack.allocator = std::make_unique<QuadrotorAllocator>(ap);
        return stack;
    }

    if (vp.archetype == VehicleArchetype::FixedWing)
    {
        stack.controller = std::make_unique<FixedWingController>(vp.fixedwing_gnc);
        stack.allocator = std::make_unique<FixedWingAllocator>();
        return stack;
    }

    if (vp.archetype == VehicleArchetype::VTOL)
    {
        VtolController::Params cp = vp.vtol_gnc;
        if (vp.mass_total > 1e-6)
            cp.mass = vp.mass_total;
        stack.controller = std::make_unique<VtolController>(cp);
        VtolAllocator::Params ap;
        if (vp.max_total_lift_N > 1e-6)
            ap.max_total_lift_N = vp.max_total_lift_N;
        stack.allocator = std::make_unique<VtolAllocator>(ap);
        return stack;
    }

    SlenderBodyAUVController::Params gnc_params = vp.gnc;
    apply_inertia_normalized_gains(gnc_params, vp);
    stack.controller = std::make_unique<SlenderBodyAUVController>(gnc_params);
    stack.allocator = std::make_unique<FinAllocator>(vp.allocator);
    return stack;
}

} // namespace hydrox
