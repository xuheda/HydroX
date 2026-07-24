// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once
/**
 * thruster_allocator.h — ThrusterMatrixAllocator: tau -> N thruster commands.
 *
 * Generic control allocation for ANY multi-thruster vehicle (ROV, hovering AUV,
 * twin-screw USV). Builds the 6xN thruster configuration matrix B from the
 * per-thruster geometry (column i = [dir_i ; pos_i x dir_i]) and solves the
 * damped least-squares inverse  u = B^T (B B^T + lambda I)^-1 tau.
 *
 * ONE class, ANY layout — only the thruster list changes per vehicle. This is the
 * piece that lets hydrox cover all the bluff-body ROVs without per-vehicle code.
 * (Ref: Fossen, Handbook of Marine Craft Hydrodynamics, control-allocation ch.;
 *  Johansen & Fossen, "Control allocation—A survey", Automatica 2013.)
 *
 * NOTE: per-thruster saturation is handled by clamping each command to [-1,1].
 * If a thruster saturates the realised wrench deviates from the demand; uniform
 * scaling / QP allocation is a future refinement (see doc §9).
 */
#include "gnc/control_interfaces.h"
#include <Eigen/Dense>
#include <vector>

namespace hydrox
{
    /** One thruster's geometry, in NED body frame, CoM-relative. */
    struct Thruster
    {
        Eigen::Vector3d pos{0.0, 0.0, 0.0}; // position r (m)
        Eigen::Vector3d dir{1.0, 0.0, 0.0}; // unit thrust direction (+command pushes along +dir)
        double max_thrust_N = 1.0;          // per-thruster saturation (N)
    };

    class ThrusterMatrixAllocator : public IAllocator
    {
    public:
        struct Params
        {
            std::vector<Thruster> thrusters;
            double lambda = 1e-3; // Tikhonov damping (robustness near-singular B)
        };

        explicit ThrusterMatrixAllocator(const Params &p);

        ActuatorCmd allocate(const Wrench &tau, double surge) const override;

    private:
        Params _p;
        Eigen::Matrix<double, Eigen::Dynamic, 6> _Bpinv; // N x 6 damped pseudo-inverse
    };

} // namespace hydrox
