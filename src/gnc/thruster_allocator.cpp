// Copyright (c) 2026 OceanX. Author: xuheda
/** thruster_allocator.cpp — ThrusterMatrixAllocator (generic multi-thruster). */
#include "gnc/thruster_allocator.h"
#include <algorithm>

namespace hydrox
{

    ThrusterMatrixAllocator::ThrusterMatrixAllocator(const Params &p) : _p(p)
    {
        const int n = static_cast<int>(_p.thrusters.size());
        // Configuration matrix B (6 x n): each column maps a unit thruster force
        // to a body wrench  [force ; moment] = [dir ; pos x dir].
        Eigen::Matrix<double, 6, Eigen::Dynamic> B(6, n);
        for (int i = 0; i < n; ++i)
        {
            const Eigen::Vector3d d = _p.thrusters[i].dir.normalized();
            const Eigen::Vector3d r = _p.thrusters[i].pos;
            B.block<3, 1>(0, i) = d;
            B.block<3, 1>(3, i) = r.cross(d);
        }
        // Damped least squares: u = B^T (B B^T + lambda I)^-1 tau.
        Eigen::Matrix<double, 6, 6> BBt =
            B * B.transpose() + _p.lambda * Eigen::Matrix<double, 6, 6>::Identity();
        _Bpinv = B.transpose() * BBt.inverse(); // n x 6
    }

    ActuatorCmd ThrusterMatrixAllocator::allocate(const Wrench &tau, double /*surge*/) const
    {
        const int n = static_cast<int>(_p.thrusters.size());
        const Eigen::Matrix<double, Eigen::Dynamic, 1> u = _Bpinv * tau; // per-thruster force (N)

        ActuatorCmd cmd;
        const int nc = std::min<int>(n, static_cast<int>(cmd.ch.size()));
        for (int i = 0; i < nc; ++i)
        {
            const double mx = std::max(_p.thrusters[i].max_thrust_N, 1e-6);
            const double frac = u(i) / mx;
            cmd.ch[i] = static_cast<float>(std::max(-1.0, std::min(1.0, frac)));
        }
        // rpm is fin/prop-specific; leave 0 for a thruster array.
        return cmd;
    }

} // namespace hydrox
