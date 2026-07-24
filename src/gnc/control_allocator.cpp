/** control_allocator.cpp — FinAllocator: Generalized force -> control surfaces/thrusters allocation (normalized channels) */
#include "gnc/control_allocator.h"
#include <cmath>
#include <algorithm>

namespace hydrox
{

    FinAllocator::FinAllocator(const Params &p) : _p(p)
    {
        _kT = p.KT_0 * p.rho * std::pow(p.D_prop, 4.0);
    }

    static double clamp(double v, double lo, double hi)
    {
        return (v < lo) ? lo : (v > hi) ? hi
                                        : v;
    }

    ActuatorCmd FinAllocator::allocate(const Wrench &tau, double surge) const
    {
        const double tau_X = tau[0];
        const double tau_Z = tau[2];
        const double tau_M = tau[4];
        const double tau_N = tau[5];

        double delta_max = _p.delta_max_deg * (3.14159265358979 / 180.0);

        // Dynamic pressure (not lower than u_min to prevent division by zero)
        double u_eff = std::max(std::abs(surge), _p.u_min);
        double q = 0.5 * _p.rho * u_eff * u_eff;

        // --- Thruster ---
        double n_rps = 0.0;
        if (std::abs(tau_X) > 1e-4 && _kT > 1e-10)
        {
            double sign_x = (tau_X >= 0.0) ? 1.0 : -1.0;
            n_rps = sign_x * std::sqrt(std::abs(tau_X) / _kT);
        }
        double rpm = clamp(n_rps * 60.0, -_p.n_max_rpm, _p.n_max_rpm);
        double thrust_frac = (_p.max_thrust_N > 1e-6)
                                 ? clamp(tau_X / _p.max_thrust_N, -1.0, 1.0)
                                 : 0.0;

        // --- Horizontal Fin (Depth/Pitch) ---
        double denom_s = 2.0 * q * _p.S_fin * _p.CL_s;
        double delta_z = (denom_s > 1e-6) ? tau_Z / denom_s : 0.0;
        double delta_m = (denom_s * _p.x_fin > 1e-6)
                             ? tau_M / (denom_s * _p.x_fin)
                             : 0.0;

        // Starboard stern fin (ss) and port stern fin (ps) differential
        double delta_ss = clamp((delta_z + delta_m) / 2.0, -delta_max, delta_max);
        double delta_ps = clamp((delta_z - delta_m) / 2.0, -delta_max, delta_max);

        // --- Vertical Fin (Bow/Stern, Heading Control) ---
        // tau_N > 0 (NED clockwise) -> delta_r > 0 -> generates positive CW torque -> psi increases (standard NED).
        double denom_r = q * _p.S_fin * _p.CL_r * _p.x_fin;
        double delta_r = (denom_r > 1e-6) ? tau_N / denom_r : 0.0;
        double delta_br = clamp(delta_r / 2.0, -delta_max, delta_max);
        double delta_tr = clamp(-delta_r / 2.0, -delta_max, delta_max); // differential

        constexpr double kR2D = 180.0 / 3.14159265358979;
        // fins order: [delta_ss, delta_ps, delta_br, delta_tr], unit: degrees
        FinCmd fc;
        fc.fins[0] = delta_ss * kR2D;
        fc.fins[1] = delta_ps * kR2D;
        fc.fins[2] = delta_br * kR2D;
        fc.fins[3] = delta_tr * kR2D;
        fc.rpm = rpm;
        fc.thrust_frac = thrust_frac;
        fc.clamp(_p.delta_max_deg, _p.n_max_rpm);

        // Normalization -> general actuator command (clamp + normalize consolidated here from main_sitl)
        // ActuatorCmd has 8 channels (compatible with thruster-based vehicles); fins only fill the first 5, the rest remain 0.
        ActuatorCmd out;
        const auto n5 = fc.normalized(_p.delta_max_deg, _p.n_max_rpm);
        for (int i = 0; i < 5; ++i)
            out.ch[i] = n5[i];
        out.rpm = fc.rpm;
        return out;
    }

} // namespace hydrox
