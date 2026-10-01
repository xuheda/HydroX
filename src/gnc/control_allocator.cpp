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

        // --- Geometry-driven stern fins ---
        // Fin angle a is the radial hinge-axis azimuth in body FRD. Positive
        // source-joint deflection produces pitch/yaw effectiveness
        //   M_i = -q*S*CL_s*x*cos(a_i)*delta_i
        //   N_i = -q*S*CL_r*x*sin(a_i)*delta_i.
        // Solve the 2x4 minimum-norm allocation B^T(BB^T)^-1 [M,N]. This makes
        // plus tails and X tails use their real source-model channel order.
        std::array<double, 4> pitch_effect{};
        std::array<double, 4> yaw_effect{};
        double gram_mm = 0.0;
        double gram_mn = 0.0;
        double gram_nn = 0.0;
        constexpr double kD2R = 3.14159265358979 / 180.0;
        for (std::size_t i = 0; i < pitch_effect.size(); ++i)
        {
            const double angle = _p.fin_angles_deg[i] * kD2R;
            pitch_effect[i] = -q * _p.S_fin * _p.CL_s * _p.x_fin * std::cos(angle);
            yaw_effect[i] = -q * _p.S_fin * _p.CL_r * _p.x_fin * std::sin(angle);
            gram_mm += pitch_effect[i] * pitch_effect[i];
            gram_mn += pitch_effect[i] * yaw_effect[i];
            gram_nn += yaw_effect[i] * yaw_effect[i];
        }

        std::array<double, 4> delta{};
        const double determinant = gram_mm * gram_nn - gram_mn * gram_mn;
        if (determinant > 1e-12)
        {
            const double dual_m = (gram_nn * tau_M - gram_mn * tau_N) / determinant;
            const double dual_n = (gram_mm * tau_N - gram_mn * tau_M) / determinant;
            double max_abs_delta = 0.0;
            for (std::size_t i = 0; i < delta.size(); ++i)
            {
                delta[i] = pitch_effect[i] * dual_m + yaw_effect[i] * dual_n;
                max_abs_delta = std::max(max_abs_delta, std::abs(delta[i]));
            }
            // Preserve the requested pitch/yaw ratio when the combined demand
            // reaches a servo limit; independent clipping distorts turn direction.
            if (max_abs_delta > delta_max && max_abs_delta > 1e-12)
            {
                const double scale = delta_max / max_abs_delta;
                for (double &value : delta)
                    value *= scale;
            }
        }

        ActuatorCmd out;
        out.layout = ActuatorLayout::FinAndPropeller;
        for (std::size_t i = 0; i < delta.size(); ++i)
            out.set_position(i, delta[i], delta_max, delta_max);
        out.set_thrust(4, tau_X, _p.max_thrust_N, _p.max_thrust_N);
        out.rpm = rpm;
        return out;
    }

} // namespace hydrox
