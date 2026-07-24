/** heading_smc.cpp — Heading integral sliding mode controller (Nomoto inverse model) */
#include "gnc/heading_smc.h"
#include "gnc/ref_model.h"
#include <cmath>
#include <algorithm>

namespace hydrox
{

    HeadingSMC::HeadingSMC(const Params &p) : _p(p)
    {
        _ref = {0.0, 0.0, 0.0};
    }

    void HeadingSMC::reset(double heading_rad)
    {
        _ref = {heading_rad, 0.0, 0.0};
        _ei = 0.0;
    }

    double HeadingSMC::step(double heading_rad, double yaw_rate,
                            double heading_ref, double dt)
    {
        ref_model3_step(_ref, heading_ref, _p.wn, _p.zeta, _p.vmax, dt);

        double e_psi = ssa(_ref.xd - heading_rad);
        double e_r = _ref.vd - yaw_rate;
        _ei += e_psi * dt;
        _ei = std::max(-_p.ei_max, std::min(_p.ei_max, _ei));

        double sigma = e_r + _p.kd * e_psi + _p.ks * _ei;
        double sat = std::max(-1.0, std::min(1.0, sigma / _p.phi_b));

        // Nomoto inverse model feedforward + sliding mode feedback
        double tau_N = _p.T * (_p.wn * e_r - _p.kd * yaw_rate) / _p.K + _p.ks * sat / _p.K;
        return tau_N;
    }

} // namespace hydrox
