/** depth_pid.cpp — Depth PID outer loop controller (outputs desired pitch angle) */
#include "gnc/depth_pid.h"
#include "gnc/ref_model.h"
#include <algorithm>
#include <cmath>

namespace hydrox
{

    DepthPID::DepthPID(const Params &p) : _p(p)
    {
        _ref = {0.0, 0.0, 0.0};
    }

    void DepthPID::reset(double depth_m)
    {
        _ref = {depth_m, 0.0, 0.0};
        _ei = 0.0;
    }

    double DepthPID::step(double depth_m, double heave_rate,
                          double depth_ref, double dt)
    {
        ref_model3_step(_ref, depth_ref, _p.wn, _p.zeta, _p.vmax, dt);
        double e = depth_m - _ref.xd;
        double e_dot = heave_rate - _ref.vd;
        _ei += e * dt;
        _ei = std::max(-_p.ei_max, std::min(_p.ei_max, _ei));

        double theta_ref = _p.kp * e + _p.kd * e_dot + _p.ki * _ei;
        theta_ref = std::max(-_p.theta_max, std::min(_p.theta_max, theta_ref));
        return theta_ref;
    }

} // namespace hydrox
