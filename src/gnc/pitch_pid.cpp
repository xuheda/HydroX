/** pitch_pid.cpp — Pitch PID inner loop controller */
#include "gnc/pitch_pid.h"
#include <algorithm>
#include <cmath>

namespace hydrox
{

    PitchPID::PitchPID(const Params &p) : _p(p) {}

    void PitchPID::reset(double pitch_rad)
    {
        (void)pitch_rad;
        _ei = 0.0;
    }

    double PitchPID::step(double pitch_rad, double pitch_rate, double theta_ref, double dt)
    {
        double e = theta_ref - pitch_rad;
        _ei += e * dt;
        _ei = std::max(-_p.ei_max, std::min(_p.ei_max, _ei));

        // D-term on measured rate (negative feedback) for stability
        double tau_M = _p.kp * e - _p.kd * pitch_rate + _p.ki * _ei;
        return std::max(-_p.tau_max, std::min(_p.tau_max, tau_M));
    }

} // namespace hydrox
