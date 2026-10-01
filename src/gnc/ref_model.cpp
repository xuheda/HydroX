/** ref_model.cpp — Third-order reference model + ssa */
#include "gnc/ref_model.h"
#include <cmath>
#include <algorithm>

namespace hydrox
{

    void ref_model3_step(RefModelState &s, double r,
                         double wn, double zeta, double vmax, double dt)
    {
        double jd = wn * wn * wn * (r - s.xd) - (2.0 * zeta + 1.0) * wn * wn * s.vd - (2.0 * zeta + 1.0) * wn * s.ad;
        s.xd += dt * s.vd;
        s.vd += dt * s.ad;
        s.ad += dt * jd;
        s.vd = std::max(-vmax, std::min(vmax, s.vd));
    }

    double ssa(double a)
    {
        constexpr double pi = 3.14159265358979323846;
        return std::fmod(a + pi, 2.0 * pi) - pi;
    }

} // namespace hydrox
