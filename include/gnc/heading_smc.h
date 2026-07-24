#pragma once
/** heading_smc.h — Heading integral sliding mode controller (ported from Python heading_smc.py) */
#include "gnc/ref_model.h"

namespace hydrox
{

    class HeadingSMC
    {
    public:
        struct Params
        {
            double T = 1.0;
            double K = 0.4;
            double wn = 0.6;
            double zeta = 1.0;
            double vmax = 0.5;
            double kd = 1.0;
            double ks = 0.05;
            double phi_b = 0.1;
            double ei_max = 3.14159265358979;
        };

        explicit HeadingSMC(const Params &p = {});
        void reset(double heading_rad = 0.0);

        /** @return tau_N (N·m) */
        double step(double heading_rad, double yaw_rate,
                    double heading_ref, double dt);

    private:
        Params _p;
        RefModelState _ref;
        double _ei = 0.0;
    };

} // namespace hydrox
