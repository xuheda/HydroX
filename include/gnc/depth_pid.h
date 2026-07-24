#pragma once
/** depth_pid.h — Depth PID outer loop controller (outputs desired pitch angle) */
#include "gnc/ref_model.h"

namespace hydrox
{

    class DepthPID
    {
    public:
        struct Params
        {
            double kp = 0.08;        // rad/m  (~4.6 deg/m)
            double kd = 0.15;        // rad/(m/s)
            double ki = 0.01;        // rad/(m·s)
            double wn = 0.3;
            double zeta = 1.0;
            double vmax = 0.5;
            double ei_max = 10.0;
            double theta_max = 0.26; // rad (~15 deg)
        };

        explicit DepthPID(const Params &p = {});
        void reset(double depth_m = 0.0);

        /**
         * @return theta_ref (rad), desired pitch angle
         */
        double step(double depth_m, double heave_rate,
                    double depth_ref, double dt);

    private:
        Params _p;
        RefModelState _ref;
        double _ei = 0.0;
    };

} // namespace hydrox
