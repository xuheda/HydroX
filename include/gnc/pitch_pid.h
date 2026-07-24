#pragma once
/** pitch_pid.h — Pitch PID inner loop controller */

namespace hydrox
{

    class PitchPID
    {
    public:
        struct Params
        {
            double kp = 60.0;      // N·m/rad
            double kd = 15.0;      // N·m/(rad/s)
            double ki = 0.0;       // N·m/(rad·s)
            double ei_max = 5.0;   // Integral limit
            double tau_max = 20.0; // Pitch torque limit (N·m)
        };

        explicit PitchPID(const Params &p = {});
        void reset(double pitch_rad = 0.0);

        /**
         * @return tau_M (N·m)
         */
        double step(double pitch_rad, double pitch_rate, double theta_ref, double dt);

    private:
        Params _p;
        double _ei = 0.0;
    };

} // namespace hydrox
