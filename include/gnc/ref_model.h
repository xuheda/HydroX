#pragma once
/** ref_model.h — Third-order smooth reference model (Fossen 2021, §12.1) */
namespace hydrox
{

    struct RefModelState
    {
        double xd = 0.0; // Reference position
        double vd = 0.0; // Reference velocity
        double ad = 0.0; // Reference acceleration
    };

    /**
     * Propagate reference model by one step (forward Euler).
     * @param s     Current state (updated in place)
     * @param r     Setpoint
     * @param wn    Natural frequency (rad/s)
     * @param zeta  Damping ratio
     * @param vmax  Maximum reference velocity
     * @param dt    Time step size (s)
     */
    void ref_model3_step(RefModelState &s, double r,
                         double wn, double zeta, double vmax, double dt);

    /** Smallest signed angle, normalizes a to [-pi, pi) */
    double ssa(double a);

} // namespace hydrox
