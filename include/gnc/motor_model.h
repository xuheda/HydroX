#pragma once
/**
 * motor_model.h — Thruster motor dynamic model
 *
 * Physical steps:
 *   RPM_cmd  →  [First-order lag tau_m]  →  omega_actual (rad/s)
 *            →  T = KT * rho * D^4 * omega * |omega|       (Thrust N)
 *            →  Q = KQ * rho * D^5 * omega * |omega|       (Shaft torque N·m)
 *            →  P_shaft = Q * |omega|                      (Shaft power W)
 *            →  P_elec  = P_shaft / eta                    (Electrical power W)
 *            →  I       = P_elec / V_bat                   (Current A)
 *
 * Parameter reference: REMUS 100 / VideoRay class torpedo thruster
 *   tau_m  ≈ 0.20 s   (Motor + propeller inertia time constant)
 *   KT_0 = 0.4566     (Dimensionless thrust coefficient, consistent with ControlAllocator)
 *   KQ_0 = 0.06       (Dimensionless torque coefficient)
 *   D    = 0.14 m
 *   eta  = 0.85       (Motor efficiency)
 *   V    = 24 V       (Nominal battery voltage)
 */
#include <cmath>

namespace hydrox
{

    struct MotorState
    {
        double omega_rad_s = 0.0; // Actual angular velocity (rad/s)
        double rpm_actual = 0.0;  // Actual RPM (= omega * 60/(2pi))
        double thrust_N = 0.0;    // Actual thrust (N), can be positive or negative
        double torque_Nm = 0.0;   // Shaft torque (N·m)
        double power_W = 0.0;     // Electrical power (W) >= 0
        double current_A = 0.0;   // Current (A) >= 0
    };

    class MotorModel
    {
    public:
        struct Params
        {
            double tau_m = 0.20;     // First-order lag time constant (s)
            double KT_0 = 0.4566;    // Thrust coefficient
            double KQ_0 = 0.0600;    // Torque coefficient
            double D_prop = 0.14;    // Propeller diameter (m)
            double rho = 1028.0;     // Water density (kg/m³)
            double eta_motor = 0.85; // Motor efficiency (shaft power/electrical power)
            double V_bat = 24.0;     // Nominal battery voltage (V)
            double rpm_max = 1525.0; // Max speed (RPM)
        };

        explicit MotorModel(const Params &p = {}) : _p(p) {}

        void reset() { _omega = 0.0; }

        /**
         * step — Perform one simulation step
         * @param rpm_cmd  GNC commanded speed (RPM, from ControlAllocator)
         * @param dt       Time step size (s)
         * @return         Motor state after this step
         */
        MotorState step(double rpm_cmd, double dt)
        {
            // 1. RPM -> omega_cmd (rad/s)
            const double omega_cmd = rpm_cmd * (2.0 * 3.14159265358979 / 60.0);

            // 2. First-order lag integration (Euler)
            //    d(omega)/dt = (omega_cmd - omega) / tau_m
            _omega += (dt / _p.tau_m) * (omega_cmd - _omega);

            // 3. Propeller hydrodynamics
            //    T = KT * rho * D^4 * omega * |omega|  (keep sign, omega < 0 -> reverse thrust)
            const double D4 = _p.D_prop * _p.D_prop * _p.D_prop * _p.D_prop;
            const double D5 = D4 * _p.D_prop;
            const double thrust = _p.KT_0 * _p.rho * D4 * _omega * std::abs(_omega);
            const double torque = _p.KQ_0 * _p.rho * D5 * _omega * std::abs(_omega);

            // 4. Power & current (take absolute value, power has no direction)
            const double p_shaft = std::abs(torque * _omega);
            const double p_elec = (_p.eta_motor > 1e-6) ? p_shaft / _p.eta_motor : 0.0;
            const double current = (_p.V_bat > 1e-6) ? p_elec / _p.V_bat : 0.0;

            MotorState s;
            s.omega_rad_s = _omega;
            s.rpm_actual = _omega * (60.0 / (2.0 * 3.14159265358979));
            s.thrust_N = thrust;
            s.torque_Nm = torque;
            s.power_W = p_elec;
            s.current_A = current;
            return s;
        }

        double omega() const { return _omega; }

    private:
        Params _p;
        double _omega = 0.0; // Current actual omega (rad/s)
    };

} // namespace hydrox
