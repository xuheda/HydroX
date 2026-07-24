#pragma once
/** control_allocator.h — FinAllocator: tau -> cruciform/X-tail fins + stern prop.
 *
 * Was `ControlAllocator`. Implements IAllocator. Step 2 will replace the hardwired
 * +-cross ss/ps/br/tr closed form with a config-driven effectiveness matrix built
 * from fin_angles/fin_cop, so X-tail and + cross fall out of the same code.
 */
#include "types.h"
#include "gnc/control_interfaces.h"
#include <Eigen/Core>
#include <cmath>

namespace hydrox
{

    class FinAllocator : public IAllocator
    {
    public:
        struct Params
        {
            double rho = 1028.0;
            // Values are loaded by fossen_vehicle_params.cpp from engine/Content/Fossen JSON.
            // These zero-defaults will cause divide-by-zero if used without initialization.
            double S_fin = 0.0;          // fin planform area (m²)
            double CL_s = 0.0;           // 3-D elevator lift slope /rad
            double CL_r = 0.0;           // 3-D rudder lift slope /rad
            double x_fin = 1.0;          // fin CoP moment arm from CoM (m)
            double D_prop = 0.14;        // propeller diameter (m)
            double KT_0 = 0.4566;        // open-water thrust coeff at zero advance ratio
            double n_max_rpm = 1000.0;   // max propeller speed (RPM), for motor model clamp
            double max_thrust_N = 1.0;   // max thrust (N), for channel4 normalization
            double delta_max_deg = 15.0;
            double u_min = 0.3;
        };

        explicit FinAllocator(const Params &p = {});

        /**
         * @param tau    Generalized force [X, Y, Z, K, M, N]
         * @param surge  Current surge speed (m/s)
         * @return       Normalized actuator channels + thruster speed
         */
        ActuatorCmd allocate(const Wrench &tau, double surge) const override;

    private:
        Params _p;
        double _kT; // Thrust coefficient KT_0 * rho * D^4
    };

} // namespace hydrox
