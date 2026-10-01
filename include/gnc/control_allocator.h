#pragma once
#include "hydrox/default_parameter.h"
/** control_allocator.h — FinAllocator: tau -> geometry-driven fins + stern prop. */
#include "types.h"
#include "gnc/control_interfaces.h"
#include <array>
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
            // Values are supplied by a validated VehicleBundle. These zero-defaults
            // will cause divide-by-zero if used without initialization.
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
            // Radial hinge-axis azimuths in body FRD, measured from +Y toward
            // +Z. Channel i always addresses physical fin i in source-model order.
            std::array<double, 4> fin_angles_deg = {315.0, 45.0, 135.0, 225.0};
        };

        explicit FinAllocator(const Params &p = detail::default_parameter<Params>());

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
