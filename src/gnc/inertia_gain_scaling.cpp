#include "fossen_vehicle_params.h"

namespace hydrox
{
    void apply_inertia_normalized_gains(
        SlenderBodyAUVController::Params &gnc,
        const FossenControlParams &vp)
    {
        if (vp.M44_pitch <= 0.0 || vp.mass_total <= 0.0)
            return;

        constexpr double kRefM44 = 25.68;
        constexpr double kRefMass = 71.5;
        const double torque_ratio = vp.M44_pitch / kRefM44;
        const double force_ratio = vp.mass_total / kRefMass;

        gnc.pitch.kp *= torque_ratio;
        gnc.pitch.kd *= torque_ratio;
        gnc.pitch.tau_max *= torque_ratio;
        if (!vp.yaw_rate_control_loaded_from_json)
        {
            gnc.yaw_rate.kp *= torque_ratio;
            gnc.yaw_rate.ki *= torque_ratio;
            gnc.yaw_rate.kd *= torque_ratio;
            gnc.yaw_rate.tau_max *= torque_ratio;
        }

        gnc.surge.kp *= force_ratio;
        gnc.surge.ki *= force_ratio;
        gnc.surge.kd *= force_ratio;
    }
} // namespace hydrox
