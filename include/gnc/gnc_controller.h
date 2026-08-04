#pragma once
/**
 * gnc_controller.h — SlenderBodyAUVController (cascade control, torpedo-type AUV configuration)
 *
 * Configuration: Underactuated slender body (EcaA9/LAUV). Depth is controlled via pitch, not directly via heave.
 * Input: NavigationState + GNCSetpoint + GNCMode
 * Output: Wrench (generalized force tau) — Allocation is handled by a separate IAllocator (see control_interfaces.h).
 *
 * Formerly GNCController; renamed after separating the allocator to distinguish it from other configurations (ROV/USV/Aero).
 */
#include "types.h"
#include "gnc/control_interfaces.h"
#include "gnc/depth_pid.h"
#include "gnc/pitch_pid.h"
#include "gnc/heading_smc.h"
#include "gnc/surge_p.h"

namespace hydrox
{

    class SlenderBodyAUVController : public IController
    {
    public:
        struct Params
        {
            struct YawRatePI
            {
                double kp = 35.0;
                double ki = 50.0;
                double kd = 3.0;
                double ei_max = 0.6;
                double tau_max = 0.0; // <= 0 means unlimited
                double command_gain = 1.0;
                double feed_forward = 0.0;
                double ref_filter_tau = 0.0;       // s; <= 0 disables filtering
                double ref_slew_limit = 0.0;       // rad/s^2; <= 0 disables slew limiting
            };

            struct TurnSpeedCompensation
            {
                double drop_start_radps = 0.0;
                double drop_gain_mps_per_radps = 0.0;
                double drop_max_mps = 0.0;
            };

            DepthPID::Params depth;
            PitchPID::Params pitch;
            HeadingSMC::Params heading;
            SurgeP::Params surge;
            YawRatePI yaw_rate;
            TurnSpeedCompensation turn_speed;
        };

        explicit SlenderBodyAUVController(const Params &p = {});

        void reset(const NavigationState &state) override;
        void set_mode(GNCMode mode) override
        {
            _mode = mode;
            if (_mode == GNCMode::DISABLED)
            {
                _surge_p.reset();
                _yaw_rate_i = 0.0;
                _yaw_rate_ref_initialized = false;
                _yaw_rate_ref_filtered = 0.0;
            }
        }
        void set_setpoint(const GNCSetpoint &sp) override { _sp = sp; }

        Wrench update(const NavigationState &state, double dt) override;

    private:
        GNCMode _mode = GNCMode::DISABLED;
        GNCSetpoint _sp;

        DepthPID _depth_pid;
        PitchPID _pitch_pid;
        HeadingSMC _heading_smc;
        SurgeP _surge_p;
        Params::YawRatePI _yaw_rate_p;
        Params::TurnSpeedCompensation _turn_speed_p;
        // Integral state for PI yaw-rate tracking (faithful follow of yaw_rate_ref).
        double _yaw_rate_i = 0.0;
        double _yaw_rate_ref_filtered = 0.0;
        bool _yaw_rate_ref_initialized = false;

        double shaped_yaw_rate_ref(double raw_ref, double dt);
    };

} // namespace hydrox
