#pragma once
/**
 * fossen_vehicle_params.h — legacy control-parameter compatibility loader.
 *
 * New integrations should use vehicle_bundle.h. This API remains for callers
 * that explicitly supply an existing parameter file or parameter directory.
 */

#include "gnc/control_allocator.h"
#include "gnc/fixedwing_allocator.h"
#include "gnc/fixedwing_controller.h"
#include "gnc/motor_model.h"
#include "gnc/gnc_controller.h"
#include "gnc/multirotor_controller.h"
#include "gnc/surface_controller.h"
#include "gnc/thruster_controller.h"
#include "gnc/vtol_controller.h"
#include "types.h"

#include <string>

namespace hydrox
{

    // Control archetype — selects the IController/IAllocator pair. SlenderBodyFin = torpedo (fins+prop);
    // Thruster = ROV/hovering AUV (thruster array); Surface = USV.
    enum class VehicleArchetype
    {
        SlenderBodyFin,
        Thruster,
        Surface,
        Multirotor,
        FixedWing,
        VTOL,
    };

    /** Map a (possibly non-canonical) vehicle type string to its control archetype. */
    VehicleArchetype archetype_for(const std::string &type);

    struct FossenControlParams
    {
        bool valid = false;
        bool loaded_from_json = false;
        VehicleArchetype archetype = VehicleArchetype::SlenderBodyFin;
        VehicleClass vehicle_class = VehicleClass::UUV;
        std::string vehicle_type;
        std::string source_path;
        FinAllocator::Params allocator;
        MotorModel::Params motor;
        SlenderBodyAUVController::Params gnc;
        ThrusterVehicleController::Params thruster_gnc;
        SurfaceVesselController::Params surface_gnc;
        MultirotorController::Params multirotor_gnc;
        FixedWingController::Params fixedwing_gnc;
        FixedWingAllocator::Params fixedwing_allocator;
        VtolController::Params vtol_gnc;
        bool yaw_rate_control_loaded_from_json = false;
        bool archetype_control_loaded_from_json = false;

        // Thruster/surface vehicles: per-thruster saturation (N). Fin vehicles leave 0.
        double max_thrust_per_thruster_N = 0.0;

        // Surface vehicles may group several physical engines behind a single
        // HydroX channel (for example VRX WAM-V's two engines per side).
        // This is the effective surge force and yaw lever of one such channel.
        double surface_channel_surge_limit_N = 0.0;
        double surface_channel_lever_arm_m = 0.395;

        // Air vehicles: physical sum of all vertical lift rotors at full command.
        // This must match the UE Aero kT/omega_max model because the allocator
        // mixes in thrust space and emits normalized rotor angular speed.
        double max_total_lift_N = 0.0;

        // Rigid + added inertia, computed from the JSON inertia/geometry section
        // (prolate-spheroid Lamb factors, same formula as the UE5 dynamics side).
        // Used to scale GNC gains per vehicle — see apply_inertia_normalized_gains.
        // Zero means "unknown" → gain normalization is skipped (defaults kept).
        double M44_pitch = 0.0;  // pitch/yaw moment of inertia incl. added mass (kg·m²)
        double mass_total = 0.0; // surge mass incl. added mass (kg)
    };

    std::string canonical_vehicle_type(const std::string &type);
    std::string fossen_params_filename(const std::string &type);

    FossenControlParams builtin_fossen_control_params(const std::string &type);

    FossenControlParams load_fossen_control_params(
        const std::string &type,
        const std::string &explicit_path = {},
        const std::string &params_dir = {},
        std::string *error = nullptr);

    /**
     * Scale GNC control gains by THIS vehicle's inertia so every vehicle gets the
     * same closed-loop response (natural frequency ωn, damping ζ) regardless of
     * mass/inertia — instead of all sharing one gain set tuned for EcaA9.
     *
     * Why this works: a 2nd-order torque loop has kp = J·ωn², kd = J·2ζωn. Scaling
     * kp AND kd by (J_this / J_ref) keeps ωn and ζ identical while adapting the raw
     * gains to the vehicle's inertia. So:
     *   - torque loops (pitch, yaw + the pitch torque limit) scale with M44_pitch
     *   - the force loop (surge) scales with mass_total
     * The reference inertia is the value the DEFAULT PID gains were tuned for
     * (EcaA9). EcaA9 → ratio≈1 (gains unchanged, zero regression); a light hull
     * like LAUV (M44≈3 vs 26) gets its torque gains scaled down ~8×, which is what
     * keeps it stable.
     *
     * JSON yaw-rate gains are treated as vehicle-specific physical gains and
     * are not scaled a second time.
     *
     * No-op if vp.M44_pitch / mass_total are unknown (zero) -> defaults are kept.
     */
    void apply_inertia_normalized_gains(SlenderBodyAUVController::Params &gnc,
                                        const FossenControlParams &vp);

} // namespace hydrox
