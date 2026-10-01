#include "estimation_profile.h"

namespace hydrox
{
    namespace
    {
        EstimationProfile uuv_profile()
        {
            EstimationProfile profile;
            // Hydrodynamic and centripetal specific force is not a gravity-only
            // attitude observation. Level from the accelerometer only during
            // stationary initialization/hold, then let gyro + DVL/depth/mag
            // propagate the moving underwater vehicle.
            profile.ekf.level_max_body_speed_mps = 0.25;
            profile.ekf.level_max_gyro_radps = 0.10;
            return profile;
        }

        EstimationProfile usv_profile()
        {
            EstimationProfile profile;
            profile.vehicle_class = VehicleClass::USV;
            profile.vertical_aid = VerticalAidMode::SurfaceConstraint;
            profile.medium_velocity_kind = MediumVelocityKind::WaterCurrent;
            // This is a mean-water-level prior, not an instantaneous pressure
            // sensor. Wave heave must not be forced to zero with centimetre
            // precision and leak into attitude/IMU-bias corrections.
            profile.surface_constraint_variance = 1.0;

            // Surface craft normally receive continuous GPS and have stronger
            // horizontal manoeuvre disturbances than submerged vehicles.
            profile.ekf.q_pos = 0.02;
            profile.ekf.q_att = 0.002;
            profile.ekf.q_vel = 0.2;
            profile.ekf.q_medium_velocity = 0.005;
            // A floating hull is not a stationary gravity reference even at
            // zero surge: wave acceleration remains. Do not admit the static
            // leveling pseudo-observation in this dynamic platform profile.
            // Roll/pitch propagate with the IMU and aided navigation model.
            profile.ekf.level_max_body_speed_mps = 0.0;
            profile.ekf.level_max_gyro_radps = 0.0;
            profile.ekf.r_vertical = profile.surface_constraint_variance;
            profile.ekf.initial_position_variance = 2.0;
            profile.ekf.initial_attitude_variance = 0.25;
            // Preserve magnetometer reacquisition after a wave-driven yaw
            // transient exceeds the ordinary scalar innovation gate.
            profile.ekf.gate_heading_nis = 100.0;

            profile.ekf.initial_medium_velocity_variance = 2.0;
            return profile;
        }

        EstimationProfile ugv_profile()
        {
            EstimationProfile profile;
            profile.vehicle_class = VehicleClass::UGV_DIFFERENTIAL;
            profile.vertical_aid = VerticalAidMode::SurfaceConstraint;
            profile.medium_velocity_kind = MediumVelocityKind::None;
            profile.estimate_medium_velocity = false;
            profile.fuse_bottom_track_dvl = false;
            profile.fuse_relative_medium_velocity = false;
            profile.fuse_wheel_odometry = true;
            profile.ekf.q_pos = 0.03;
            profile.ekf.q_att = 0.003;
            profile.ekf.q_vel = 0.25;
            profile.ekf.q_medium_velocity = 0.0;
            profile.ekf.r_vertical = profile.surface_constraint_variance;
            profile.ekf.r_gps_xy = 0.25;
            profile.ekf.r_gps_velocity = 0.09;
            profile.ekf.initial_position_variance = 2.0;
            profile.ekf.initial_attitude_variance = 0.25;
            return profile;
        }

        EstimationProfile uav_profile(VehicleClass vehicle_class)
        {
            EstimationProfile profile;
            profile.vehicle_class = vehicle_class;
            profile.vertical_aid = VerticalAidMode::BarometerAndGps;
            profile.ekf.r_vertical = 0.25;
            profile.medium_velocity_kind = MediumVelocityKind::Wind;
            // Wind is named explicitly but remains invalid until an airspeed or
            // aerodynamic-relative-velocity observation is implemented.
            profile.estimate_medium_velocity = false;
            profile.fuse_bottom_track_dvl = false;
            profile.fuse_relative_medium_velocity = false;

            profile.ekf.q_pos = 0.05;
            profile.ekf.q_att = 0.005;
            profile.ekf.q_vel = 0.5;
            profile.ekf.q_medium_velocity = 0.0;
            profile.ekf.r_gps_xy = 0.25;
            profile.ekf.r_gps_z = 0.5;
            profile.ekf.r_gps_velocity = 0.09;
            // Accelerometer attitude is a valid gravity observation only
            // while the airframe is essentially stationary. During flight,
            // translational specific force must not be mistaken for tilt.
            // Keep this gate deliberately tighter than the hover controller's
            // recapture speed. With a low-rate GPS aid, using 0.25 m/s here
            // can classify a real lateral manoeuvre as stationary for nearly
            // one GPS period and pull roll/pitch away from gyro propagation.
            // The resulting attitude and velocity lag creates a position-hold
            // limit cycle even when the controller itself is well damped.
            profile.ekf.level_max_body_speed_mps = 0.05;
            profile.ekf.level_max_gyro_radps = 0.02;
            profile.ekf.initial_position_variance = 4.0;
            profile.ekf.initial_attitude_variance = 0.25;
            profile.ekf.initial_velocity_variance = 4.0;
            profile.ekf.initial_medium_velocity_variance = 25.0;
            return profile;
        }
    } // namespace

    EstimationProfile estimation_profile_for(VehicleClass vehicle_class)
    {
        switch (vehicle_class)
        {
        case VehicleClass::USV:
            return usv_profile();
        case VehicleClass::UGV_DIFFERENTIAL:
            return ugv_profile();
        case VehicleClass::UAV_MULTIROTOR:
        case VehicleClass::UAV_FIXED_WING:
        case VehicleClass::UAV_VTOL:
            return uav_profile(vehicle_class);
        case VehicleClass::UUV:
        default:
            return uuv_profile();
        }
    }

    const char *vertical_aid_mode_name(VerticalAidMode mode)
    {
        switch (mode)
        {
        case VerticalAidMode::BarometerAndGps:
            return "barometer_and_gps";
        case VerticalAidMode::SurfaceConstraint:
            return "surface_constraint";
        case VerticalAidMode::GpsAltitude:
            return "gps_altitude";
        case VerticalAidMode::PressureDepth:
        default:
            return "pressure_depth";
        }
    }

    const char *medium_velocity_kind_name(MediumVelocityKind kind)
    {
        switch (kind)
        {
        case MediumVelocityKind::WaterCurrent:
            return "water_current";
        case MediumVelocityKind::Wind:
            return "wind";
        case MediumVelocityKind::None:
        default:
            return "none";
        }
    }
} // namespace hydrox
