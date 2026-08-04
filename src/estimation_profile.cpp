#include "estimation_profile.h"

namespace hydrox
{
    namespace
    {
        EstimationProfile uuv_profile()
        {
            // Historical HydroX tuning is retained as the UUV baseline.
            return {};
        }

        EstimationProfile usv_profile()
        {
            EstimationProfile profile;
            profile.vehicle_class = VehicleClass::USV;
            profile.vertical_aid = VerticalAidMode::SurfaceConstraint;
            profile.medium_velocity_kind = MediumVelocityKind::WaterCurrent;

            // Surface craft normally receive continuous GPS and have stronger
            // horizontal manoeuvre disturbances than submerged vehicles.
            profile.ekf.q_pos = 0.02;
            profile.ekf.q_att = 0.002;
            profile.ekf.q_vel = 0.2;
            profile.ekf.q_medium_velocity = 0.005;
            profile.ekf.r_vertical = profile.surface_constraint_variance;
            profile.ekf.initial_position_variance = 2.0;
            profile.ekf.initial_attitude_variance = 0.25;
            profile.ekf.initial_medium_velocity_variance = 2.0;
            return profile;
        }

        EstimationProfile uav_profile(VehicleClass vehicle_class)
        {
            EstimationProfile profile;
            profile.vehicle_class = vehicle_class;
            profile.vertical_aid = VerticalAidMode::GpsAltitude;
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
