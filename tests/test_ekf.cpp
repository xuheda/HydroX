// Copyright (c) 2026 OceanX. Author: xuheda
/**
 * test_ekf.cpp — Offline unit test pinning down the sign conventions and observability of aided strapdown INS.
 *
 * Covers:
 *   1. Static and level (f_b=[0,0,-g], omega=0, DVL=0) -> zero drift in position/attitude/velocity.
 *   2. Missing specific force (have_accel=false) kinematic degradation -> consistent with legacy behavior, no divergence.
 *   3. Accelerometer bias converges under DVL+leveling aiding (b_a is observable).
 *   4. Under actual roll tilt, leveling pulls the estimated attitude towards ground truth (roll is observable).
 *   5. Magnetometer heading aid pulls yaw towards magnetic north when mag is available.
 *   6. Water-track DVL plus GPS ground velocity estimates NED current without treating water speed as ground speed.
 *   7. Platform profiles select distinct covariance and environmental-state policies.
 *   8. UAV accelerometer leveling is inhibited during translational/rotational manoeuvres.
 *   9. GPS-only ground velocity cannot corrupt fixed-wing roll/pitch.
 *  10. Bottom-track DVL safely reacquires velocity after a long stationary hold.
 */
#include "ekf.h"

#include <cmath>
#include <cstdio>
#include <optional>

namespace
{
    using hydrox::EKF;
    using hydrox::EKF_G;

    constexpr double kPi = 3.14159265358979323846;

    bool approx(double a, double b, double eps)
    {
        return std::abs(a - b) <= eps;
    }

    int expect(bool ok, const char *msg)
    {
        if (!ok)
        {
            std::fprintf(stderr, "FAIL: %s\n", msg);
            return 1;
        }
        return 0;
    }

    // When static and level, the accelerometer reading is f_b = [0, 0, -g]
    const Eigen::Vector3d kLevelAccel{0.0, 0.0, -EKF_G};
    const Eigen::Vector3d kZeroOmega{0.0, 0.0, 0.0};

    hydrox::DVLMeasurement zero_dvl(double timestamp = 0.0)
    {
        hydrox::DVLMeasurement d;
        d.vel_x = d.vel_y = d.vel_z = 0.0;
        d.beam_valid = 4;
        d.timestamp = timestamp;
        return d;
    }

    hydrox::NavigationMeasurements navigation_measurements(
        const Eigen::Vector3d &accel,
        bool have_accel,
        const Eigen::Vector3d &gyro,
        const std::optional<hydrox::DVLMeasurement> &dvl,
        const std::optional<Eigen::Vector3d> &mag = std::nullopt)
    {
        hydrox::NavigationMeasurements meas;
        meas.accel_body.value = accel;
        meas.accel_body.meta.valid = have_accel;
        meas.gyro_body.value = gyro;
        meas.gyro_body.meta.valid = true;

        // The legacy test API always supplied a depth measurement of zero.
        meas.depth.value = 0.0;
        meas.depth.meta.valid = true;

        if (dvl && dvl->beam_valid > 0)
        {
            meas.dvl_velocity_body.value = dvl->velocity_body();
            meas.dvl_velocity_body.meta.valid = true;
        }
        if (mag)
        {
            meas.mag_body.value = *mag;
            meas.mag_body.meta.valid = true;
        }
        return meas;
    }
}

int main()
{
    int fails = 0;
    const std::optional<hydrox::GPSMeasurement> no_gps;
    const std::optional<hydrox::DVLMeasurement> no_dvl;
    const std::optional<hydrox::DVLMeasurement> no_water_dvl;

    // ── 1. Static and Level: Zero Drift ────────────────────────────────────────────────
    {
        EKF ekf;
        hydrox::NavigationState init = hydrox::NavigationState::zeros();
        ekf.reset(init);

        auto dvl = zero_dvl();
        const auto meas = navigation_measurements(kLevelAccel, true, kZeroOmega, dvl);
        hydrox::NavigationState s;
        for (int i = 0; i < 2000; ++i) // 20 s @ 100 Hz
        {
            dvl.timestamp = static_cast<double>(i + 1) * 0.01;
            s = ekf.update(meas, dvl, no_water_dvl, no_gps, 0.01);
        }

        fails += expect(approx(s.eta[0], 0.0, 1e-3), "stationary: N drift");
        fails += expect(approx(s.eta[1], 0.0, 1e-3), "stationary: E drift");
        fails += expect(approx(s.eta[2], 0.0, 1e-3), "stationary: D drift");
        fails += expect(approx(s.eta[3], 0.0, 1e-3), "stationary: roll drift");
        fails += expect(approx(s.eta[4], 0.0, 1e-3), "stationary: pitch drift");
        fails += expect(approx(s.nu[0], 0.0, 1e-3), "stationary: u drift");
        fails += expect(approx(s.nu[1], 0.0, 1e-3), "stationary: v drift");
        fails += expect(approx(s.nu[2], 0.0, 1e-3), "stationary: w drift");
    }

    // ── 2. Missing Specific Force -> Kinematic Degradation, No Divergence ───────────────────────────────────
    {
        EKF ekf;
        ekf.reset(hydrox::NavigationState::zeros());
        auto dvl = zero_dvl();
        const auto meas = navigation_measurements(Eigen::Vector3d::Zero(), false,
                                                  kZeroOmega, dvl);
        hydrox::NavigationState s;
        for (int i = 0; i < 1000; ++i)
        {
            dvl.timestamp = static_cast<double>(i + 1) * 0.01;
            s = ekf.update(meas, dvl, no_water_dvl, no_gps, 0.01);
        }
        fails += expect(std::isfinite(s.eta[2]) && approx(s.eta[2], 0.0, 1e-2),
                        "no-accel fallback: depth stays bounded");
        fails += expect(approx(s.nu[2], 0.0, 1e-2),
                        "no-accel fallback: w stays bounded (driven by DVL)");
    }

    // ── 3. Vertical Accelerometer Bias Observable (Converges Under DVL Velocity Aiding) ─────────────────────
    {
        EKF ekf;
        hydrox::NavigationState init = hydrox::NavigationState::zeros();
        init.nu[0] = 1.0;
        ekf.reset(init);
        const auto meas = navigation_measurements(kLevelAccel, true, kZeroOmega, no_dvl);
        hydrox::NavigationState s;
        for (int i = 0; i < 100; ++i)
            s = ekf.update(meas, no_dvl, no_water_dvl, no_gps, 0.01);

        fails += expect(approx(s.nu[0], 1.0, 1e-2),
                        "accel/no-DVL: surge is not zeroed by ZVU");
        fails += expect(approx(s.eta[0], 1.0, 5e-2),
                        "accel/no-DVL: position propagates from velocity");
    }

    // Note: Horizontal bias at rest is indistinguishable from small tilt angles (leveling/tilt ambiguity),
    // so we choose the z-axis — it does not couple with attitude in a level state, and converges cleanly under velocity aiding.
    {
        EKF ekf;
        ekf.reset(hydrox::NavigationState::zeros());
        auto dvl = zero_dvl();
        // Actually static, but IMU has a +0.2 m/s² bias on the z-axis -> reading = ground truth + bias
        const Eigen::Vector3d biased = kLevelAccel + Eigen::Vector3d(0.0, 0.0, 0.2);
        const auto meas = navigation_measurements(biased, true, kZeroOmega, dvl);
        for (int i = 0; i < 6000; ++i) // 60 s
        {
            dvl.timestamp = static_cast<double>(i + 1) * 0.01;
            ekf.update(meas, dvl, no_water_dvl, no_gps, 0.01);
        }

        const double baz = ekf.state()[11];
        fails += expect(approx(baz, 0.2, 0.05),
                        "accel bias b_az converges to true 0.2");
    }

    // ── 4. Under Roll Tilt, Leveling Pulls Attitude Towards Ground Truth ─────────────────────────────
    {
        EKF ekf;
        ekf.reset(hydrox::NavigationState::zeros());
        auto dvl = zero_dvl();
        // Actually static, roll = +10 deg: body frame specific force = -R_nb^T g_n
        const double roll = 10.0 * kPi / 180.0;
        // R_nb^T g_n under roll-only = [0, -g*sin(phi), -g*cos(phi)]? Verified directly using h:
        // f = -R^T g, R^T g = [ -g*s_theta ; g*c_theta*s_phi ; g*c_theta*c_phi ], theta=0 -> [0; g*s_phi; g*c_phi]
        const Eigen::Vector3d accel_roll(0.0, -EKF_G * std::sin(roll),
                                         -EKF_G * std::cos(roll));
        const auto meas = navigation_measurements(accel_roll, true, kZeroOmega, dvl);
        hydrox::NavigationState s;
        for (int i = 0; i < 6000; ++i)
        {
            dvl.timestamp = static_cast<double>(i + 1) * 0.01;
            s = ekf.update(meas, dvl, no_water_dvl, no_gps, 0.01);
        }

        fails += expect(approx(s.eta[3], roll, 2.0 * kPi / 180.0),
                        "leveling: roll converges to true +10deg");
        fails += expect(approx(s.eta[4], 0.0, 2.0 * kPi / 180.0),
                        "leveling: pitch stays ~0");
    }

    // 5. Magnetometer Heading Aid
    {
        EKF ekf;
        hydrox::NavigationState init = hydrox::NavigationState::zeros();
        init.eta[5] = 30.0 * kPi / 180.0;
        ekf.reset(init);
        auto dvl = zero_dvl();
        const Eigen::Vector3d mag_body(25.0, 0.0, 43.301270189);
        const std::optional<Eigen::Vector3d> mag = mag_body;
        const auto meas = navigation_measurements(kLevelAccel, true, kZeroOmega, dvl, mag);
        hydrox::NavigationState s;
        for (int i = 0; i < 1000; ++i)
        {
            dvl.timestamp = static_cast<double>(i + 1) * 0.01;
            s = ekf.update(meas, dvl, no_water_dvl, no_gps, 0.01);
        }

        fails += expect(approx(s.eta[5], 0.0, 2.0 * kPi / 180.0),
                        "mag heading: yaw converges to magnetic north");
    }
    // WAM-V wave yaw can create a healthy magnetometer residual larger than
    // the ordinary scalar gate. The USV profile must begin reacquisition.
    {
        const auto usv =
            hydrox::estimation_profile_for(hydrox::VehicleClass::USV);
        EKF ekf(usv);
        ekf.reset(hydrox::NavigationState::zeros());
        auto dvl = zero_dvl();
        const Eigen::Vector3d mag_body(25.0, 0.0, 43.301270189);
        const std::optional<Eigen::Vector3d> mag = mag_body;
        for (int i = 0; i < 100; ++i)
        {
            dvl.timestamp = static_cast<double>(i + 1) * 0.01;
            const auto nominal = navigation_measurements(
                kLevelAccel, true, kZeroOmega, dvl, mag);
            ekf.update(nominal, dvl, no_water_dvl, no_gps, 0.01);
        }

        const Eigen::Vector3d transient_yaw_rate(0.0, 0.0, 0.42);
        const auto recovery = navigation_measurements(
            kLevelAccel, true, transient_yaw_rate, dvl, mag);
        const auto recovered =
            ekf.update(recovery, dvl, no_water_dvl, no_gps, 1.0);
        fails += expect(
            ekf.last_stats().heading_accepted == 1 &&
                std::abs(recovered.eta[5]) < 0.35,
            "USV magnetometer: large healthy residual begins reacquisition");
    }

    // 6. Water-track DVL + GPS ground velocity observes the NED water current.
    // The vehicle is yawed +90 degrees, so this also pins the body/NED rotation
    // signs in the water-relative measurement model.
    {
        EKF::Params params;
        params.q_medium_velocity = 1.0e-6;
        params.r_relative_medium_velocity = 1.0e-4;
        params.r_gps_velocity = 1.0e-4;
        params.medium_velocity_valid_std = 0.5;
        EKF ekf(params);

        hydrox::NavigationState init = hydrox::NavigationState::zeros();
        init.eta[5] = 0.5 * kPi;
        ekf.reset(init);
        const auto initial_covariance = ekf.covariance();
        fails += expect(
            initial_covariance.allFinite() &&
                approx(initial_covariance(0, 0), 1.0, 1e-12) &&
                approx(initial_covariance(14, 14), params.initial_gyro_bias_variance, 1e-12) &&
                approx(initial_covariance(15, 15), 4.0, 1e-12),
            "water-track: default initial covariance is preserved");

        const Eigen::Vector3d ground_velocity_body(1.2, -0.4, 0.1);
        const Eigen::Vector3d current_ned(0.3, 0.5, -0.1);
        const Eigen::Vector3d gps_velocity_ned(0.4, 1.2, 0.1);
        // R_nb^T * current_ned at yaw +90deg is [0.5, -0.3, -0.1].
        const Eigen::Vector3d water_velocity_body(0.7, -0.1, 0.2);

        hydrox::DVLMeasurement water_dvl;
        water_dvl.vel_x = water_velocity_body.x();
        water_dvl.vel_y = water_velocity_body.y();
        water_dvl.vel_z = water_velocity_body.z();
        water_dvl.beam_valid = 4;
        water_dvl.tracking_mode = 2;

        hydrox::GPSMeasurement gps;
        gps.has_velocity = true;

        hydrox::NavigationMeasurements meas;
        meas.gyro_body.meta.valid = true;
        meas.gyro_body.value = kZeroOmega;
        meas.water_dvl_velocity_body.meta.valid = true;
        meas.water_dvl_velocity_body.value = water_velocity_body;
        meas.water_dvl_velocity_body.covariance =
            1.0e-4 * Eigen::Matrix3d::Identity();
        meas.gps_position_ned.meta.valid = true;
        meas.gps_position_ned.covariance =
            0.1 * Eigen::Matrix3d::Identity();
        meas.gps_velocity_ned.meta.valid = true;
        meas.gps_velocity_ned.value = gps_velocity_ned;
        meas.gps_velocity_ned.covariance =
            1.0e-4 * Eigen::Matrix3d::Identity();
        // Ground course differs from hull heading in this sideslip case.
        meas.mag_body.meta.valid = true;
        meas.mag_body.value = Eigen::Vector3d(0.0, -25.0, 43.3012701892);

        hydrox::NavigationState s;
        for (int i = 0; i < 80; ++i)
        {
            const double t = static_cast<double>(i + 1) * 0.1;
            water_dvl.timestamp = t;
            gps.timestamp = t;
            gps.pos_n = gps_velocity_ned.x() * t;
            gps.pos_e = gps_velocity_ned.y() * t;
            gps.vel_n = gps_velocity_ned.x();
            gps.vel_e = gps_velocity_ned.y();
            gps.vel_d = gps_velocity_ned.z();
            s = ekf.update(meas, no_dvl, water_dvl, gps, 0.1);
        }

        if ((s.medium_velocity_ned - current_ned).norm() >= 0.08 ||
            (s.nu.segment<3>(0) - ground_velocity_body).norm() >= 0.08)
        {
            std::fprintf(
                stderr,
                "water-track diagnostic: medium=(%.6f, %.6f, %.6f) "
                "velocity=(%.6f, %.6f, %.6f)\n",
                s.medium_velocity_ned.x(),
                s.medium_velocity_ned.y(),
                s.medium_velocity_ned.z(),
                s.nu[0],
                s.nu[1],
                s.nu[2]);
        }
        fails += expect((s.medium_velocity_ned - current_ned).norm() < 0.08,
                        "water-track: NED current converges with GPS velocity");
        fails += expect((s.nu.segment<3>(0) - ground_velocity_body).norm() < 0.08,
                        "water-track: state velocity remains body-frame ground velocity");
        fails += expect(s.medium_velocity_valid,
                        "water-track: current validity follows covariance after observation");
        fails += expect(
            s.medium_velocity_kind == hydrox::MediumVelocityKind::WaterCurrent,
            "water-track: generic medium state is labelled as water current");

        // Mutating a duplicate frame must not change the filter: both DVL and
        // GPS measurements are consumed by their HIL timestamp exactly once.
        const Eigen::Vector3d current_before_duplicate = s.medium_velocity_ned;
        meas.mag_body.meta.valid = false;
        water_dvl.vel_x += 5.0;
        gps.vel_n += 5.0;
        s = ekf.update(meas, no_dvl, water_dvl, gps, 0.1);
        fails += expect((s.medium_velocity_ned - current_before_duplicate).norm() < 1.0e-9 &&
                            ekf.last_stats().water_dvl_accepted == 0 &&
                            ekf.last_stats().gps_velocity_accepted == 0,
                        "water-track: duplicate timestamps are never fused twice");
    }

    // 7. Platform estimation profiles configure the common EKF without
    // branching the filter implementation.
    {
        const auto uuv =
            hydrox::estimation_profile_for(hydrox::VehicleClass::UUV);
        const auto usv =
            hydrox::estimation_profile_for(hydrox::VehicleClass::USV);
        const auto uav =
            hydrox::estimation_profile_for(hydrox::VehicleClass::UAV_FIXED_WING);

        fails += expect(
            uuv.vertical_aid == hydrox::VerticalAidMode::PressureDepth &&
                uuv.medium_velocity_kind ==
                    hydrox::MediumVelocityKind::WaterCurrent &&
                uuv.estimate_medium_velocity &&
                approx(uuv.ekf.level_max_body_speed_mps, 0.25, 1e-12) &&
                approx(uuv.ekf.level_max_gyro_radps, 0.10, 1e-12),
            "profile: UUV uses pressure depth/current and stationary-only leveling");
        fails += expect(
            usv.vertical_aid == hydrox::VerticalAidMode::SurfaceConstraint &&
                usv.ekf.q_vel != uuv.ekf.q_vel &&
                approx(usv.ekf.gate_heading_nis, 100.0, 1e-12) &&
                approx(usv.ekf.level_max_body_speed_mps, 0.0, 1e-12) &&
                approx(usv.ekf.level_max_gyro_radps, 0.0, 1e-12),
            "profile: USV does not treat a floating hull as a static gravity reference");
        fails += expect(
            uav.vertical_aid == hydrox::VerticalAidMode::BarometerAndGps &&
                uav.medium_velocity_kind == hydrox::MediumVelocityKind::Wind &&
                !uav.estimate_medium_velocity &&
                !uav.fuse_bottom_track_dvl &&
                !uav.fuse_relative_medium_velocity &&
                approx(uav.ekf.level_max_body_speed_mps, 0.05, 1e-12) &&
                approx(uav.ekf.level_max_gyro_radps, 0.02, 1e-12),
            "profile: UAV uses barometer/GPS altitude and disables unobservable wind/DVL");

        EKF usv_ekf(usv);
        EKF uav_ekf(uav);
        fails += expect(
            approx(usv_ekf.covariance()(0, 0),
                   usv.ekf.initial_position_variance, 1e-12) &&
                approx(usv_ekf.covariance()(3, 3),
                       usv.ekf.initial_attitude_variance, 1e-12),
            "profile: EKF applies platform initial covariance");
        fails += expect(
            uav_ekf.medium_velocity_kind() ==
                    hydrox::MediumVelocityKind::Wind &&
                !uav_ekf.estimates_medium_velocity(),
            "profile: UAV keeps wind semantics but never reports it observable");
    }

    // 8. During a UAV pitch manoeuvre, a near-g accelerometer norm does not
    // prove that specific force is gravity-only. The angular-rate gate must
    // keep static leveling from cancelling valid gyro integration.
    {
        const auto uav =
            hydrox::estimation_profile_for(hydrox::VehicleClass::UAV_MULTIROTOR);
        EKF ekf(uav);
        ekf.reset(hydrox::NavigationState::zeros());

        const Eigen::Vector3d pitch_rate(0.0, 0.20, 0.0);
        const auto meas = navigation_measurements(
            kLevelAccel, true, pitch_rate, no_dvl);
        const hydrox::NavigationState s =
            ekf.update(meas, no_dvl, no_water_dvl, no_gps, 0.1);

        fails += expect(
            s.eta[4] > 0.015 && ekf.last_stats().level_rejected == 1 &&
                ekf.last_stats().level_accepted == 0,
            "UAV maneuver: static leveling cannot cancel gyro pitch integration");
    }

    // A low-rate GPS sample can temporarily under-report velocity during a
    // hover correction. That must not reopen accelerometer leveling and turn
    // translational specific force into a false roll/pitch observation.
    {
        const auto uav =
            hydrox::estimation_profile_for(hydrox::VehicleClass::UAV_MULTIROTOR);
        EKF ekf(uav);
        auto moving = hydrox::NavigationState::zeros();
        moving.nu[0] = 0.08;
        ekf.reset(moving);

        const Eigen::Vector3d lateral_specific_force(0.0, 1.0, -9.80665);
        const auto meas = navigation_measurements(
            lateral_specific_force, true, Eigen::Vector3d::Zero(), no_dvl);
        const hydrox::NavigationState s =
            ekf.update(meas, no_dvl, no_water_dvl, no_gps, 0.1);

        fails += expect(
            ekf.last_stats().level_rejected == 1 &&
                ekf.last_stats().level_accepted == 0 &&
                std::abs(s.eta[3]) < 1.0e-4,
            "UAV hover correction: estimated motion keeps static leveling closed");
    }

    // 9. The same gravity-only observation constraint applies underwater:
    // centripetal/hydrodynamic acceleration must not be interpreted as pitch.
    {
        const auto uuv =
            hydrox::estimation_profile_for(hydrox::VehicleClass::UUV);
        EKF ekf(uuv);
        ekf.reset(hydrox::NavigationState::zeros());

        const Eigen::Vector3d pitch_rate(0.0, 0.20, 0.0);
        const auto meas = navigation_measurements(
            kLevelAccel, true, pitch_rate, no_dvl);
        const hydrox::NavigationState s =
            ekf.update(meas, no_dvl, no_water_dvl, no_gps, 0.1);

        fails += expect(
            s.eta[4] > 0.015 && ekf.last_stats().level_rejected == 1 &&
                ekf.last_stats().level_accepted == 0,
            "UUV maneuver: static leveling cannot cancel gyro pitch integration");
    }

    // 10. GPS ground velocity observes R_nb*v_b, so attitude and body velocity
    // are inseparable unless DVL/wheel odometry supplies an independent body-
    // frame velocity observation. A fixed wing has no such aid: a launch-speed
    // innovation must correct velocity without tilting the aircraft estimate.
    {
        auto uav =
            hydrox::estimation_profile_for(hydrox::VehicleClass::UAV_FIXED_WING);
        uav.ekf.gate_gps_velocity_nis = 1.0e6;
        EKF ekf(uav);

        auto init = hydrox::NavigationState::zeros();
        init.eta[4] = 0.25;
        init.nu[0] = 8.0;
        ekf.reset(init);

        auto meas = navigation_measurements(
            kLevelAccel, true, kZeroOmega, no_dvl);
        meas.gps_position_ned.meta.valid = true;
        meas.gps_position_ned.covariance =
            0.25 * Eigen::Matrix3d::Identity();
        meas.gps_velocity_ned.meta.valid = true;
        meas.gps_velocity_ned.covariance =
            0.09 * Eigen::Matrix3d::Identity();

        hydrox::GPSMeasurement gps;
        gps.timestamp = 1.0;
        gps.has_velocity = true;
        gps.has_altitude = true;
        gps.pos_n = gps.pos_e = gps.pos_d = 0.0;
        gps.vel_n = 12.0;
        gps.vel_e = gps.vel_d = 0.0;

        const auto s = ekf.update(meas, no_dvl, no_water_dvl, gps, 0.0);
        fails += expect(
            approx(s.eta[4], init.eta[4], 1.0e-9),
            "UAV GPS velocity: launch innovation cannot alter pitch without body-speed aid");
        fails += expect(
            ekf.last_stats().gps_velocity_accepted == 1 &&
                (s.nu.segment<3>(0) - init.nu.segment<3>(0)).norm() > 0.1,
            "UAV GPS velocity: launch innovation still corrects body velocity");
    }

    // 11. The SITL air-launch contract releases a fixed wing from a frozen
    // spawn hold at cruise speed. IMU specific force cannot observe that
    // instantaneous state transition, so several mutually consistent GPS
    // samples must safely reacquire position/velocity after the NIS gate has
    // rejected isolated innovations.
    {
        const auto uav =
            hydrox::estimation_profile_for(hydrox::VehicleClass::UAV_FIXED_WING);
        EKF ekf(uav);
        auto init = hydrox::NavigationState::zeros();
        init.eta[0] = -100.0;
        init.eta[2] = -45.0;
        ekf.reset(init);

        hydrox::NavigationMeasurements meas;
        meas.gyro_body.meta.valid = true;
        meas.gyro_body.value = kZeroOmega;
        meas.gps_position_ned.meta.valid = true;
        meas.gps_position_ned.covariance =
            0.25 * Eigen::Matrix3d::Identity();
        meas.gps_velocity_ned.meta.valid = true;
        meas.gps_velocity_ned.covariance =
            0.09 * Eigen::Matrix3d::Identity();

        hydrox::GPSMeasurement gps;
        gps.has_velocity = true;
        gps.has_altitude = true;
        gps.timestamp = 0.1;
        gps.pos_n = -100.0;
        gps.pos_e = 0.0;
        gps.pos_d = -45.0;
        gps.vel_n = gps.vel_e = gps.vel_d = 0.0;
        (void)ekf.update(meas, no_dvl, no_water_dvl, gps, 0.1);

        hydrox::NavigationState recovered;
        for (int sample = 1; sample <= 3; ++sample)
        {
            gps.timestamp = 0.1 + 0.2 * sample;
            gps.pos_n = -100.0 + 12.0 * 0.2 * sample;
            gps.vel_n = 12.0;
            recovered = ekf.update(
                meas, no_dvl, no_water_dvl, gps, 0.2);
            if (sample < 3)
            {
                fails += expect(
                    ekf.last_stats().gps_velocity_rejected == 1 &&
                        ekf.last_stats().gps_velocity_accepted == 0,
                    "GPS reacquisition: isolated launch innovations remain gated");
            }
        }
        fails += expect(
            ekf.last_stats().gps_velocity_accepted == 1 &&
                std::abs(recovered.eta[0] - gps.pos_n) < 1.0e-9 &&
                std::abs(recovered.nu[0] - 12.0) < 1.0e-9,
            "GPS reacquisition: three consistent launch samples reset observed navigation states");
        fails += expect(ekf.last_stats().gps_xy_accepted == 1,
            "GPS reacquisition: accepted position reset refreshes runtime navigation health in the same tick");

        gps.timestamp += 0.2;
        gps.vel_n = 80.0;
        const auto one_outlier =
            ekf.update(meas, no_dvl, no_water_dvl, gps, 0.2);
        fails += expect(
            ekf.last_stats().gps_velocity_rejected == 1 &&
                std::abs(one_outlier.nu[0] - 80.0) > 20.0,
            "GPS reacquisition: one plausible but inconsistent outlier cannot reset velocity");
    }

    // 12. A long stationary hold makes velocity covariance tight. Three
    // consecutive, physically consistent bottom-track samples must recover a
    // real departure that crosses the ordinary NIS gate; one outlier must not.
    {
        const auto uuv =
            hydrox::estimation_profile_for(hydrox::VehicleClass::UUV);
        EKF ekf(uuv);
        ekf.reset(hydrox::NavigationState::zeros());

        auto dvl = zero_dvl();
        auto meas = navigation_measurements(
            kLevelAccel, true, kZeroOmega, dvl);
        for (int i = 0; i < 100; ++i)
        {
            dvl.timestamp = static_cast<double>(i + 1) * 0.2;
            meas.dvl_velocity_body.value = dvl.velocity_body();
            (void)ekf.update(meas, dvl, no_water_dvl, no_gps, 0.2);
        }

        dvl.timestamp += 0.2;
        dvl.vel_x = 8.0;
        meas.dvl_velocity_body.value = dvl.velocity_body();
        const auto isolated =
            ekf.update(meas, dvl, no_water_dvl, no_gps, 0.2);
        fails += expect(
            ekf.last_stats().dvl_rejected == 1 &&
                ekf.last_stats().dvl_reacquired == 0 &&
                std::abs(isolated.nu[0] - 8.0) > 1.0,
            "DVL reacquisition: one departure-sized innovation remains gated");

        EKF departure_ekf(uuv);
        departure_ekf.reset(hydrox::NavigationState::zeros());
        dvl = zero_dvl();
        meas = navigation_measurements(kLevelAccel, true, kZeroOmega, dvl);
        for (int i = 0; i < 100; ++i)
        {
            dvl.timestamp = static_cast<double>(i + 1) * 0.2;
            meas.dvl_velocity_body.value = dvl.velocity_body();
            (void)departure_ekf.update(
                meas, dvl, no_water_dvl, no_gps, 0.2);
        }

        hydrox::NavigationState recovered;
        for (int sample = 1; sample <= 3; ++sample)
        {
            dvl.timestamp = 20.0 + 0.2 * sample;
            dvl.vel_x = static_cast<double>(sample);
            meas.dvl_velocity_body.value = dvl.velocity_body();
            recovered = departure_ekf.update(
                meas, dvl, no_water_dvl, no_gps, 0.2);
            if (sample < 3)
            {
                fails += expect(
                    departure_ekf.last_stats().dvl_rejected == 1 &&
                        departure_ekf.last_stats().dvl_reacquired == 0,
                    "DVL reacquisition: fewer than three samples remain gated");
            }
        }
        fails += expect(
            departure_ekf.last_stats().dvl_accepted == 1 &&
                departure_ekf.last_stats().dvl_reacquired == 1 &&
                std::abs(recovered.nu[0] - 3.0) < 1.0e-9,
            "DVL reacquisition: three consistent departure samples reset body velocity");
    }

    // Small surge and angular rate do not make a wave-driven hull a static
    // gravity reference. This used to admit spurious attitude corrections
    // when the boat briefly slowed down at a waypoint.
    {
        EKF filter(hydrox::estimation_profile_for(hydrox::VehicleClass::USV));
        auto initial = hydrox::NavigationState::zeros();
        initial.nu[0] = 0.10;
        filter.reset(initial);
        auto meas = navigation_measurements(Eigen::Vector3d(0, 0.4, -EKF_G), true,
            Eigen::Vector3d(0.01, 0, 0), no_dvl);
        filter.update(meas, no_dvl, no_water_dvl, no_gps, 0.01);
        fails += expect(filter.last_stats().level_rejected == 1 &&
                        filter.last_stats().level_accepted == 0,
                        "slow wave-driven USV must not enable gravity-only leveling");
    }

    // A floating boat heaves with waves; its mean-surface pseudo-observation
    // is not a precise, instantaneous depth measurement.
    {
        const auto profile = hydrox::estimation_profile_for(hydrox::VehicleClass::USV);
        EKF filter(profile);
        auto initial = hydrox::NavigationState::zeros();
        initial.eta[5] = 0.5 * kPi;
        initial.nu.head<3>() = Eigen::Vector3d(0.0, -0.6, 0.4);
        filter.reset(initial);
        double max_error = 0.0;
        for (int step = 1; step <= 6000; ++step)
        {
            const double t = step * 0.01;
            const double phi = 0.08 * std::sin(2.0*t);
            const double theta = 0.05 * std::sin(1.7*t);
            const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(0.5*kPi, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX())).toRotationMatrix();
            const Eigen::Vector3d gyro(0.16 * std::cos(2*t),
                std::cos(phi)*0.085*std::cos(1.7*t), -std::sin(phi)*0.085*std::cos(1.7*t));
            auto meas = navigation_measurements(rotation.transpose()*Eigen::Vector3d(0,0,-0.8*std::sin(2*t)-EKF_G),
                true, gyro, no_dvl, rotation.transpose()*Eigen::Vector3d(25,0,43.3012701892));
            meas.depth.variance = profile.surface_constraint_variance;
            meas.depth.meta.source = hydrox::NavMeasurementSource::SurfaceConstraint;
            std::optional<hydrox::GPSMeasurement> gps;
            if (step % 20 == 0)
            {
                gps.emplace(); gps->timestamp = t;
                gps->pos_n = 0.6*t; gps->pos_e = 0;
                gps->has_velocity = true;
                gps->vel_n = 0.6; gps->vel_e = 0; gps->vel_d = 0.4*std::cos(2*t);
                meas.gps_position_ned.meta.valid = meas.gps_velocity_ned.meta.valid = true;
            }
            const auto state = filter.update(meas, no_dvl, no_water_dvl, gps, 0.01);
            max_error = std::max(max_error, std::hypot(state.eta[0]-0.6*t,state.eta[1]));
        }
        std::printf("wave navigation max horizontal error %.6f m\n", max_error);
        fails += expect(max_error < 2.0, "USV wave motion: mean-surface aid must not corrupt horizontal navigation");
    }

    // Nonzero 3-D rotations exercise owned Eigen prediction values. Returning
    // a lazy expression from a lambda references a destroyed rotation matrix.
    for (int sample = 0; sample < 80; ++sample)
    {
        const double yaw = -2.8 + 0.07 * sample;
        const double roll = 0.2 * std::sin(sample);
        const double pitch = 0.15 * std::cos(sample);
        const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
        auto initial = hydrox::NavigationState::zeros();
        initial.eta << 1200.0, 200.0, 0.0, roll, pitch, yaw;
        initial.nu.head<3>() = Eigen::Vector3d(0.7, -0.25, 0.12);
        initial.medium_velocity_ned = Eigen::Vector3d(0.3, -0.1, 0.05);
        const Eigen::Vector3d velocity = rotation * initial.nu.head<3>();
        auto meas = navigation_measurements(rotation.transpose() * kLevelAccel, true, kZeroOmega, no_dvl);
        meas.mag_body.meta.valid = true;
        meas.mag_body.value = rotation.transpose() * Eigen::Vector3d(25.0, 0.0, 43.3012701892);
        meas.depth.meta.valid = false;
        meas.gps_position_ned.meta.valid = true;
        meas.gps_velocity_ned.meta.valid = true;
        hydrox::GPSMeasurement gps;
        gps.pos_n = initial.eta[0] + velocity.x() * 0.01;
        gps.pos_e = initial.eta[1] + velocity.y() * 0.01;
        gps.vel_n = velocity.x(); gps.vel_e = velocity.y(); gps.vel_d = velocity.z();
        gps.has_velocity = true; gps.timestamp = 1.0;
        EKF gps_filter(hydrox::estimation_profile_for(hydrox::VehicleClass::USV));
        gps_filter.reset(initial);
        auto result = gps_filter.update(meas, no_dvl, no_water_dvl, gps, 0.01);
        fails += expect(gps_filter.last_stats().gps_velocity_accepted == 1 &&
            (result.nu.head<3>() - initial.nu.head<3>()).norm() < 1.e-8,
            "GPS rotation prediction: exact 3-D velocity must have zero innovation");

        hydrox::DVLMeasurement water;
        const Eigen::Vector3d relative = initial.nu.head<3>() - rotation.transpose() * initial.medium_velocity_ned;
        water.vel_x = relative.x(); water.vel_y = relative.y(); water.vel_z = relative.z();
        water.beam_valid = 4; water.timestamp = 1.0;
        meas.gps_position_ned.meta.valid = false;
        meas.gps_velocity_ned.meta.valid = false;
        meas.water_dvl_velocity_body.meta.valid = true;
        meas.water_dvl_velocity_body.value = relative;
        EKF water_filter;
        water_filter.reset(initial);
        result = water_filter.update(meas, no_dvl, water, no_gps, 0.01);
        fails += expect(water_filter.last_stats().water_dvl_accepted == 1 &&
            (result.nu.head<3>() - initial.nu.head<3>()).norm() < 1.e-8,
            "water-track rotation prediction: exact 3-D velocity must have zero innovation");
    }

    if (fails == 0)
        std::printf("test_ekf: all checks passed\n");
    return fails == 0 ? 0 : 1;
}
