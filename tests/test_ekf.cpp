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
                approx(initial_covariance(14, 14), 1.0, 1e-12) &&
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
                uuv.estimate_medium_velocity,
            "profile: UUV uses pressure depth and estimates water current");
        fails += expect(
            usv.vertical_aid == hydrox::VerticalAidMode::SurfaceConstraint &&
                usv.ekf.q_vel != uuv.ekf.q_vel,
            "profile: USV uses a surface constraint and platform tuning");
        fails += expect(
            uav.vertical_aid == hydrox::VerticalAidMode::GpsAltitude &&
                uav.medium_velocity_kind == hydrox::MediumVelocityKind::Wind &&
                !uav.estimate_medium_velocity &&
                !uav.fuse_bottom_track_dvl &&
                !uav.fuse_relative_medium_velocity,
            "profile: UAV uses GPS altitude and disables unobservable wind/DVL");

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

    if (fails == 0)
        std::printf("test_ekf: all checks passed\n");
    return fails == 0 ? 0 : 1;
}
