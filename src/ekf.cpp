/**
 * ekf.cpp - Hybrid aided EKF implementation.
 * See ekf.h for state layout, mechanized prediction, and kinematic fallback semantics.
 */
#include "ekf.h"
#include <Eigen/Dense> // Includes LU (inverse), Core, and all other modules
#include <algorithm>
#include <cmath>

namespace hydrox {

	template <int M>
	bool EKF::_apply_linear_update(
		const Eigen::Matrix<double, M, EKF_N>& H,
		const Eigen::Matrix<double, M, 1>&	 innovation,
		const Eigen::Matrix<double, M, M>&	 R,
		double								 nis_gate,
		double*								 nis_out) {
		// The observation dimension is at most three. Form P*H^T once, then
		// apply P <- P - K*(P*H^T)^T directly. This is algebraically the same
		// update as (I-KH)P, but it avoids materialising several 18x18 Eigen
		// temporaries on the flight-control task stack.
		Eigen::Matrix<double, EKF_N, M> PHt;
		for (int i = 0; i < EKF_N; ++i) {
			for (int a = 0; a < M; ++a) {
				double value = 0.0;
				for (int j = 0; j < EKF_N; ++j)
					value += _covariance(i, j) * H(a, j);
				PHt(i, a) = value;
			}
		}

		Eigen::Matrix<double, M, M> S = R;
		for (int a = 0; a < M; ++a) {
			for (int b = 0; b < M; ++b) {
				for (int j = 0; j < EKF_N; ++j)
					S(a, b) += H(a, j) * PHt(j, b);
			}
		}
		const Eigen::Matrix<double, M, M> S_inverse = S.inverse();
		if (!S_inverse.allFinite()) {
			if (nis_out)
				*nis_out = std::numeric_limits<double>::infinity();
			return false;
		}

		double nis = 0.0;
		for (int a = 0; a < M; ++a) {
			for (int b = 0; b < M; ++b)
				nis += innovation[a] * S_inverse(a, b) * innovation[b];
		}
		if (nis_out)
			*nis_out = nis;
		if (!std::isfinite(nis) || nis > nis_gate)
			return false;

		Eigen::Matrix<double, EKF_N, M> K;
		for (int i = 0; i < EKF_N; ++i) {
			for (int a = 0; a < M; ++a) {
				double value = 0.0;
				for (int b = 0; b < M; ++b)
					value += PHt(i, b) * S_inverse(b, a);
				K(i, a) = value;
			}
		}
		for (int i = 0; i < EKF_N; ++i) {
			for (int a = 0; a < M; ++a)
				_x[i] += K(i, a) * innovation[a];
		}

		// K*PHt^T is symmetric in exact arithmetic. Average its mirrored
		// entries while updating the upper triangle so round-off cannot make P
		// asymmetric between prediction steps.
		for (int i = 0; i < EKF_N; ++i) {
			for (int j = i; j < EKF_N; ++j) {
				double correction_ij = 0.0;
				double correction_ji = 0.0;
				for (int a = 0; a < M; ++a) {
					correction_ij += K(i, a) * PHt(j, a);
					correction_ji += K(j, a) * PHt(i, a);
				}
				const double updated = _covariance(i, j) -
					0.5 * (correction_ij + correction_ji);
				_covariance(i, j) = updated;
				_covariance(j, i) = updated;
			}
		}
		return true;
	}

	EKF::EKF(const Params& p) {
		// Process noise Q (diagonal, blocked by state)
		Eigen::Matrix<double, EKF_N, 1> q_diag;
		q_diag << p.q_pos, p.q_pos, p.q_pos, p.q_att, p.q_att, p.q_att, p.q_vel,
			p.q_vel, p.q_vel, p.q_ba, p.q_ba, p.q_ba, p.q_bg, p.q_bg, p.q_bg,
			p.q_medium_velocity, p.q_medium_velocity, p.q_medium_velocity;
		_Q = q_diag.asDiagonal();

		_R_dvl = p.r_dvl * Eigen::Matrix3d::Identity();
		_R_water_dvl = p.r_relative_medium_velocity * Eigen::Matrix3d::Identity();
		_R_zvu = p.r_zvu * Eigen::Matrix3d::Identity();
		_R_depth << p.r_vertical;
		_R_gps = p.r_gps_xy * Eigen::Matrix2d::Identity();
		_R_gps_z << p.r_gps_z;
		_R_gps_velocity = p.r_gps_velocity * Eigen::Matrix3d::Identity();
		_R_heading << p.r_heading;
		_R_level = p.r_level * Eigen::Matrix3d::Identity();
		_level_gate = p.level_gate;
		_level_max_body_speed_mps = std::max(0.0, p.level_max_body_speed_mps);
		_level_max_gyro_radps = std::max(0.0, p.level_max_gyro_radps);
		_gate_dvl_nis = p.gate_dvl_nis;
		_gate_water_dvl_nis = p.gate_relative_medium_velocity_nis;
		_gate_gps_velocity_nis = p.gate_gps_velocity_nis;
		_gate_level_nis = p.gate_level_nis;
		_gate_gps_xy_nis = p.gate_gps_xy_nis;
		_gate_scalar_nis = p.gate_scalar_nis;
		_gate_heading_nis = p.gate_heading_nis;
		_dvl_velocity_reacquire_samples =
			std::max(0, p.dvl_velocity_reacquire_samples);
		_dvl_velocity_reacquire_consistency_mps =
			std::max(0.0, p.dvl_velocity_reacquire_consistency_mps);
		_dvl_velocity_reacquire_max_interval_s =
			std::max(0.0, p.dvl_velocity_reacquire_max_interval_s);
		_dvl_velocity_reacquire_max_speed_mps =
			std::max(0.0, p.dvl_velocity_reacquire_max_speed_mps);
		_min_variance = p.min_variance;
		_gps_navigation_reacquire_samples =
			std::max(0, p.gps_navigation_reacquire_samples);
		_gps_navigation_reacquire_consistency_mps =
			std::max(0.0, p.gps_navigation_reacquire_consistency_mps);
		_gps_navigation_reacquire_max_interval_s =
			std::max(0.0, p.gps_navigation_reacquire_max_interval_s);
		_gps_navigation_reacquire_max_speed_mps =
			std::max(0.0, p.gps_navigation_reacquire_max_speed_mps);
		_initial_variance_diag << p.initial_position_variance,
			p.initial_position_variance, p.initial_position_variance,
			p.initial_attitude_variance, p.initial_attitude_variance,
			p.initial_attitude_variance, p.initial_velocity_variance,
			p.initial_velocity_variance, p.initial_velocity_variance,
			p.initial_accel_bias_variance, p.initial_accel_bias_variance,
			p.initial_accel_bias_variance, p.initial_gyro_bias_variance,
			p.initial_gyro_bias_variance, p.initial_gyro_bias_variance,
			p.initial_medium_velocity_variance, p.initial_medium_velocity_variance,
			p.initial_medium_velocity_variance;
		for (int i = 0; i < EKF_N; ++i)
			_initial_variance_diag[i] =
				std::max(_initial_variance_diag[i], _min_variance);
		_medium_velocity_valid_std = std::max(p.medium_velocity_valid_std, 0.0);
		_x.setZero();
		_reset_covariance();
	}

	EKF::EKF(const EstimationProfile& profile) : EKF(profile.ekf) {
		_medium_velocity_kind = profile.medium_velocity_kind;
		_estimate_medium_velocity = profile.estimate_medium_velocity;
	}

	void EKF::_reset_covariance() {
		_covariance = _initial_variance_diag.asDiagonal();
	}

	void EKF::reset(const NavigationState& init) {
		_x.setZero();
		_x.segment<3>(0) = init.eta.segment<3>(0); // Position
		_x.segment<3>(3) = init.eta.segment<3>(3); // Attitude
		_x.segment<3>(6) = init.nu.segment<3>(0);  // Body frame velocity
		_x.segment<3>(15) = init.medium_velocity_ned;
		// Biases (9..14) initialize to zero. Medium velocity comes from the
		// supplied navigation state and uses profile-specific covariance.
		_reset_covariance();
		_last_bottom_dvl_timestamp_s = -std::numeric_limits<double>::infinity();
		_last_water_dvl_timestamp_s = -std::numeric_limits<double>::infinity();
		_last_wheel_odometry_timestamp_s = -std::numeric_limits<double>::infinity();
		_last_gps_timestamp_s = -std::numeric_limits<double>::infinity();
		_dvl_velocity_rejection_streak = 0;
		_last_rejected_dvl_velocity_body.setZero();
		_last_rejected_dvl_velocity_timestamp_s =
			-std::numeric_limits<double>::infinity();
		_initialized = true;
		_gps_velocity_rejection_streak = 0;
		_last_rejected_gps_velocity_ned.setZero();
		_last_rejected_gps_velocity_timestamp_s =
			-std::numeric_limits<double>::infinity();
	}

	NavigationState EKF::update(
		const NavigationMeasurements&		 meas,
		const std::optional<DVLMeasurement>& bottom_dvl,
		const std::optional<DVLMeasurement>& water_dvl,
		const std::optional<GPSMeasurement>& gps,
		double								 dt) {
		_last_stats = {};
		const bool			  have_accel = meas.accel_body.meta.valid;
		const Eigen::Vector3d accel_body =
			have_accel ? meas.accel_body.value : Eigen::Vector3d::Zero();
		const Eigen::Vector3d omega_body =
			meas.gyro_body.meta.valid ? meas.gyro_body.value : Eigen::Vector3d::Zero();

		if (!_initialized) {
			NavigationState s;
			s.eta.setZero();
			if (meas.depth.meta.valid)
				s.eta[2] = meas.depth.value;
			else if (gps && gps->has_altitude)
				s.eta[2] = gps->pos_d;
			reset(s);
		}

		_predict(accel_body, omega_body, have_accel, dt);

		// Heading psi (idx 5) and yaw gyro bias b_gz (idx 14) are unobservable
		// from gravity, DVL, depth, and position-only GPS. Restore them after
		// those updates, then apply explicit heading aids such as magnetometer,
		// simulator heading fallback, or GPS course.
		const double psi_pred = _x[5];
		const double bgz_pred = _x[14];

		// Perform accelerometer leveling measurement when specific force is available,
		// making roll/pitch and horizontal gyro bias observable. The gravity-only
		// model is invalid during a manoeuvre even when |norm(f)-g| is small (for
		// example, lateral acceleration changes the norm only at second order).
		// Dynamic platform profiles therefore admit this update only while both
		// estimated body speed and measured angular rate indicate a stationary
		// initialization/hold condition.
		if (have_accel) {
			const bool stationary_for_leveling =
				_x.segment<3>(6).norm() <= _level_max_body_speed_mps &&
				omega_body.norm() <= _level_max_gyro_radps;
			if (stationary_for_leveling && _update_level(
					accel_body, _valid_cov3(meas.accel_body.covariance, _R_level)))
				++_last_stats.level_accepted;
			else
				++_last_stats.level_rejected;
		}

		const bool bottom_dvl_available = bottom_dvl && bottom_dvl->beam_valid > 0
			&& meas.dvl_velocity_body.meta.valid;
		const bool water_dvl_available = water_dvl && water_dvl->beam_valid > 0
			&& meas.water_dvl_velocity_body.meta.valid;
		const bool wheel_odometry_available =
			meas.wheel_odometry_velocity_body.meta.valid;

		if (bottom_dvl_available
			&& _consume_new_sample(
				bottom_dvl->timestamp, _last_bottom_dvl_timestamp_s)) {
			bool reacquired = false;
			if (_update_bottom_dvl(
					*bottom_dvl,
					_valid_cov3(meas.dvl_velocity_body.covariance, _R_dvl),
					reacquired)) {
				++_last_stats.dvl_accepted;
				if (reacquired)
					++_last_stats.dvl_reacquired;
			} else
				++_last_stats.dvl_rejected;
		}
		if (water_dvl_available
			&& _consume_new_sample(water_dvl->timestamp, _last_water_dvl_timestamp_s)) {
			if (_update_water_dvl(
					water_dvl->velocity_body(),
					_valid_cov3(meas.water_dvl_velocity_body.covariance, _R_water_dvl),
					have_accel))
				++_last_stats.water_dvl_accepted;
			else
				++_last_stats.water_dvl_rejected;
		}
		if (wheel_odometry_available
			&& _consume_new_sample(
				meas.wheel_odometry_velocity_body.meta.timestamp_s,
				_last_wheel_odometry_timestamp_s)) {
			if (_update_dvl(
					meas.wheel_odometry_velocity_body.value,
					_valid_cov3(meas.wheel_odometry_velocity_body.covariance, _R_dvl)))
				++_last_stats.wheel_odometry_accepted;
			else
				++_last_stats.wheel_odometry_rejected;
		}
		if (!have_accel && !bottom_dvl_available && !water_dvl_available
			&& !wheel_odometry_available)
			_update_zvu(); // kinematic fallback: weakly hold velocity near zero without
						   // DVL

		if (meas.depth.meta.valid) {
			if (_update_depth(
					meas.depth.value,
					_valid_variance(meas.depth.variance, _R_depth(0, 0))))
				++_last_stats.depth_accepted;
			else
				++_last_stats.depth_rejected;
		}
		const bool gps_new = gps && meas.gps_position_ned.meta.valid
			&& _consume_new_sample(gps->timestamp, _last_gps_timestamp_s);
		if (gps_new) {
			Eigen::Matrix2d Rgps = Eigen::Matrix2d::Zero();
			Rgps(0, 0) = meas.gps_position_ned.covariance(0, 0);
			Rgps(0, 1) = meas.gps_position_ned.covariance(0, 1);
			Rgps(1, 0) = meas.gps_position_ned.covariance(1, 0);
			Rgps(1, 1) = meas.gps_position_ned.covariance(1, 1);
			if (_update_gps_xy(*gps, _valid_cov2(Rgps, _R_gps)))
				++_last_stats.gps_xy_accepted;
			else
				++_last_stats.gps_xy_rejected;

			if (gps->has_altitude) {
				if (_update_gps_altitude(
						gps->pos_d,
						_valid_variance(
							meas.gps_position_ned.covariance(2, 2), _R_gps_z(0, 0))))
					++_last_stats.gps_z_accepted;
				else
					++_last_stats.gps_z_rejected;
			}

			if (gps->has_velocity && meas.gps_velocity_ned.meta.valid) {
				// GPS ground velocity alone cannot distinguish body velocity
				// from attitude. Only expose attitude columns when a separate
				// body-frame velocity aid (DVL or wheel odometry) makes that
				// relationship observable. This is especially important for
				// fixed-wing launch, where GPS is the sole velocity aid.
				const bool body_velocity_observed = bottom_dvl_available ||
					(water_dvl_available && have_accel) || wheel_odometry_available;
				if (_update_gps_velocity(
						*gps,
						_valid_cov3(meas.gps_velocity_ned.covariance, _R_gps_velocity),
						body_velocity_observed))
					++_last_stats.gps_velocity_accepted;
				else
					++_last_stats.gps_velocity_rejected;
			}
		}

		// Restore unobservable heading state (pure gyro integration)
		_x[5] = psi_pred;
		_x[14] = bgz_pred;
		const std::optional<double> mag_heading = meas.mag_body.meta.valid
			? _heading_from_magnetometer(meas.mag_body.value)
			: std::nullopt;
		if (mag_heading) {
			if (_update_heading(
					*mag_heading, _heading_variance_from_mag(meas.mag_body)))
				++_last_stats.heading_accepted;
			else
				++_last_stats.heading_rejected;
		} else if (meas.truth_heading_debug.meta.valid) {
			if (_update_heading(
					meas.truth_heading_debug.value,
					_valid_variance(
						meas.truth_heading_debug.variance, _R_heading(0, 0))))
				++_last_stats.heading_accepted;
			else
				++_last_stats.heading_rejected;
		} else if (gps_new) {
			if (_update_gps_course_yaw(*gps, _R_heading(0, 0)))
				++_last_stats.heading_accepted;
		}
		_x[5] = _ssa(_x[5]);

		// Assemble output
		NavigationState out;
		out.eta.segment<3>(0) = _x.segment<3>(0);
		out.eta.segment<3>(3) = _x.segment<3>(3);
		out.nu.segment<3>(0) = _x.segment<3>(6);
		out.nu.segment<3>(3) =
			omega_body - _x.segment<3>(12); // De-biased gyro angular velocity
		out.depth_m = _x[2];
		out.medium_velocity_ned = _x.segment<3>(15);
		out.medium_velocity_kind = _medium_velocity_kind;
		for (int axis = 0; axis < 3; ++axis) {
			out.medium_velocity_std_ned[axis] =
				std::sqrt(std::max(_min_variance, _covariance(15 + axis, 15 + axis)));
		}
		out.medium_velocity_valid = _estimate_medium_velocity
			&& (out.medium_velocity_std_ned.array() <= _medium_velocity_valid_std)
				   .all();
		out.dvl_valid = bottom_dvl.has_value();
		return out;
	}

	// Continuous-time dynamics x_dot = f(x, accel, omega)
	Eigen::Matrix<double, EKF_N, 1> EKF::_dynamics(
		const Eigen::Matrix<double, EKF_N, 1>& x,
		const Eigen::Vector3d&				   accel,
		const Eigen::Vector3d&				   omega,
		bool								   have_accel) {
		const double		  phi = x[3], theta = x[4], psi = x[5];
		const Eigen::Vector3d v_b = x.segment<3>(6);
		const Eigen::Vector3d b_a = x.segment<3>(9);
		const Eigen::Vector3d b_g = x.segment<3>(12);

		const Eigen::Vector3d omega_c = omega - b_g; // De-biased angular velocity
		const Eigen::Matrix3d R = _rot_nb(phi, theta, psi);
		const Eigen::Matrix3d J2 = _J2(phi, theta);

		Eigen::Matrix<double, EKF_N, 1> xdot;
		xdot.setZero();

		xdot.segment<3>(0) = R * v_b;	   // Position
		xdot.segment<3>(3) = J2 * omega_c; // Attitude

		if (have_accel) {
			const Eigen::Vector3d f = accel - b_a; // De-biased specific force
			const Eigen::Vector3d g_n(0.0, 0.0, EKF_G);
			// v_dot_b = f + R_nb^T g_n - omega x v_b (when static and level, f = [0, 0,
			// -g] -> v_dot = 0)
			xdot.segment<3>(6) = f + R.transpose() * g_n - omega_c.cross(v_b);
		}
		// have_accel=false -> velocity is not integrated via specific force (driven by
		// DVL/ZVU updates), degrading to kinematics

		// Bias random walk: mean derivative is 0
		return xdot;
	}

	// Prediction step: state Euler integration plus a structured Jacobian and
	// covariance propagation. Only the attitude columns are nonlinear enough
	// to justify finite differences. The remaining blocks follow directly from
	// the mechanization, and rows 9..17 are identically zero. Exploiting that
	// structure avoids the 18 dynamics evaluations and two dense 18x18 GEMMs
	// that prevented the FMUv6C target from sustaining the 100 Hz HIL contract.
	void EKF::_predict(
		const Eigen::Vector3d& accel,
		const Eigen::Vector3d& omega,
		bool				   have_accel,
		double				   dt) {
		const Eigen::Matrix<double, EKF_N, 1> f0 =
			_dynamics(_x, accel, omega, have_accel);

		auto& A = _predict_jacobian;
		A.setZero();

		// R and J2 supply exact velocity and gyro-bias blocks.
		const double phi = _x[3];
		const double theta = _x[4];
		const double psi = _x[5];
		const Eigen::Matrix3d R = _rot_nb(phi, theta, psi);
		const Eigen::Matrix3d J2 = _J2(phi, theta);
		A.block<3, 3>(0, 6) = R;
		A.block<3, 3>(3, 12) = -J2;

		// Attitude affects R, J2, and body-frame gravity. Three forward
		// differences retain the established nonlinear model without spending
		// fifteen evaluations on states whose derivatives are known exactly.
		const double eps = 1e-6;
		for (int j = 3; j < 6; ++j) {
			Eigen::Matrix<double, EKF_N, 1> xp = _x;
			xp[j] += eps;
			const Eigen::Matrix<double, EKF_N, 1> fp =
				_dynamics(xp, accel, omega, have_accel);
			A.col(j) = (fp - f0) / eps;
		}

		if (have_accel) {
			const Eigen::Vector3d omega_c =
				omega - _x.segment<3>(12);
			const Eigen::Vector3d velocity = _x.segment<3>(6);
			Eigen::Matrix3d omega_cross;
			omega_cross << 0.0, -omega_c.z(), omega_c.y(),
				omega_c.z(), 0.0, -omega_c.x(),
				-omega_c.y(), omega_c.x(), 0.0;
			Eigen::Matrix3d velocity_cross;
			velocity_cross << 0.0, -velocity.z(), velocity.y(),
				velocity.z(), 0.0, -velocity.x(),
				-velocity.y(), velocity.x(), 0.0;
			A.block<3, 3>(6, 6) = -omega_cross;
			A.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity();
			A.block<3, 3>(6, 12) = -velocity_cross;
		}

		// State Euler integration
		_x += dt * f0;

		// Measurement updates are theoretically symmetric but can accumulate
		// round-off asymmetry. Restore symmetry before using AP transpose as
		// P*A^T in the structured propagation below.
		for (int i = 0; i < EKF_N; ++i) {
			for (int j = i + 1; j < EKF_N; ++j) {
				const double symmetric =
					0.5 * (_covariance(i, j) + _covariance(j, i));
				_covariance(i, j) = symmetric;
				_covariance(j, i) = symmetric;
			}
		}

		// F*P*F^T = P + dt*(A*P + P*A^T) + dt^2*A*P*A^T.
		// A has nonzero entries only in rows 0..8 and columns 0..14.
		auto& AP = _predict_ap;
		AP.setZero();
		for (int i = 0; i < 9; ++i) {
			for (int k = 0; k < 15; ++k) {
				const double aik = A(i, k);
				if (aik == 0.0)
					continue;
				for (int j = 0; j < EKF_N; ++j)
					AP(i, j) += aik * _covariance(k, j);
			}
		}

		const double dt2 = dt * dt;
		for (int i = 0; i < EKF_N; ++i) {
			for (int j = i; j < EKF_N; ++j) {
				double propagated = _covariance(i, j) +
					dt * (AP(i, j) + AP(j, i)) + _Q(i, j) * dt;
				if (i < 9 && j < 9) {
					double second_order = 0.0;
					for (int k = 0; k < 15; ++k)
						second_order += AP(i, k) * A(j, k);
					propagated += dt2 * second_order;
				}
				_covariance(i, j) = propagated;
				_covariance(j, i) = propagated;
			}
		}
	}

	// Measurement Updates
	bool EKF::_update_bottom_dvl(
		const DVLMeasurement& sample,
		const Eigen::Matrix3d& R,
		bool& reacquired) {
		reacquired = false;
		const Eigen::Vector3d z = sample.velocity_body();
		if (_update_dvl(z, R)) {
			_dvl_velocity_rejection_streak = 0;
			_last_rejected_dvl_velocity_timestamp_s =
				-std::numeric_limits<double>::infinity();
			return true;
		}

		const double sample_interval =
			sample.timestamp - _last_rejected_dvl_velocity_timestamp_s;
		const bool consecutive =
			_dvl_velocity_rejection_streak > 0 &&
			std::isfinite(sample_interval) && sample_interval > 0.0 &&
			sample_interval <= _dvl_velocity_reacquire_max_interval_s;
		const bool consistent = consecutive &&
			(z - _last_rejected_dvl_velocity_body).norm() <=
				_dvl_velocity_reacquire_consistency_mps;
		_dvl_velocity_rejection_streak = consistent
			? _dvl_velocity_rejection_streak + 1
			: 1;
		_last_rejected_dvl_velocity_body = z;
		_last_rejected_dvl_velocity_timestamp_s = sample.timestamp;

		if (_dvl_velocity_reacquire_samples <= 0 ||
			_dvl_velocity_rejection_streak < _dvl_velocity_reacquire_samples ||
			z.norm() > _dvl_velocity_reacquire_max_speed_mps)
			return false;

		// Bottom-track DVL directly observes body velocity. Reinitialize only
		// that observed block; position, attitude and IMU biases stay continuous.
		_x.segment<3>(6) = z;
		for (int index = 6; index <= 8; ++index) {
			_covariance.row(index).setZero();
			_covariance.col(index).setZero();
		}
		_covariance.block<3, 3>(6, 6) = R;
		for (int axis = 0; axis < 3; ++axis)
			_covariance(6 + axis, 6 + axis) = std::max(
				_min_variance, _covariance(6 + axis, 6 + axis));
		_dvl_velocity_rejection_streak = 0;
		_last_rejected_dvl_velocity_timestamp_s =
			-std::numeric_limits<double>::infinity();
		reacquired = true;
		return true;
	}

	bool EKF::_update_dvl(const Eigen::Vector3d& z_dvl, const Eigen::Matrix3d& R) {
		Eigen::Matrix<double, 3, EKF_N> H;
		H.setZero();
		H(0, 6) = 1.0;
		H(1, 7) = 1.0;
		H(2, 8) = 1.0;

		const Eigen::Vector3d y = z_dvl - _x.segment<3>(6);
		return _apply_linear_update<3>(H, y, R, _gate_dvl_nis);
	}

	bool EKF::_update_water_dvl(
		const Eigen::Vector3d& z_water_dvl,
		const Eigen::Matrix3d& R,
		bool				   allow_attitude_correction) {
		// Water-track DVL reports the vehicle velocity relative to the water
		// mass. The EKF velocity state remains body-frame ground velocity;
		// water current is held in world NED, hence h = v_g,b - R_nb^T c_n.
		// Return an owned vector: an auto-deduced Eigen expression would keep
		// a dangling reference to the local rotation matrix after this call.
		const auto water_prediction = [](const Eigen::Matrix<double, EKF_N, 1>& x) -> Eigen::Vector3d {
			const Eigen::Matrix3d R_nb = _rot_nb(x[3], x[4], x[5]);
			return x.segment<3>(6) - R_nb.transpose() * x.segment<3>(15);
		};

		const Eigen::Vector3d			h = water_prediction(_x);
		Eigen::Matrix<double, 3, EKF_N> H;
		H.setZero();
		H.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();

		// Without accelerometer mechanization/leveling, body attitude is not
		// independently observable from two unknown velocity vectors. Letting
		// this relative-velocity update rotate the vehicle then creates a
		// positive-feedback ambiguity between attitude, ground velocity and
		// medium velocity. Freeze the direct attitude columns in that fallback
		// mode; gyro propagation still carries the nominal attitude.
		if (allow_attitude_correction) {
			const double eps = 1e-6;
			for (int j = 3; j <= 5; ++j) {
				Eigen::Matrix<double, EKF_N, 1> xp = _x;
				xp[j] += eps;
				H.col(j) = (water_prediction(xp) - h) / eps;
			}
		}
		const Eigen::Matrix3d R_nb = _rot_nb(_x[3], _x[4], _x[5]);
		H.block<3, 3>(0, 15) = -R_nb.transpose();

		const Eigen::Vector3d y = z_water_dvl - h;
		return _apply_linear_update<3>(H, y, R, _gate_water_dvl_nis);
	}

	void EKF::_update_zvu() {
		// Pseudo-observation: body velocity = [0,0,0] with high noise.
		// Prevents velocity states from drifting when DVL is unavailable.
		Eigen::Matrix<double, 3, EKF_N> H;
		H.setZero();
		H(0, 6) = 1.0;
		H(1, 7) = 1.0;
		H(2, 8) = 1.0;

		const Eigen::Vector3d y = -_x.segment<3>(6); // innovation: 0 - estimated vel
		(void)_apply_linear_update<3>(
			H, y, _R_zvu, std::numeric_limits<double>::infinity());
	}

	bool EKF::_update_depth(double depth_m, double variance) {
		Eigen::Matrix<double, 1, EKF_N> H;
		H.setZero();
		H(0, 2) = 1.0;

		Eigen::Matrix<double, 1, 1> y;
		y(0, 0) = depth_m - _x[2];
		Eigen::Matrix<double, 1, 1> R;
		R << _valid_variance(variance, _R_depth(0, 0));
		return _apply_linear_update<1>(H, y, R, _gate_scalar_nis);
	}

	bool EKF::_update_gps_xy(const GPSMeasurement& gps, const Eigen::Matrix2d& R) {
		Eigen::Matrix<double, 2, EKF_N> H;
		H.setZero();
		H(0, 0) = 1.0;
		H(1, 1) = 1.0;

		Eigen::Vector2d z;
		z << gps.pos_n, gps.pos_e;
		const Eigen::Vector2d y = z - _x.segment<2>(0);
		return _apply_linear_update<2>(H, y, R, _gate_gps_xy_nis);
	}

	bool EKF::_update_gps_velocity(
		const GPSMeasurement&  gps,
		const Eigen::Matrix3d& R,
		bool				   allow_attitude_correction) {
		// GPS is ground-referenced in NED; it observes R_nb * v_ground_body.
		// Materialize before the temporary rotation matrix is destroyed.
		const auto velocity_prediction = [](const Eigen::Matrix<double, EKF_N, 1>& x) -> Eigen::Vector3d {
			return _rot_nb(x[3], x[4], x[5]) * x.segment<3>(6);
		};

		const Eigen::Vector3d			h = velocity_prediction(_x);
		Eigen::Matrix<double, 3, EKF_N> H;
		H.setZero();
		H.block<3, 3>(0, 6) = _rot_nb(_x[3], _x[4], _x[5]);
		if (allow_attitude_correction) {
			const double eps = 1e-6;
			for (int j = 3; j <= 5; ++j) {
				Eigen::Matrix<double, EKF_N, 1> xp = _x;
				xp[j] += eps;
				H.col(j) = (velocity_prediction(xp) - h) / eps;
			}
		}

		const Eigen::Vector3d z(gps.vel_n, gps.vel_e, gps.vel_d);
		const Eigen::Vector3d y = z - h;
		double nis = std::numeric_limits<double>::infinity();
		if (!_apply_linear_update<3>(
				H, y, R, _gate_gps_velocity_nis, &nis)) {
			if (!std::isfinite(nis))
				return false;
			const double sample_interval =
				gps.timestamp - _last_rejected_gps_velocity_timestamp_s;
			const bool consecutive =
				_gps_velocity_rejection_streak > 0 &&
				std::isfinite(sample_interval) && sample_interval > 0.0 &&
				sample_interval <= _gps_navigation_reacquire_max_interval_s;
			const bool consistent = consecutive &&
				(z - _last_rejected_gps_velocity_ned).norm() <=
					_gps_navigation_reacquire_consistency_mps;
			_gps_velocity_rejection_streak = consistent
				? _gps_velocity_rejection_streak + 1
				: 1;
			_last_rejected_gps_velocity_ned = z;
			_last_rejected_gps_velocity_timestamp_s = gps.timestamp;

			const bool plausible_speed =
				z.norm() <= _gps_navigation_reacquire_max_speed_mps;
			if (_gps_navigation_reacquire_samples <= 0 ||
				_gps_velocity_rejection_streak <
					_gps_navigation_reacquire_samples ||
				!plausible_speed)
				return false;

			// Several time-adjacent, mutually consistent GPS observations
			// identify a navigation-mode transition rather than one outlier.
			// Reset only GPS-observed states; attitude and IMU biases remain
			// continuous.
			const Eigen::Matrix3d R_nb = _rot_nb(_x[3], _x[4], _x[5]);
			_x[0] = gps.pos_n;
			_x[1] = gps.pos_e;
			// This vetted reacquisition actually accepts GPS position, even
			// if the ordinary position innovation was rejected earlier in
			// this tick. Report it immediately so runtime position-aid health
			// does not wait for another GPS frame and falsely latch a fault.
			++_last_stats.gps_xy_accepted;
			_x.segment<3>(6) = R_nb.transpose() * z;
			for (const int index : {0, 1, 6, 7, 8}) {
				_covariance.row(index).setZero();
				_covariance.col(index).setZero();
			}
			_covariance(0, 0) = std::max(_min_variance, _R_gps(0, 0));
			_covariance(1, 1) = std::max(_min_variance, _R_gps(1, 1));
			_covariance.block<3, 3>(6, 6) = R_nb.transpose() * R * R_nb;
			for (int axis = 0; axis < 3; ++axis)
				_covariance(6 + axis, 6 + axis) = std::max(
					_min_variance, _covariance(6 + axis, 6 + axis));
			_gps_velocity_rejection_streak = 0;
			_last_rejected_gps_velocity_timestamp_s =
				-std::numeric_limits<double>::infinity();
			return true;
		}

		_gps_velocity_rejection_streak = 0;
		_last_rejected_gps_velocity_timestamp_s =
			-std::numeric_limits<double>::infinity();
		return true;
	}

	bool EKF::_update_gps_course_yaw(const GPSMeasurement& gps, double variance) {
		if (!gps.has_velocity)
			return false;

		const double ground_speed = std::hypot(gps.vel_n, gps.vel_e);
		const double body_speed = std::hypot(_x[6], _x[7]);
		if (ground_speed < 0.5 || body_speed < 0.5)
			return false;

		const double world_course = std::atan2(gps.vel_e, gps.vel_n);
		const double body_course = std::atan2(_x[7], _x[6]);
		const double yaw_meas = _ssa(world_course - body_course);
		return _update_heading(yaw_meas, variance);
	}

	bool EKF::_update_gps_altitude(double pos_d, double variance) {
		Eigen::Matrix<double, 1, EKF_N> H;
		H.setZero();
		H(0, 2) = 1.0;

		Eigen::Matrix<double, 1, 1> y;
		y(0, 0) = pos_d - _x[2];
		Eigen::Matrix<double, 1, 1> R;
		R << _valid_variance(variance, _R_gps_z(0, 0));
		return _apply_linear_update<1>(H, y, R, _gate_scalar_nis);
	}

	std::optional<double>
	EKF::_heading_from_magnetometer(const Eigen::Vector3d& mag_body) const {
		if (!mag_body.allFinite())
			return std::nullopt;

		const double norm = mag_body.norm();
		if (norm < 1.0 || norm > 1000.0)
			return std::nullopt;

		const double phi = _x[3];
		const double theta = _x[4];
		const double cphi = std::cos(phi);
		const double sphi = std::sin(phi);
		const double ctheta = std::cos(theta);
		const double stheta = std::sin(theta);

		// Remove roll/pitch from the body-frame magnetic vector. With the
		// default generated field declination at zero, yaw = -atan2(E, N).
		const double leveled_x = ctheta * mag_body.x()
			+ stheta * (sphi * mag_body.y() + cphi * mag_body.z());
		const double leveled_y = cphi * mag_body.y() - sphi * mag_body.z();
		const double horizontal = std::hypot(leveled_x, leveled_y);
		if (horizontal < 1.0)
			return std::nullopt;

		constexpr double declination_rad = 0.0;
		return _ssa(declination_rad - std::atan2(leveled_y, leveled_x));
	}

	bool EKF::_update_heading(double heading_yaw, double variance) {
		Eigen::Matrix<double, 1, EKF_N> H;
		H.setZero();
		H(0, 5) = 1.0;

		const double				y = _ssa(heading_yaw - _x[5]);
		Eigen::Matrix<double, 1, 1> R;
		R << _valid_variance(variance, _R_heading(0, 0));
		Eigen::Matrix<double, 1, 1> innovation;
		innovation << y;
		if (!_apply_linear_update<1>(
				H, innovation, R, _gate_heading_nis))
			return false;
		_x[5] = _ssa(_x[5]);
		return true;
	}

	bool EKF::_update_level(const Eigen::Vector3d& accel, const Eigen::Matrix3d& R) {
		// Only enabled when nearly static or moving at constant speed (no significant
		// linear acceleration): |norm(f_meas) - g| <= threshold
		if (std::abs(accel.norm() - EKF_G) > _level_gate)
			return false;

		const double		  phi = _x[3], theta = _x[4], psi = _x[5];
		// Predicted specific force (static leveling model) h = -R_nb^T g_n =
		// -gravity_body
		const Eigen::Vector3d h = -_gravity_body(phi, theta, psi);

		// Numerical Jacobian H = dh/dx (only depends on attitude columns 3/4/5)
		Eigen::Matrix<double, 3, EKF_N> H;
		H.setZero();
		const double eps = 1e-6;
		for (int j = 3; j <= 5; ++j) {
			double ang[3] = { phi, theta, psi };
			ang[j - 3] += eps;
			const Eigen::Vector3d hp = -_gravity_body(ang[0], ang[1], ang[2]);
			H.col(j) = (hp - h) / eps;
		}

		const Eigen::Vector3d y =
			(accel - _x.segment<3>(9)) - h; // Measurement = de-biased specific force
		return _apply_linear_update<3>(H, y, R, _gate_level_nis);
	}

	bool EKF::_consume_new_sample(double timestamp_s, double& last_timestamp_s) {
		// Every sensor frame must affect the filter at most once.  Advancing
		// the watermark before innovation gating also prevents a rejected
		// outlier from being retried on every high-rate IMU tick.
		if (!std::isfinite(timestamp_s) || timestamp_s <= last_timestamp_s)
			return false;
		last_timestamp_s = timestamp_s;
		return true;
	}

	Eigen::Matrix3d
	EKF::_valid_cov3(const Eigen::Matrix3d& R, const Eigen::Matrix3d& fallback) const {
		Eigen::Matrix3d out = R;
		if (!out.allFinite())
			return fallback;
		out = 0.5 * (out + out.transpose());
		for (int i = 0; i < 3; ++i) {
			if (out(i, i) < _min_variance)
				out(i, i) = fallback(i, i);
			if (out(i, i) < _min_variance)
				out(i, i) = _min_variance;
		}
		return out;
	}

	Eigen::Matrix2d
	EKF::_valid_cov2(const Eigen::Matrix2d& R, const Eigen::Matrix2d& fallback) const {
		Eigen::Matrix2d out = R;
		if (!out.allFinite())
			return fallback;
		out = 0.5 * (out + out.transpose());
		for (int i = 0; i < 2; ++i) {
			if (out(i, i) < _min_variance)
				out(i, i) = fallback(i, i);
			if (out(i, i) < _min_variance)
				out(i, i) = _min_variance;
		}
		return out;
	}

	double EKF::_valid_variance(double variance, double fallback) const {
		if (!std::isfinite(variance) || variance < _min_variance)
			variance = fallback;
		if (!std::isfinite(variance) || variance < _min_variance)
			variance = _min_variance;
		return variance;
	}

	double EKF::_heading_variance_from_mag(const Vector3Measurement& mag) const {
		const double mean_var = _valid_cov3(mag.covariance, _R_level).trace() / 3.0;
		const double field_norm = std::max(1.0, mag.value.norm());
		const double approx = mean_var / (field_norm * field_norm);
		return std::clamp(_valid_variance(approx, _R_heading(0, 0)), 1.0e-5, 0.25);
	}

	// Utility functions
	Eigen::Matrix3d EKF::_rot_nb(double phi, double theta, double psi) {
		double			cphi = std::cos(phi), sphi = std::sin(phi);
		double			cth = std::cos(theta), sth = std::sin(theta);
		double			cpsi = std::cos(psi), spsi = std::sin(psi);
		Eigen::Matrix3d R;
		R << cpsi * cth, cpsi * sth * sphi - spsi * cphi,
			cpsi * sth * cphi + spsi * sphi, spsi * cth,
			spsi * sth * sphi + cpsi * cphi, spsi * sth * cphi - cpsi * sphi, -sth,
			cth * sphi, cth * cphi;
		return R;
	}

	Eigen::Matrix3d EKF::_J2(double phi, double theta) {
		double cphi = std::cos(phi), sphi = std::sin(phi);
		double cth = std::cos(theta), sth = std::sin(theta);
		if (std::abs(cth) < 1e-6)
			cth = (cth < 0 ? -1e-6 : 1e-6);
		Eigen::Matrix3d J;
		J << 1.0, sphi * sth / cth, cphi * sth / cth, 0.0, cphi, -sphi, 0.0, sphi / cth,
			cphi / cth;
		return J;
	}

	Eigen::Vector3d EKF::_gravity_body(double phi, double theta, double psi) {
		const Eigen::Vector3d g_n(0.0, 0.0, EKF_G);
		return _rot_nb(phi, theta, psi).transpose() * g_n;
	}

	double EKF::_ssa(double a) {
		return std::atan2(std::sin(a), std::cos(a));
	}

} // namespace hydrox
