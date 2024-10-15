//
// Created by peer on 23-2-24.
//

#ifndef FT_KALMAN_KALMANFILTER_HPP
#define FT_KALMAN_KALMANFILTER_HPP

#include <iostream>
#include "Matrix.hpp"
#include "Data.hpp"

//#define n 9 // n is the amount of state variables
//// n: 3 positions, 3 directions, 3 accelerations
//// n: [x, y, z, dir, v, acc_x, acc_y, acc_z] (no direction because they won't ever change)
//#define p 3 // p is the amount of inputs
//// p: []
//#define m 3 // m is the amount of outputs
//// m: []

#define NOISE 1
#define GPS_NOISE 0.1
#define GYROSCOPE_NOISE 0.01
#define ACCELEROMETER_NOISE 0.001

template<size_t Nx, size_t Nz, size_t Nu>
class KalmanFilter {
public:
	using StateVector = Matrix<double, Nx, 1>;
	using MeasurementVector = Matrix<double, Nz, 1>;
	using StateTransitionMatrix = Matrix<double, Nx, Nx>;
	using InputVector = Matrix<double, Nu, 1>;
	using ControlMatrix = Matrix<double, Nx, Nu>;
	using EstimateCovarianceMatrix = Matrix<double, Nx, Nx>;
	using ProcessNoiseCovariance = Matrix<double, Nx, Nx>;
	using MeasurementCovariance = Matrix<double, Nz, Nz>;
	using ProcessNoiseVector = Matrix<double, Nx, 1>;
	using MeasurementNoiseVector = Matrix<double, Nz, 1>;
	using ObservationMatrix = Matrix<double, Nz, Nx>;
	using KalmanGain = Matrix<double, Nx, Nz>;

private:
	Vector<double, Nx> previousState{};
	Vector<double, Nx> currenState{};
	Vector<double, Nx> predictedState{};
	unsigned int k;
	// Matrices
	Matrix<double, Nx, Nx> state_covariance_matrix;
	Matrix<double, Nx, Nx> state_transition_matrix;
	Matrix<double, Nz, Nx> state_to_measurement_matrix;
	Matrix<double, Nz, Nz> measurement_covariance_matrix;
	Matrix<double, Nx, Nz> kalman_gain_matrix;

	Matrix<double, Nx, Nx> P_mat = Matrix<double, Nx, Nx>({
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   ,0    , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , GYROSCOPE_NOISE	, 0   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , GYROSCOPE_NOISE   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , GYROSCOPE_NOISE   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE}),
	});

	// TODO: fill this noise matrix
	Matrix<double, Nx, Nx> Q_mat = Matrix<double, Nx, Nx>({
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   ,0    , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , GYROSCOPE_NOISE	, 0   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , GYROSCOPE_NOISE   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , GYROSCOPE_NOISE   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE}),
	});

	Matrix<double, Nx, Nx> H_mat = Matrix<double, Nx, Nx>({
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 1   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 1   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 1   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0	, 0   , 0   , 1   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 1   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 1}),
	});

	Matrix<double, Nx, Nx> R_mat = Matrix<double, Nx, Nx>({
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   ,0    , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , GYROSCOPE_NOISE	, 0   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , GYROSCOPE_NOISE   , 0   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , GYROSCOPE_NOISE   , 0   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE   , 0   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE   , 0}),
		std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE}),
	});

	Matrix<double, Nx, Nx> identity = Matrix<double, Nx, Nx>({
		std::array<double, Nx>({ 1  , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 }),
		std::array<double, Nx>({ 0  , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 }),
		std::array<double, Nx>({ 0  , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 }),
		std::array<double, Nx>({ 0  , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 }),
		std::array<double, Nx>({ 0  , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 }),
		std::array<double, Nx>({ 0  , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 }),
		std::array<double, Nx>({ 0  , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 }),
		std::array<double, Nx>({ 0  , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 }),
		std::array<double, Nx>({ 0  , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 }),
	});

public:
	explicit KalmanFilter() {
		this->currenState = Matrix<double, Nx, 1>();
	}

	// Vector3d predict(size_t time_step, const InputVector &inputs);

	// ///

	// EstimateCovarianceMatrix extrapolate_covariance(const StateTransitionMatrix F_mat, const EstimateCovarianceMatrix P_mat);

	// MeasurementVector calculate_measurement_vector(const InputVector &inputs);

	// KalmanGain calculate_kalman_gain();

	// StateVector update_state_matrix(KalmanGain &kalman, StateVector x_prev, MeasurementVector z_n);

	// EstimateCovarianceMatrix update_covariance_matrix(KalmanGain &kalman);

	///

	Vector3d predict(size_t time_step, const InputVector& inputs) {
		std::cout << "IN" << this->currenState << std::endl;

		auto time = (double)time_step / 1000;

		// acceleration squared.
		auto acsq = 0.5 * time * time;

		auto F_mat = Matrix<double, 9, 9>({
			std::array<double, 9>({ 1  , 0  , 0   , time, 0   , 0   , acsq, 0   , 0}),
			std::array<double, 9>({ 0  , 1  , 0   , 0   , time, 0   , 0   , acsq, 0}),
			std::array<double, 9>({ 0  , 0  , 1   , 0   , 0   , time, 0   , 0   , acsq}),
			std::array<double, 9>({ 0  , 0  , 0   , 1	, 0   , 0   , time, 0   , 0}),
			std::array<double, 9>({ 0  , 0  , 0   , 0   , 1   , 0   , 0   , time, 0}),
			std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 1   , 0   , 0   , time}),
			std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 1   , 0   , 0}),
			std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 1   , 0}),
			std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 1}),
		});

		this->currenState = F_mat * this->currenState;

		std::cout << "STATE\n" << this->currenState << std::endl;

		this->P_mat = this->extrapolate_covariance(F_mat, this->P_mat);

		std::cout << "P_MAT\n" << this->P_mat << std::endl;

		auto measurement = this->calculate_measurement_vector(inputs);

		std::cout << "MEASUREMENT\n" << measurement << std::endl;

		auto kalman = this->calculate_kalman_gain();

		std::cout << "KALMAN GAIN\n" << kalman << std::endl;

		this->predictedState = this->update_state_matrix(kalman, this->currenState, measurement);

		std::cout << "UPDATED\n" << this->predictedState << std::endl;

		this->currenState = this->predictedState;

		auto updated_covariance = this->update_covariance_matrix(kalman);

		std::cout << "COV\n" << updated_covariance << std::endl;

		this->P_mat = updated_covariance;

		Vector3d predicted_pos;

		predicted_pos[0][0] = this->predictedState[0][0];
		predicted_pos[1][0] = this->predictedState[1][0];
		predicted_pos[2][0] = this->predictedState[2][0];

	//	auto predicted_mu = A * mu_t + B * u_t;
	//	auto predicted_sigma = A * sigma_t * A.transpose() + Q;
		return predicted_pos;

	//	x = x0 + vx0 * Δt + 1/2 * ax * Δt^2
	//	y = y0 + vy0 * Δt + 1/2 * ay * Δt^2
	//	z = z0 + vz0 * Δt + 1/2 * az * Δt^2
	}

	[[nodiscard]] const Vector<double, Nx>& get_state() const {
		return (this->currenState);
	}

	Matrix<double, Nx, Nx> extrapolate_covariance(const StateTransitionMatrix F_mat, const EstimateCovarianceMatrix P_mat) {
		// Pn+1,n=FPn,nFT
		auto P_n1_mat = F_mat * P_mat * F_mat.transpose() + this->Q_mat;

		return P_n1_mat;
	}

	Matrix<double, Nx, Nx> get_state_transition_matrix(double time_step) {
		auto mat = Matrix<double, Nx, Nx>::template identity<Nx>();
		for (size_t i = 0; i < 6; i++) {
			mat[i][i + 3] = time_step;
		}
		for (size_t i = 0; i < 3; i++) {
			mat[i][i + 6] = 0.5 * time_step * time_step;
		}
		return (mat);
	}

	MeasurementVector calculate_measurement_vector(const InputVector &inputs) {
		// z_n = Hx_n + v_n

		auto z_n = this->H_mat * inputs;

		return z_n;
	}

	KalmanGain calculate_kalman_gain() {
		// Kn = Pn,n-1HT(HPn,n-1HT + Rn)^-1
		auto k_n = this->P_mat * this->H_mat.transpose() * (this->H_mat * this->P_mat * this->H_mat.transpose() + this->R_mat).pow(-1);

		// auto k_n = this->P_mat * this->H_mat.transpose();

		// std::cout << "S1\n" << k_n << std::endl;

		// auto im = (this->H_mat * this->P_mat * this->H_mat.transpose() + this->R_mat);

		// std::cout << "SIM\n" << im << std::endl;

		// k_n = k_n * im.pow(-1);

		// std::cout << "S2\n" << k_n << std::endl;

		// exit(1);


		return k_n;
	}

	StateVector update_state_matrix(KalmanGain &kalman, StateVector x_prev, MeasurementVector z_n) {
		// x_n,n = x_n,n-1 + Kn(Zn - Hx_n,n-1)

		auto x_n_n = x_prev + kalman * (z_n - this->H_mat * x_prev);

		return x_n_n;
	}

	EstimateCovarianceMatrix update_covariance_matrix(KalmanGain &kalman) {
		// P_n,n = (I - KnH)P_n,n-1(I - KnH)T + KnRnKnT

		auto p_n_n =
				(this->identity - kalman * this->H_mat) *
				this->P_mat *
				(this->identity - kalman * this->H_mat).transpose() +
				kalman * this->R_mat * kalman.transpose();

		return p_n_n;
	}

	void set_state(std::array<double, Nx> &state) {
		this->currenState = Matrix<double, Nx, 1>(state);
	}

	Matrix<double, Nx, 1> get_initial_process_noise() {
		const Vector3d gps_noise(std::array<double, 3>({0.1, 0.1, 0.1}));
		const Vector3d acceleration_noise(std::array<double, 3>({0.001, 0.001, 0.001}));
		const Vector3d gyroscope_noise(std::array<double, 3>({0.01, 0.01, 0.01}));

		auto x = gps_noise.vstack(acceleration_noise).vstack(gyroscope_noise);
		return (x);
	}

	double get_current_speed() {
		auto speed = std::sqrt(std::pow(this->currenState[0][3], 2) + std::pow(this->currenState[0][4], 2) + std::pow(this->currenState[0][5], 2));

		return speed;
	}
};




#endif // FT_KALMAN_KALMANFILTER_HPP
