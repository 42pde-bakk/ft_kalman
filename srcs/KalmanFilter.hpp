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

enum InputType {
	ACCELERATION,
	POSITION
};

template<size_t Nx, size_t Nz, size_t Nu>
class KalmanFilter {
public:
	using StateVector = Matrix<double, Nx, 1>;
	using MeasurementVector = Matrix<double, Nz, 1>;
	using StateTransitionMatrix = Matrix<double, Nx, Nx>;
	using InputVector = Matrix<double, Nz, 1>;
	using ControlMatrix = Matrix<double, Nx, Nu>;
	using EstimateCovarianceMatrix = Matrix<double, Nx, Nx>;
	using ProcessNoiseCovariance = Matrix<double, Nx, Nx>;
	using MeasurementCovariance = Matrix<double, Nz, Nz>;
	using ProcessNoiseVector = Matrix<double, Nx, 1>;
	using MeasurementNoiseVector = Matrix<double, Nz, 1>;
	using ObservationMatrix = Matrix<double, Nz, Nx>;
	using KalmanGain = Matrix<double, Nx, Nz>;

	Vector<double, Nx> state{};

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
		{ 0  , 0  , 0   , 0   , 0   , 0   ,0    , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , GYROSCOPE_NOISE	, 0   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0   , GYROSCOPE_NOISE   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , GYROSCOPE_NOISE   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE},
	});

	// TODO: fill this noise matrix
	Matrix<double, Nx, Nx> Q_mat = Matrix<double, Nx, Nx>({
		{ 0  , 0  , 0   , 0   , 0   , 0   ,0    , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , GYROSCOPE_NOISE	, 0   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0   , GYROSCOPE_NOISE   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , GYROSCOPE_NOISE   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE},
	});

	Matrix<double, Nx, Nx> H_mat = Matrix<double, Nx, Nx>({
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 1   , 0   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 1   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 1   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0	, 0   , 0   , 1   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 1   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 1},
	});

	Matrix<double, Nx, Nx> R_mat = Matrix<double, Nx, Nx>({
		{ 0  , 0  , 0   , 0   , 0   , 0   ,0    , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , GYROSCOPE_NOISE	, 0   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0   , GYROSCOPE_NOISE   , 0   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , GYROSCOPE_NOISE   , 0   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE   , 0   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE   , 0},
		{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE},
	});

	Matrix<double, Nx, Nx> identity = Matrix<double, Nx, Nx>({
		{ 1  , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 },
		{ 0  , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 },
		{ 0  , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 },
		{ 0  , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 },
		{ 0  , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 },
		{ 0  , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 },
		{ 0  , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 },
		{ 0  , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 },
		{ 0  , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 },
	});
	EstimateCovarianceMatrix P_mat = EstimateCovarianceMatrix::template identity<Nx>();

	ObservationMatrix H_mat_acceleration = ObservationMatrix({
		std::array<double, Nx>({ 0, 0, 1, 0, 0, 0, 0, 0, 0 }),
		std::array<double, Nx>({ 0, 0, 0, 0, 0, 1, 0, 0, 0 }),
		std::array<double, Nx>({ 0, 0, 0, 0, 0, 0, 0, 0, 1 }),
	});

	ObservationMatrix H_mat_position = ObservationMatrix({
		std::array<double, Nx>({ 1, 0, 0, 0, 0, 0, 0, 0, 0 }),
		std::array<double, Nx>({ 0, 0, 0, 1, 0, 0, 0, 0, 0 }),
		std::array<double, Nx>({ 0, 0, 0, 0, 0, 0, 1, 0, 0 }),
	});

	MeasurementCovariance R_mat_acceleration = MeasurementCovariance::template diag<Nz>(0.002);
	MeasurementCovariance R_mat_position = MeasurementCovariance::template diag<Nz>(0.02);

	Matrix<double, Nx, Nx> identity = Matrix<double, Nx, Nx>::template identity<Nx>();

public:
	explicit KalmanFilter() {
		this->currenState = Matrix<double, Nx, 1>();
	}

	StateTransitionMatrix make_f_mat(double delta) {
		auto acsq = 0.5 * delta * delta;

		StateTransitionMatrix F = StateTransitionMatrix::template identity<Nx>();

		for (size_t i = 0; i < 3; i++)
		{
			F[i * 3][i * 3 + 1] = delta;
			F[i * 3][i * 3 + 2] = acsq;
			F[i * 3 + 1][i * 3 + 2] = delta;
		}
		
		return F;
	}

	Vector3d predict(size_t time_step, const InputVector& inputs, InputType type) {
		Vector3d predicted_pos;

		std::cout << "START" << std::endl;

		auto F = this->make_f_mat((double)time_step * 1.0e-3);
		auto Q = this->generate_process_noise_covariance(F);

		std::cout << "FQ" << Q << std::endl;

		auto P = this->P_mat;

		auto F_mat = Matrix<double, 9, 9>({
			{ 1  , 0  , 0   , time, 0   , 0   , acsq, 0   , 0},
			{ 0  , 1  , 0   , 0   , time, 0   , 0   , acsq, 0},
			{ 0  , 0  , 1   , 0   , 0   , time, 0   , 0   , acsq},
			{ 0  , 0  , 0   , 1	, 0   , 0   , time, 0   , 0},
			{ 0  , 0  , 0   , 0   , 1   , 0   , 0   , time, 0},
			{ 0  , 0  , 0   , 0   , 0   , 1   , 0   , 0   , time},
			{ 0  , 0  , 0   , 0   , 0   , 0   , 1   , 0   , 0},
			{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 1   , 0},
			{ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 1},
		});

		if (type == InputType::ACCELERATION) {
			H = this->H_mat_acceleration;
			R = this->R_mat_acceleration;
			Z = inputs;
		} else {
			H = this->H_mat_position;
			R = this->R_mat_position;
			Z = inputs;
		}

		std::cout << "HALFWAY" << std::endl;

		auto X_hat = F * this->state;

		P = (F * P * F.transpose()) + Q;

		std::cout << "1: " << P << std::endl;

		auto K = P * H.transpose() * (H * P * H.transpose() + R).pow(-1);

		std::cout << "2: " << K << std::endl;

		X_hat = X_hat + K * (Z - H * X_hat);

		std::cout << "3: " << X_hat << std::endl;

		P = ( this->identity - K * H ) * P * (this->identity - K * H).transpose() + K * R * K.transpose();


		std::cout << "4: " << P << std::endl;


		this->state = X_hat;
		this->P_mat = P;

		predicted_pos[0][0] = this->state[0][0];
		predicted_pos[1][0] = this->state[3][0];
		predicted_pos[2][0] = this->state[6][0];


		return predicted_pos;
	}

	[[nodiscard]] const Vector<double, Nx>& get_state() const {
		return (this->currenState);
	}



	ProcessNoiseCovariance generate_process_noise_covariance(const StateTransitionMatrix &F_mat) {
		auto Q_mat = ProcessNoiseCovariance({
			std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
			std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
			std::array<double, Nx>({ 0  , 0  , 1   , 0   , 0   , 0   , 0   , 0   , 0}),
			std::array<double, Nx>({ 0  , 0  , 0   , 0	 , 0   , 0   , 0   , 0   , 0}),
			std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
			std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 1   , 0   , 0   , 0}),
			std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
			std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
			std::array<double, Nx>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 1}),
		});

		return (F_mat * Q_mat * F_mat.transpose()) * 0.001;
	}

	void set_state(std::array<double, Nx> &state) {
		this->currenState = Matrix<double, Nx, 1>(state);
	}

	Matrix<double, Nx, 1> get_initial_process_noise() {
		const Vector3d gps_noise({0.1, 0.1, 0.1});
		const Vector3d acceleration_noise({0.001, 0.001, 0.001});
		const Vector3d gyroscope_noise({0.01, 0.01, 0.01});

		auto x = gps_noise.vstack(acceleration_noise).vstack(gyroscope_noise);
		return (x);
	}

	double get_current_speed() {
		auto speed = std::sqrt(std::pow(this->currenState[0][3], 2) + std::pow(this->currenState[0][4], 2) + std::pow(this->currenState[0][5], 2));

		return speed;
	}
};




#endif // FT_KALMAN_KALMANFILTER_HPP
