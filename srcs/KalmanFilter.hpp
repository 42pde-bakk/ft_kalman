//
// Created by peer on 23-2-24.
//

#ifndef FT_KALMAN_KALMANFILTER_HPP
#define FT_KALMAN_KALMANFILTER_HPP

#include "Matrix.hpp"
#include "Data.hpp"

#define Nx 9
#define Nz 9
#define Nu 9
#define n 9 // n is the amount of state variables
// n: 3 positions, 3 directions, 3 accelerations
// n: [x, y, z, dir, v, acc_x, acc_y, acc_z] (no direction because they won't ever change)
#define p 3 // p is the amount of inputs
// p: []
#define m 3 // m is the amount of outputs
// m: []

#define NOISE 1
#define GPS_NOISE 0.1
#define GYROSCOPE_NOISE 0.01
#define ACCELEROMETER_NOISE 0.001

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
	Vector<double, n> state{};
	unsigned int k;
	// Matrices
	Matrix<double, n, n> state_covariance_matrix;
	Matrix<double, n, n> state_transition_matrix;
	Matrix<double, m, n> state_to_measurement_matrix;
	Matrix<double, m, m> measurement_covariance_matrix;
	Matrix<double, n, m> kalman_gain_matrix;

	EstimateCovarianceMatrix P_mat = EstimateCovarianceMatrix({
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   ,0    , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0	, 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
	});

	// TODO: fill this noise matrix
	ProcessNoiseCovariance Q_mat = ProcessNoiseCovariance({
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   ,0    , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0	, 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
	});

	ObservationMatrix H_mat = ObservationMatrix({
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 1   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 1   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 1   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0	, 0   , 0   , 1   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 1   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 1}),
	});

	MeasurementCovariance R_mat = MeasurementCovariance({
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   ,0    , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , GYROSCOPE_NOISE	, 0   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , GYROSCOPE_NOISE   , 0   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , GYROSCOPE_NOISE   , 0   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE   , 0   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE   , 0}),
		std::array<double, 9>({ 0  , 0  , 0   , 0   , 0   , 0   , 0   , 0   , ACCELEROMETER_NOISE}),
	});

	Matrix<double, n, n> identity = Matrix<double, n, n>({
		std::array<double, 9>({ 1  , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 }),
		std::array<double, 9>({ 0  , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 }),
		std::array<double, 9>({ 0  , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 }),
		std::array<double, 9>({ 0  , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 }),
		std::array<double, 9>({ 0  , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 }),
		std::array<double, 9>({ 0  , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 }),
		std::array<double, 9>({ 0  , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 }),
		std::array<double, 9>({ 0  , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 }),
		std::array<double, 9>({ 0  , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 }),
	});

public:
	explicit KalmanFilter();

	Vector3d predict(size_t time_step, const InputVector &inputs);

	[[nodiscard]] const Vector<double, n>& get_state() const;

	///

	EstimateCovarianceMatrix extrapolate_covariance(const StateTransitionMatrix F_mat, const EstimateCovarianceMatrix P_mat);

	MeasurementVector calculate_measurement_vector(const InputVector &inputs);

	KalmanGain calculate_kalman_gain();

	StateVector update_state_matrix(KalmanGain &kalman, StateVector x_prev, MeasurementVector z_n);

	EstimateCovarianceMatrix update_covariance_matrix(KalmanGain &kalman);

	///

	void set_state(std::array<double, n> &state) {
		this->state = Matrix<double, n, 1>(state);
	}

	double get_current_speed();
};


#endif // FT_KALMAN_KALMANFILTER_HPP