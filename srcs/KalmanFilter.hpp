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

	Matrix<double, n, n> P_mat = Matrix<double, 9, 9>({
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
	Matrix<double, n, n> Q_mat = Matrix<double, 9, 9>({
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

	Matrix<double, 9, 9> H_mat = Matrix<double, 9, 9>({
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

	Matrix<double, n, n> R_mat = Matrix<double, n, n>({
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

	Vector3d predict(size_t time_step, const Matrix<double, n, 1>& inputs);

	[[nodiscard]] const Vector<double, n>& get_state() const;

	Matrix<double, n, n> extrapolate_covariance(Matrix<double, n, n> F_mat, Matrix<double, n, n> P_mat);

	Matrix<double, n, n> get_state_transition_matrix(double time_step);

	Matrix<double, n, 1> calculate_measurement_vector(const Matrix<double, n, 1> &inputs);

	Matrix<double, n, n> calculate_kalman_gain();

	Matrix<double, n, 1> update_state_matrix(Matrix<double, n, n> &kalman, Matrix<double, n, 1> x_prev, Matrix<double, n, 1> z_n);

	Matrix<double, n, n> update_covariance_matrix(Matrix<double, n, n> &kalman);

	void set_state(std::array<double, n> &state) {
		this->state = Matrix<double, n, 1>(state);
	}

	double get_current_speed();
};


#endif // FT_KALMAN_KALMANFILTER_HPP