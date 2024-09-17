//
// Created by peer on 23-2-24.
//

#ifndef FT_KALMAN_KALMANFILTER_HPP
#define FT_KALMAN_KALMANFILTER_HPP

#include "Matrix.hpp"
#include "Data.hpp"

#define n 9 // n is the amount of state variables
// n: 3 positions, 3 directions, 3 accelerations
// n: [x, y, z, dir, v, acc_x, acc_y, acc_z] (no direction because they won't ever change)
#define p 3 // p is the amount of inputs
// p: []
#define m 3 // m is the amount of outputs
// m: []

#define NOISE 1
#define GPS_NOISE (NOISE * 0.1)
#define GYROSCOPE_NOISE (NOISE * 0.01)
#define ACCELEROMETER_NOISE (NOISE * 0.001)

class KalmanFilter {
	Vector<double, n> state{};
	unsigned int k;
	// Matrices
	Matrix<double, n, n> state_covariance_matrix;
	Matrix<double, n, n> state_transition_matrix;
	Matrix<double, m, n> state_to_measurement_matrix;
	Matrix<double, m, m> measurement_covariance_matrix;
	Matrix<double, n, n> process_noise_covariance_matrix;
	Matrix<double, n, m> kalman_gain_matrix;

	KalmanFilter() = default;
public:
	explicit KalmanFilter(const Data& data);

	Vector3d predict(size_t time_step, const Vector3d& acceleration);

	[[nodiscard]] const Vector<double, n>& get_state() const;

	Matrix<double, n, n> get_state_transition_matrix(double time_step);
};


#endif //FT_KALMAN_KALMANFILTER_HPP
