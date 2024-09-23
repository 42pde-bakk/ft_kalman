//
// Created by peer on 23-2-24.
//

#include "KalmanFilter.hpp"
#include <iostream>

KalmanFilter::KalmanFilter(const Data& data) {
	this->state = data.get_position().vstack(data.get_acceleration()).vstack(data.calculate_velocity());

	this->process_noise_covariance_matrix = Matrix<double, n, n>({
		std::array<double, n>({pow(GPS_NOISE, 2), 0, 0, 0, 0, 0}),
		std::array<double, n>({0, pow(GPS_NOISE, 2), 0, 0, 0, 0}),
		std::array<double, n>({0, 0, pow(GPS_NOISE, 2), 0, 0, 0}),
		std::array<double, n>({0, 0, 0, (pow(GYROSCOPE_NOISE, 2) + pow(ACCELEROMETER_NOISE, 2) * DT), 0, 0}),
		std::array<double, n>({0, 0, 0, 0, (pow(GYROSCOPE_NOISE, 2) + pow(ACCELEROMETER_NOISE, 2) * DT), 0}),
		std::array<double, n>({0, 0, 0, 0, 0, (pow(GYROSCOPE_NOISE, 2) + pow(ACCELEROMETER_NOISE, 2) * DT)})
	});
}


Matrix<double, n, 1> get_initial_process_noise() {
	const Vector3d gps_noise(std::array<double, 3>({0.1, 0.1, 0.1}));
	const Vector3d acceleration_noise(std::array<double, 3>({0.001, 0.001, 0.001}));
	const Vector3d gyroscope_noise(std::array<double, 3>({0.01, 0.01, 0.01}));

	auto x = gps_noise.vstack(acceleration_noise).vstack(gyroscope_noise);
	return (x);
}

const Vector<double, n>& KalmanFilter::get_state() const {
	return (this->state);
}

Vector3d KalmanFilter::predict(size_t time_step, const Vector3d& acceleration) {
	Vector3d predicted_pos;

	(void)acceleration;
	(void)time_step;

	auto time = (double)time_step / 1000;

	// acceleration squared.
	auto acsq = 0.5 * time * time;

	auto F_vec = Matrix<double, 9, 9>({
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

	this->state = F_vec * this->state;

	predicted_pos[0][0] = this->state[0][0];
	predicted_pos[1][0] = this->state[1][0];
	predicted_pos[2][0] = this->state[2][0];

	std::cout << this->state << "\n-----" << std::endl;

//	auto predicted_mu = A * mu_t + B * u_t;
//	auto predicted_sigma = A * sigma_t * A.transpose() + Q;
	return predicted_pos;

//	x = x0 + vx0 * Δt + 1/2 * ax * Δt^2
//	y = y0 + vy0 * Δt + 1/2 * ay * Δt^2
//	z = z0 + vz0 * Δt + 1/2 * az * Δt^2
}


Matrix<double, n, n> KalmanFilter::get_state_transition_matrix(double time_step) {
	auto mat = Matrix<double, n, n>::identity<n>();
	for (size_t i = 0; i < 6; i++) {
		mat[i][i + 3] = time_step;
	}
	for (size_t i = 0; i < 3; i++) {
		mat[i][i + 6] = 0.5 * time_step * time_step;
	}
	return (mat);
}
