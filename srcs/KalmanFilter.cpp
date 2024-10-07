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

Matrix<double, n, n> KalmanFilter::extrapolate_covariance(Matrix<double, n, n> F_mat, Matrix<double, n, n> P_mat) {
	// Pn+1,n=FPn,nFT
	auto P_n1_mat = F_mat * P_mat * F_mat.transpose() + this->Q_mat;

	return P_n1_mat;
}

Matrix<double, n, 1> KalmanFilter::calculate_measurement_vector(const Matrix<double, 6, 1> &inputs) {
	// z_n = Hx_n + v_n

	auto z_n = this->H_mat * inputs;

	return z_n;
}

Matrix<double, n, n> KalmanFilter::calculate_kalman_gain() {
	// Kn = Pn,n-1HT(HPn,n-1HT + Rn)^-1

	auto k_n = this->P_mat * this->H_mat.transpose() * (this->H_mat * this->P_mat * this->H_mat.transpose() + this->R_mat).pow(-1);

	return k_n;
}

Matrix<double, n, 1> KalmanFilter::update_state_matrix(Matrix<double, n, n> &kalman, Matrix<double, n, 1> x_prev, Matrix<double, n, 1> z_n) {
	// x_n,n = x_n,n-1 + Kn(Zn - Hx_n,n-1)

	auto x_n_n = x_prev + kalman * (z_n - this->H_mat * x_prev);

	return x_n_n;
}

Matrix<double, n, n> KalmanFilter::update_covariance_matrix(Matrix<double, n, n> &kalman) {
	// P_n,n = (I - KnH)P_n,n-1(I - KnH)T + KnRnKnT

	auto p_n_n = 
		(this->identity - kalman * this->H_mat) * 
		this->P_mat * 
		(this->identity - kalman * this->H_mat).transpose() +
		kalman * this->R_mat * kalman.transpose();

	return p_n_n;
}

Vector3d KalmanFilter::predict(size_t time_step, const Matrix<double, 6, 1>& inputs) {
	Vector3d predicted_pos;

	(void)time_step;

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

	this->state = F_mat * this->state;
	this->P_mat = this->extrapolate_covariance(F_mat, this->P_mat);

	auto measurement = this->calculate_measurement_vector(inputs);
	auto kalman = this->calculate_kalman_gain();
	auto updated_state = this->update_state_matrix(kalman, this->state, measurement);

	this->state = updated_state;

	auto updated_covariance = this->update_covariance_matrix(kalman);

	this->P_mat = updated_covariance;

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
