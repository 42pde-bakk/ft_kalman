//
// Created by peer on 23-2-24.
//

#include "KalmanFilter.hpp"
#include <iostream>

KalmanFilter::KalmanFilter() {
	this->state = Matrix<double, n, 1>();
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

Matrix<double, n, 1> KalmanFilter::calculate_measurement_vector(const Matrix<double, n, 1> &inputs) {
	// z_n = Hx_n + v_n

	auto z_n = this->H_mat * inputs;

	return z_n;
}

Matrix<double, n, n> KalmanFilter::calculate_kalman_gain() {
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

double KalmanFilter::get_current_speed() {
	auto speed = std::sqrt(std::pow(this->state[0][3], 2) + std::pow(this->state[0][4], 2) + std::pow(this->state[0][5], 2));
	std::cout << "SPEED" << speed << std::endl;

	return speed;
}

Vector3d KalmanFilter::predict(size_t time_step, const Matrix<double, n, 1>& inputs) {
	Vector3d predicted_pos;

	(void)time_step;
	(void)inputs;

	std::cout << "IN" << this->state << std::endl;

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

	std::cout << "STATE\n" << this->state << std::endl;

	this->P_mat = this->extrapolate_covariance(F_mat, this->P_mat);

	std::cout << "P_MAT\n" << this->P_mat << std::endl;

	auto measurement = this->calculate_measurement_vector(inputs);

	std::cout << "MEASUREMENT\n" << measurement << std::endl;

	auto kalman = this->calculate_kalman_gain();

	std::cout << "KALMAN GAIN\n" << kalman << std::endl;

	auto updated_state = this->update_state_matrix(kalman, this->state, measurement);

	std::cout << "UPDATED\n" << updated_state << std::endl;

	this->state = updated_state;

	auto updated_covariance = this->update_covariance_matrix(kalman);

	std::cout << "COV\n" << updated_covariance << std::endl;

	this->P_mat = updated_covariance;

	predicted_pos[0][0] = this->state[0][0];
	predicted_pos[1][0] = this->state[1][0];
	predicted_pos[2][0] = this->state[2][0];

//	auto predicted_mu = A * mu_t + B * u_t;
//	auto predicted_sigma = A * sigma_t * A.transpose() + Q;
	return predicted_pos;

//	x = x0 + vx0 * Δt + 1/2 * ax * Δt^2
//	y = y0 + vy0 * Δt + 1/2 * ay * Δt^2
//	z = z0 + vz0 * Δt + 1/2 * az * Δt^2
}
