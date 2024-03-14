//
// Created by peer on 23-2-24.
//

#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(const Data& data) : state(data.get_position().vstack(data.get_direction()).vstack(data.get_acceleration())) {
}


Vector<double, n> get_initial_process_noise() {
	const Vector3d gps_noise({0.1, 0.1, 0.1});
	const Vector3d acceleration_noise({0.001, 0.001, 0.001});
	const Vector3d gyroscope_noise({0.01, 0.01, 0.01});

	auto x = gps_noise.vstack(acceleration_noise).vstack(gyroscope_noise);
	return (x);
}

const Vector<double, n>& KalmanFilter::get_state() const {
	return (this->state);
}

Vector3d KalmanFilter::predict(double time_step, const Vector3d& acceleration) {
	Vector3d predicted_pos;
	for (int i = 0; i < 3; i++) {
		this->state[i][0] +=  0.5 * time_step * acceleration[i][0] * acceleration[i][0];
		predicted_pos[i][0] = this->state[i][0];
	}
//	auto predicted_mu = A * mu_t + B * u_t;
//	auto predicted_sigma = A * sigma_t * A.transpose() + Q;
	return predicted_pos;

//	x = x0 + vx0 * Δt + 1/2 * ax * Δt^2
//	y = y0 + vy0 * Δt + 1/2 * ay * Δt^2
//	z = z0 + vz0 * Δt + 1/2 * az * Δt^2
}
