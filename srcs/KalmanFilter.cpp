//
// Created by peer on 23-2-24.
//

#include "KalmanFilter.hpp"

Vector<double, n> get_initial_process_noise() {
	const Vector3d gps_noise({0.1, 0.1, 0.1});
	const Vector3d acceleration_noise({0.001, 0.001, 0.001});
	const Vector3d gyroscope_noise({0.01, 0.01, 0.01});

	auto x = gps_noise.vstack(acceleration_noise);
}

const Vector<double, 9>& KalmanFilter::get_state() const {
	return (this->state);
}

Vector3d KalmanFilter::predict(double time_step, const Vector3d& acceleration) {
	auto predicted_mu = A * mu_t + B * u_t;
	auto predicted_sigma = A * sigma_t * A.transpose() + Q;
}

KalmanFilter::KalmanFilter() : k(0) {
}
