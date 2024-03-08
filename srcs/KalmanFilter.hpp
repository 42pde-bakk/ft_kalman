//
// Created by peer on 23-2-24.
//

#ifndef FT_KALMAN_KALMANFILTER_HPP
#define FT_KALMAN_KALMANFILTER_HPP

#include "Matrix.hpp"

#define n 9 // n is the amount of state variables
// n: [x, y, z, vx, vy, vz, and three rotations]
#define p 3 // p is the amount of inputs
// p: []
#define m 3 // m is the amount of outputs
// m: []



class KalmanFilter {
	Vector<double, n> state{};
	unsigned int k;
	// Matrices
	Matrix<double, n, n> A; // state gain
	Matrix<double, n, p> B; // input gain
	Matrix<double, m, n> H; // output gain
	Vector<double, n> w; // process noise
	Matrix<double, n, n> Q; // process noise covariance
	Vector<double, m> v; // measurement noise
	Matrix<double, m, m> R; // measurement noise covariance
	Matrix<double, n, n> Pmin; // a priori covariance
	Matrix<double, n, n> Pplus; // a posteriori covariance
	Matrix<double, n, m> K; // Kalman Filter Gain

public:
	KalmanFilter();

	Vector3d predict(double time_step, const Vector3d& acceleration);

	[[nodiscard]] const Vector<double, 9>& get_state() const;

};


#endif //FT_KALMAN_KALMANFILTER_HPP
