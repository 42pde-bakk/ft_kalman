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
#define ACCELEROMETER_NOISE 0.000001
// ACCELEROMETER should be 0.001 but okruitho had it at 0.000001
enum InputType {
	ACCELERATION,
	POSITION
};

constexpr double	Ppos = 0.0,
					Pvel = 0.001,
					Pacl = 0.00001,
					Pmul = Pvel * Pacl;

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
	using KalmanGainMatrix = Matrix<double, Nx, Nz>;

	using CovarianceMatrix = Matrix<double, 3, 9>;
	using RMatrix = Matrix<double, 3, 3>;
	using FMatrix = Matrix<double, 9, 9>;
	using QMatrix = FMatrix;

	const RMatrix R_acceleration = RMatrix::diag<3>(ACCELEROMETER_NOISE);
	const RMatrix R_position = RMatrix::diag<3>(GPS_NOISE);
	const RMatrix R_velocity = RMatrix::diag<3>(GYROSCOPE_NOISE);
	const Matrix<double, 9, 9> I = Matrix<double, 9, 9>::identity<9>();
	const CovarianceMatrix H_velocity{
			{0, 1, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 1, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 1, 0},
		};
	const CovarianceMatrix H_acceleration{
			{0, 0, 1, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 1, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 1},
		};
	const CovarianceMatrix H_position{
			{1, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 1, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 1, 0, 0},
		};

private:
	StateVector state{}; // x,xvel,xacc, y,yvel,yacc, z,zvel,zacc
	StateVector X_hat{};
	Matrix<double, 9, 9> P{
		{Ppos, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, Pvel, Pmul, 0, 0, 0, 0, 0, 0},
		{0, Pmul, Pacl, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, Ppos, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, Pvel, Pmul, 0, 0, 0},
		{0, 0, 0, 0, Pmul, Pacl, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, Ppos, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, Pvel, Pmul},
		{0, 0, 0, 0, 0, 0, 0, Pmul, Pacl},
	};
	KalmanGainMatrix kalmanGain;


public:
	static FMatrix makeFMatrix(const double timedelta) {
		const double vel = timedelta * timedelta * 0.5;
		FMatrix result = FMatrix::identity<9>();
		for (size_t i = 0; i < 3; i++) {
			result[i * 3][i * 3 + 1] = timedelta;
			result[i * 3][i * 3 + 2] = vel;
			result[i * 3 + 1][i * 3 + 2] = timedelta;
		}
		return (result);
	}
	static QMatrix makeQMatrix(const FMatrix& f) {
		QMatrix q;
		for (size_t i = 0; i < 3; i++) {
			q[i * 3 + 2][i * 3 + 2] = 1;
		}
		return (f * q * f.transpose()) * 0.001;
	}

	KalmanGainMatrix update(const CovarianceMatrix& h, const RMatrix& r, const Matrix<double, 3, 1>& z, const double timedelta) {
		const auto f = makeFMatrix(timedelta);
		const auto q = makeQMatrix(f);

		// predict step
		X_hat = f * X_hat;
		P = (f * P * f.transpose()) + q;
		kalmanGain = P * h.transpose() * (h * P * h.transpose() + r).pow(-1);

		X_hat = X_hat + kalmanGain * (z - h * X_hat);

		P = (I - kalmanGain * h) * P * (I - kalmanGain * h).transpose() + kalmanGain * r * kalmanGain.transpose();

		return kalmanGain;
	}

	void set_state(const StateVector& stateVector) {
		this->state = stateVector;
		this->X_hat = stateVector;
	}

	StateVector get_state() {
		return (this->X_hat);
	}

};




#endif // FT_KALMAN_KALMANFILTER_HPP
