//
// Created by peer on 23-2-24.
//

#ifndef FT_KALMAN_KALMANFILTER_HPP
#define FT_KALMAN_KALMANFILTER_HPP

#include <fstream>
#include "Matrix.hpp"

#define NOISE 1
#define GPS_NOISE 0.1
#define GYROSCOPE_NOISE 0.01
#define ACCELEROMETER_NOISE 0.000001

// ACCELEROMETER should be 0.001 but okruitho had it at 0.000001
enum InputType {
    ACCELERATION,
    POSITION
};

constexpr double Ppos = 0.0,
        Pvel = 0.001,
        Pacl = 0.00001,
        Pmul = Pvel * Pacl;

template<size_t Nx, size_t Nz, size_t Nu>
class KalmanFilter {
public:
    using StateVector = Matrix<double, Nx, 1>;
    using MeasurementVector = Matrix<double, Nz, 1>;
    using EstimateCovarianceMatrix = Matrix<double, Nx, Nx>;
    using ObservationMatrix = Matrix<double, Nz, Nx>;
    using KalmanGainMatrix = Matrix<double, Nx, Nz>;
    // using ProcessNoiseCovariance = Matrix<double, Nx, Nx>;
    // using MeasurementCovariance = Matrix<double, Nz, Nz>;
    // using ProcessNoiseVector = Matrix<double, Nx, 1>;
    // using MeasurementNoiseVector = Matrix<double, Nz, 1>;
    // using StateTransitionMatrix = Matrix<double, Nx, Nx>;
    // using InputVector = Matrix<double, Nz, 1>;
    // using ControlMatrix = Matrix<double, Nx, Nu>;

    using RMatrix = Matrix<double, Nz, Nz>;
    using FMatrix = Matrix<double, Nx, Nx>;
    using QMatrix = FMatrix;

    const RMatrix R_acceleration = RMatrix::template diag<3>(ACCELEROMETER_NOISE);
    const RMatrix R_position = RMatrix::template diag<3>(GPS_NOISE);
    const RMatrix R_velocity = RMatrix::template diag<3>(GYROSCOPE_NOISE / ACCELEROMETER_NOISE);
    const Matrix<double, Nx, Nx> I = Matrix<double, 9, 9>::identity<9>();
    const ObservationMatrix H_velocity{
        {0, 1, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 0},
    };
    const ObservationMatrix H_acceleration{
        {0, 0, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 1},
    };
    const ObservationMatrix H_position{
        {1, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 0, 0},
    };

private:
    StateVector state; // x,xvel,xacc, y,yvel,yacc, z,zvel,zacc
    EstimateCovarianceMatrix P{
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
    // Also known as the TransitionMatrix
    static FMatrix makeFMatrix(const double timedelta) {
        const double vel = timedelta * timedelta * 0.5;
        FMatrix result = FMatrix::template identity<9>();
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

    void predict(const double timedelta) {
        const auto F = makeFMatrix(timedelta);
        const auto q = makeQMatrix(F);

        this->state = F * this->state;
        this->P = (F * this->P * F.transpose()) + q;
    }

    KalmanGainMatrix update(const ObservationMatrix& h, const RMatrix& r, const MeasurementVector& z) {
        kalmanGain = this->P * h.transpose() * (h * this->P * h.transpose() + r).pow(-1);
        this->state = this->state + kalmanGain * (z - h * this->state);

        this->P = (I - kalmanGain * h) * P * (I - kalmanGain * h).transpose() + kalmanGain * r * kalmanGain.transpose();

        return kalmanGain;
    }

    void set_state(const StateVector& stateVector) {
        this->state = stateVector;
    }

    StateVector get_state() {
        return (this->state);
    }

    void printState(std::ofstream& of) {
        for (size_t i = 0; i < Nx; i++) {
            of << this->state[i][0] << " ";
        }
        of << std::endl;
    }
};


#endif // FT_KALMAN_KALMANFILTER_HPP
