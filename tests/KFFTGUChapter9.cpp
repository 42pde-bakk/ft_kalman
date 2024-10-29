//
// Created by peer on 10/29/24.
//

#include <catch2/catch_all.hpp>
#include "Matrix.hpp"
constexpr double EPSILON = 0.0001;


TEST_CASE("9.2", "[9 Multivariate KF Examples]") {
    // Here it is assumed that delta t = 1s.
    constexpr double dt = 1.0;
    constexpr double stddev_accel = 0.2; // 0.2m/s^2 random acceleration standard deviation
    constexpr double stddev_measure = 3; // 3m measurement error standard deviation

    // State Transition matrix F
    Matrix<double, 6, 6> F = {
        {1, dt, .5 * std::pow(dt, 2), 0, 0,  0},
        {0, 1,  dt,                   0, 0,  0},
        {0, 0,  1,                    0, 0,  0},
        {0, 0,  0,                    1, dt, .5 * std::pow(dt, 2)},
        {0, 0,  0,                    0, 0,  1}
    };

    // Process Noise matrix Q
    Matrix<double, 6, 6> Q = {
        {std::pow(dt, 4) / 4, std::pow(dt, 3) / 2, std::pow(dt, 2) / 2, 0,                   0,                   0},
        {std::pow(dt, 3) / 2, std::pow(dt, 2),     dt,                  0,                   0,                   0},
        {std::pow(dt, 2) / 2, dt,                  1,                   0,                   0,                   0},
        {0,                   0,                   0,                   std::pow(dt, 4) / 4, std::pow(dt, 3) / 2, std::pow(dt, 2) / 2},
        {0,                   0,                   0,                   std::pow(dt, 3) / 2, std::pow(dt, 2),     dt},
        {0,                   0,                   0,                   std::pow(dt, 2) / 2, dt,                  1}
    };
    Q = Q * std::pow(stddev_accel, 2);

    // Measurement Covariance R
    Matrix<double, 2, 2> R = {
        {std::pow(stddev_measure, 2), 0},
        {0,                           std::pow(stddev_measure, 2)}
    };
    Vector<double, 6> x_hat_00({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    Matrix<double, 6, 6> P_00({
        {500, 0, 0, 0, 0, 0},
        {0, 500, 0, 0, 0, 0},
        {0, 0, 500, 0, 0, 0},
        {0, 0, 0, 500, 0, 0},
        {0, 0, 0, 0, 500, 0},
        {0, 0, 0, 0, 0, 500},
    });

}