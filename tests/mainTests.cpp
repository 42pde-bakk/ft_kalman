//
// Created by peer on 8-10-24.
//
#define CONFIG_CATCH_MAIN
#include <catch2/catch_all.hpp>
#include <catch2/catch_approx.hpp>


#include "../srcs/KalmanFilter.hpp"
#include "../srcs/Matrix.hpp"

// Define dimensions for the Kalman Filter test
constexpr size_t Nx = 9;  // State vector size
constexpr size_t Nz = 3;  // Measurement vector size
constexpr size_t Nu = 3;  // Control input size

TEST_CASE("KalmanFilter Initialization") {
	KalmanFilter<Nx, Nz, Nu> filter;

	SECTION("Initial State is Zero") {
		auto state = filter.get_state();
		for (size_t i = 0; i < Nx; i++) {
			REQUIRE(state[i][0] == Catch::Approx(0.0));
		}
	}

	SECTION("Initial Process Noise") {
		auto noise = filter.get_initial_process_noise();
		REQUIRE(noise[0][0] == Catch::Approx(0.1));  // GPS noise
		REQUIRE(noise[6][0] == Catch::Approx(0.01)); // Gyroscope noise
	}
}

TEST_CASE("State Transition Matrix") {
	KalmanFilter<Nx, Nz, Nu> filter;
	double time_step = 0.1;  // 100ms time step

	SECTION("State Transition Matrix is correctly computed") {
		auto F = filter.get_state_transition_matrix(time_step);

		// Check a few specific values in the transition matrix
		REQUIRE(F[0][0] == Catch::Approx(1.0));
		REQUIRE(F[0][3] == Catch::Approx(0.1)); // velocity component
		REQUIRE(F[0][6] == Catch::Approx(0.5 * time_step * time_step)); // acceleration component
	}
}

TEST_CASE("Covariance Matrix Update") {
	KalmanFilter<Nx, Nz, Nu> filter{};
	double time_step = 0.1;

	SECTION("Covariance Matrix after extrapolation") {
		auto F = filter.get_state_transition_matrix(time_step);
		auto P_before = filter.extrapolate_covariance(F, filter.get_state_transition_matrix(time_step));

		// Ensure covariance matrix is updated correctly
		auto P_after = filter.extrapolate_covariance(F, P_before);

		// Check that the matrix is still positive definite
		INFO("P_after: " << P_after);
		REQUIRE(P_after[0][0] >= 0);
	}
}

//TEST_CASE("Kalman Gain Calculation") {
//	KalmanFilter<Nx, Nz, Nu> filter;
//
//	SECTION("Kalman Gain Matrix Calculation") {
//		auto kalman_gain = filter.calculate_kalman_gain();
//
//		// Ensure Kalman Gain matrix has reasonable values
//		REQUIRE(kalman_gain[0][0] >= 0);
//	}
//}

//TEST_CASE("State Update with Measurement") {
//	KalmanFilter<Nx, Nz, Nu> filter;
//	std::array<double, Nx> initial_state = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//	filter.set_state(initial_state);
//
//	SECTION("State Update After Measurement") {
//		Matrix<double, Nx, 1> measurement{};
//		measurement[0][0] = 2.0;  // Simulate a new measurement on position x
//
//		auto kalman_gain = filter.calculate_kalman_gain();
//		auto updated_state = filter.update_state_matrix(kalman_gain, filter.get_state(), measurement);
//
//		// Ensure the state is updated towards the measurement
//		REQUIRE(updated_state[0][0] > 1.0);
//		REQUIRE(updated_state[0][0] <= 2.0);
//	}
//}
