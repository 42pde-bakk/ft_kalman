//
// Created by peer on 8-10-24.
//
#include <catch2/catch_all.hpp>
#include "Matrix.hpp"
constexpr double EPSILON = 0.0001;


TEST_CASE("Matrix default constructor", "[Matrix]") {
	const Matrix<double, 2, 2> mat{};
	REQUIRE(mat.get_row_amount() == 2);
	REQUIRE(mat.get_column_amount() == 2);
}

TEST_CASE("Matrix constructor from std::array", "[Matrix]") {
	const std::array<std::array<double, 2>, 2> arr = {{{1.0, 2.0}, {3.0, 4.0}}};
	const Matrix<double, 2, 2> mat(arr);

	REQUIRE_THAT(mat[0][0], Catch::Matchers::WithinAbs(1.0, EPSILON));
	REQUIRE_THAT(mat[0][1], Catch::Matchers::WithinAbs(2.0, EPSILON));
	REQUIRE_THAT(mat[1][0], Catch::Matchers::WithinAbs(3.0, EPSILON));
	REQUIRE_THAT(mat[1][1], Catch::Matchers::WithinAbs(4.0, EPSILON));
}

TEST_CASE("Matrix constructor from scalar", "[Matrix]") {
	const Matrix<double, 2, 2> mat(5.0);

	REQUIRE_THAT(mat[0][0], Catch::Matchers::WithinAbs(5.0, EPSILON));
	REQUIRE_THAT(mat[0][1], Catch::Matchers::WithinAbs(5.0, EPSILON));
	REQUIRE_THAT(mat[1][0], Catch::Matchers::WithinAbs(5.0, EPSILON));
	REQUIRE_THAT(mat[1][1], Catch::Matchers::WithinAbs(5.0, EPSILON));
}

TEST_CASE("Matrix multiplication order", "[Matrix]") {
	constexpr size_t m = 5, n = 3, p = 4;
	const Matrix<double, m, n> lhs(8.0);
	const Matrix<double, n, p> rhs(2.0);

	const auto result = lhs * rhs;
	REQUIRE(result.get_row_amount() == m);
	REQUIRE(result.get_column_amount() == p);
}

TEST_CASE("Matrix multiplication", "[Matrix]") {
	const std::array<std::array<double, 2>, 2> lhs_arr = {{{1.0, 2.0}, {3.0, 4.0}}};
	const std::array<std::array<double, 2>, 2> rhs_arr = {{{5.0, 6.0}, {7.0, 8.0}}};
	const Matrix<double, 2, 2> lhs(lhs_arr);
	const Matrix<double, 2, 2> rhs(rhs_arr);

	const Matrix<double, 2, 2> result = lhs * rhs;

	REQUIRE_THAT(result[0][0], Catch::Matchers::WithinAbs(19.0, EPSILON));
	REQUIRE_THAT(result[0][1], Catch::Matchers::WithinAbs(22.0, EPSILON));
	REQUIRE_THAT(result[1][0], Catch::Matchers::WithinAbs(43.0, EPSILON));
	REQUIRE_THAT(result[1][1], Catch::Matchers::WithinAbs(50.0, EPSILON));
}

TEST_CASE("Matrix scalar multiplication", "[Matrix]") {
	const Matrix<double, 2, 2> mat(2.0);
	const Matrix<double, 2, 2> result = mat * 3.0;

	REQUIRE_THAT(result[0][0], Catch::Matchers::WithinAbs(6.0, EPSILON));
	REQUIRE_THAT(result[0][1], Catch::Matchers::WithinAbs(6.0, EPSILON));
	REQUIRE_THAT(result[1][0], Catch::Matchers::WithinAbs(6.0, EPSILON));
	REQUIRE_THAT(result[1][1], Catch::Matchers::WithinAbs(6.0, EPSILON));
}

TEST_CASE("Matrix transpose order", "[Matrix]") {
	const std::array<std::array<double, 4>, 2> arr = {{{1.0, 2.0, 3.0, 4.0}, {3.0, 4.0, 5.0, 6.0}}};
	const Matrix<double, 2, 4> mat(arr);

	const auto transposed = mat.transpose();
	REQUIRE(transposed.get_row_amount() == mat.get_column_amount());
	REQUIRE(transposed.get_column_amount() == mat.get_row_amount());
}

TEST_CASE("Matrix transpose", "[Matrix]") {
	const std::array<std::array<double, 2>, 2> arr = {{{1.0, 2.0}, {3.0, 4.0}}};
	const Matrix<double, 2, 2> mat(arr);

	const Matrix<double, 2, 2> transposed = mat.transpose();

	REQUIRE_THAT(transposed[0][0], Catch::Matchers::WithinAbs(1.0, EPSILON));
	REQUIRE_THAT(transposed[0][1], Catch::Matchers::WithinAbs(3.0, EPSILON));
	REQUIRE_THAT(transposed[1][0], Catch::Matchers::WithinAbs(2.0, EPSILON));
	REQUIRE_THAT(transposed[1][1], Catch::Matchers::WithinAbs(4.0, EPSILON));
}

TEST_CASE("Matrix addition", "[Matrix]") {
	const std::array<std::array<double, 2>, 2> lhs_arr = {{{1.0, 2.0}, {3.0, 4.0}}};
	const std::array<std::array<double, 2>, 2> rhs_arr = {{{5.0, 6.0}, {7.0, 8.0}}};
	const Matrix<double, 2, 2> lhs(lhs_arr);
	const Matrix<double, 2, 2> rhs(rhs_arr);

	const Matrix<double, 2, 2> result = lhs + rhs;

	REQUIRE_THAT(result[0][0], Catch::Matchers::WithinAbs(6.0, EPSILON));
	REQUIRE_THAT(result[0][1], Catch::Matchers::WithinAbs(8.0, EPSILON));
	REQUIRE_THAT(result[1][0], Catch::Matchers::WithinAbs(10.0, EPSILON));
	REQUIRE_THAT(result[1][1], Catch::Matchers::WithinAbs(12.0, EPSILON));
}

TEST_CASE("Matrix subtraction", "[Matrix]") {
	const std::array<std::array<double, 2>, 2> lhs_arr = {{{5.0, 6.0}, {7.0, 8.0}}};
	const std::array<std::array<double, 2>, 2> rhs_arr = {{{1.0, 2.0}, {3.0, 4.0}}};
	const Matrix<double, 2, 2> lhs(lhs_arr);
	const Matrix<double, 2, 2> rhs(rhs_arr);

	const Matrix<double, 2, 2> result = lhs - rhs;

	REQUIRE_THAT(result[0][0], Catch::Matchers::WithinAbs(4.0, EPSILON));
	REQUIRE_THAT(result[0][1], Catch::Matchers::WithinAbs(4.0, EPSILON));
	REQUIRE_THAT(result[1][0], Catch::Matchers::WithinAbs(4.0, EPSILON));
	REQUIRE_THAT(result[1][1], Catch::Matchers::WithinAbs(4.0, EPSILON));
}

TEST_CASE("Matrix identity", "[Matrix]") {
	Matrix<double, 3, 3> identity = Matrix<double, 3, 3>::identity<3>();

	REQUIRE_THAT(identity[0][0], Catch::Matchers::WithinAbs(1.0, EPSILON));
	REQUIRE_THAT(identity[0][1], Catch::Matchers::WithinAbs(0.0, EPSILON));
	REQUIRE_THAT(identity[0][2], Catch::Matchers::WithinAbs(0.0, EPSILON));

	REQUIRE_THAT(identity[1][0], Catch::Matchers::WithinAbs(0.0, EPSILON));
	REQUIRE_THAT(identity[1][1], Catch::Matchers::WithinAbs(1.0, EPSILON));
	REQUIRE_THAT(identity[1][2], Catch::Matchers::WithinAbs(0.0, EPSILON));

	REQUIRE_THAT(identity[2][0], Catch::Matchers::WithinAbs(0.0, EPSILON));
	REQUIRE_THAT(identity[2][1], Catch::Matchers::WithinAbs(0.0, EPSILON));
	REQUIRE_THAT(identity[2][2], Catch::Matchers::WithinAbs(1.0, EPSILON));
}
