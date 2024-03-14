//
// Created by peer on 23-2-24.
//

#ifndef FT_KALMAN_MATRIX_HPP
#define FT_KALMAN_MATRIX_HPP

#include <cstddef>
#include <vector>
#include <array>
#include <iomanip>
#include <cmath>
#include <cassert>


template<typename T, size_t ROW_AMOUNT, size_t COLUMN_AMOUNT>
class Matrix {
	std::array<std::array<T, COLUMN_AMOUNT>, ROW_AMOUNT> data;

public:
	template <typename, size_t, size_t> friend class Matrix;
	Matrix() = default;

	Matrix(const Matrix& rhs) = default;

	Matrix(const std::vector<double>& vec) {
		assert(vec.size() == ROW_AMOUNT);
		for (size_t row = 0; row < ROW_AMOUNT; row++) {
			this->data[row][0] = vec[row];
		}
	}

	[[nodiscard]] size_t get_row_amount() const {
		return (ROW_AMOUNT);
	}

	[[nodiscard]] size_t get_column_amount() const {
		return (COLUMN_AMOUNT);
	}

	void set_vector(const std::vector<T>& v) {
		assert(v.size() == ROW_AMOUNT);
		for (size_t row = 0; row < ROW_AMOUNT; row++) {
			this->data[row][0] = v[row];
		}
	}

	const std::array<T, COLUMN_AMOUNT>& operator[](size_t i) const {
		return (this->data[i]);
	}

	template<size_t R, size_t C>
	Matrix operator*(const Matrix<T, R, C>& rhs) {
		assert(COLUMN_AMOUNT == R);
		Matrix<T, ROW_AMOUNT, COLUMN_AMOUNT> out;

		for (size_t a = 0; a < ROW_AMOUNT; a++) {
			for (size_t b = 0; b < C; b++) {
				for (size_t p = 0; p < COLUMN_AMOUNT; p++) {
					out[a][b] = std::fma(this->data[a][p], rhs.data[p][b], out.data[a][b]);
//					out[a][b] = std::fma((*this)[a][p], rhs[p][b], out[a][b]); // fma = fast multiply-add
//						out[a][b] += (*this)[a][p] * rhs[p][b];
				}
			}
		}
		return (out);
	}

	Matrix operator*(T scalar) const {
		Matrix out(*this);

		for (size_t row = 0; row < ROW_AMOUNT; row++) {
			for (size_t column = 0; column < COLUMN_AMOUNT; column++) {
				out[row][column] *= scalar;
			}
		}
		return (out);
	}

	[[nodiscard]] Matrix transpose() const {
		Matrix<T, COLUMN_AMOUNT, ROW_AMOUNT> out;

		for (size_t row = 0; row < ROW_AMOUNT; row++) {
			for (size_t column = 0; column < COLUMN_AMOUNT; column++) {
				out.data[column][row] = this->data[row][column];
			}
		}
		return (out);
	}

	friend std::ostream& operator<<(std::ostream& o, const Matrix& m) {
		for (size_t row = 0; row < ROW_AMOUNT; row++) {
			for (size_t column = 0; column < COLUMN_AMOUNT; column++) {
				o << std::setprecision(5) << m.data[row][column] << ' ';
			}
			o << '\n';
		}
		return (o);
	}

	template<size_t R>
	[[nodiscard]] Matrix<double, ROW_AMOUNT + R, COLUMN_AMOUNT>	vstack(const Matrix<T, R, COLUMN_AMOUNT>& rhs) const {
		Matrix<double, ROW_AMOUNT + R, COLUMN_AMOUNT> out;
		for (size_t r = 0; r < R; r++) {
			out.data[r] = this->data[r];
		}
		for (size_t r = 0; r < R; r++) {
			out.data[R + r] = rhs.data[r];
		}
		return (out);
	}

};

template<typename T, size_t ROW_AMOUNT_VEC>
using Vector = Matrix<T, ROW_AMOUNT_VEC, 1>;
using Vector3d = Matrix<double, 3, 1>;

#endif //FT_KALMAN_MATRIX_HPP
