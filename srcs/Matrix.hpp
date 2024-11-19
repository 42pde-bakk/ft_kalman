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

// Primary template
template <typename Container, typename = void>
struct is_container : std::false_type {};
template <typename T, std::size_t N>
struct is_container<std::array<T, N>> : std::true_type {};
template <typename T>
struct is_container<std::vector<T>> : std::true_type {};

template<typename T, size_t ROW_AMOUNT, size_t COLUMN_AMOUNT>
class Matrix {
	std::array<std::array<T, COLUMN_AMOUNT>, ROW_AMOUNT> data;

public:
	template <typename, size_t, size_t> friend class Matrix;

	Matrix() {
		for (size_t row = 0; row < ROW_AMOUNT; row++) {
			for (size_t col = 0; col < COLUMN_AMOUNT; col++) {
				this->data[row][col] = T();
			}
		}
	};

	Matrix &operator=(const Matrix &rhs) {
		this->data = rhs.data;

		return *this;
	}

	Matrix(const Matrix& rhs) {
		this->data = rhs.data;
	}

	// Generic constructor for containers
	// template <typename Container>
	// Matrix(const Container& data_in,
	//        std::enable_if_t<is_container<Container>::value>* = nullptr) {
	// 	assert(data_in.size() == ROW_AMOUNT);
	// 	size_t row = 0;
	// 	for (auto& row_list : data_in) {
	// 		assert(data_in.size() == COLUMN_AMOUNT);
	// 		std::copy(row_list.begin(), row_list.end(), this->data[row].begin());
	// 		++row;
	// 	}
	// }
	Matrix(std::initializer_list<std::initializer_list<T>> init_list) {
		assert(init_list.size() == ROW_AMOUNT);
		size_t row = 0;
		for (auto& row_list : init_list) {
			assert(row_list.size() == COLUMN_AMOUNT);
			std::copy(row_list.begin(), row_list.end(), data[row].begin());
			++row;
		}
	}


	// Constructor for creating a vector (1-column matrix)
	template <typename Container>
	Matrix(const Container& data_in, std::enable_if_t<is_container<Container>::value && (COLUMN_AMOUNT == 1)>* = nullptr) {
		assert(data_in.size() == ROW_AMOUNT);
		assert(COLUMN_AMOUNT == 1);
		std::copy(data_in.cbegin(), data_in.cend(), this->data[0].begin());
	}
	Matrix(const std::initializer_list<T>& data_in) {
		assert(data_in.size() == ROW_AMOUNT);
		assert(COLUMN_AMOUNT == 1);
		std::copy(data_in.begin(), data_in.end(), this->data[0].begin());
	}


	Matrix(T n) {
		for (size_t row = 0; row < ROW_AMOUNT; row++) {
			for (size_t col = 0; col < COLUMN_AMOUNT; col++) {
				this->data[row][col] = n;
			}
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
	std::array<T, COLUMN_AMOUNT>& operator[](size_t i) {
		return (this->data[i]);
	}

	// For matrix multiplication, the number of columns in the first matrix must be equal to the number of rows in the second matrix.
	// The result matrix has the number of rows of the first and the number of columns of the second matrix.
	template<size_t ROW_AMOUNT_2, size_t COLUMN_AMOUNT_2>
	Matrix<T, ROW_AMOUNT, COLUMN_AMOUNT_2> operator*(const Matrix<T, ROW_AMOUNT_2, COLUMN_AMOUNT_2>& rhs) const {
		assert(COLUMN_AMOUNT == ROW_AMOUNT_2);
		auto out = Matrix<double, ROW_AMOUNT, COLUMN_AMOUNT_2>();

		for (size_t a = 0; a < ROW_AMOUNT; a++) {
			for (size_t b = 0; b < COLUMN_AMOUNT_2; b++) {
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

	[[nodiscard]] Matrix<T, COLUMN_AMOUNT, ROW_AMOUNT> transpose() const {
		auto out = Matrix<T, COLUMN_AMOUNT, ROW_AMOUNT>();

		for (size_t row = 0; row < ROW_AMOUNT; row++) {
			for (size_t column = 0; column < COLUMN_AMOUNT; column++) {
				out.data[column][row] = this->data[row][column];
			}
		}
		return (out);
	}

	template<size_t R>
	[[nodiscard]] Matrix<T, ROW_AMOUNT + R, COLUMN_AMOUNT>	vstack(const Matrix<T, R, COLUMN_AMOUNT>& rhs) const {
		auto out = Matrix<T, ROW_AMOUNT + R, COLUMN_AMOUNT>();
		for (size_t r = 0; r < ROW_AMOUNT; r++) {
			out.data[r] = this->data[r];
		}
		for (size_t r = 0; r < R; r++) {
			out.data[ROW_AMOUNT + r] = rhs.data[r];
		}

		return (out);
	}

	template<size_t SIZE>
	static Matrix<T, SIZE, SIZE>	identity() {
		Matrix<T, SIZE, SIZE> out;
		for (size_t i = 0; i < SIZE; i++) {
			out[i][i] = 1;
		}
		return (out);
	}

	template<size_t SIZE>
	static Matrix<double, SIZE, SIZE>	diag(double val) {
		auto out = Matrix<double, SIZE, SIZE>();

		for (size_t i = 0; i < SIZE; i++) {
			out[i][i] = val;
		}
		return (out);
	}

	friend std::ostream&	operator<<(std::ostream& o, const Matrix& m) {
		for (size_t row_nb = 0; row_nb < ROW_AMOUNT; row_nb++) {
			o << "[";
			for (size_t col_nb = 0; col_nb < COLUMN_AMOUNT; col_nb++) {
				o << m.data[row_nb][col_nb];
				if (col_nb < COLUMN_AMOUNT - 1) {
					o << ' ';
				}
			}
			o << "]\n";
		}
		return (o);
	}

	Matrix operator+(const Matrix &rhs) const {
		Matrix out(*this);

		for (size_t row = 0; row < ROW_AMOUNT; row++) {
			for (size_t column = 0; column < COLUMN_AMOUNT; column++) {
				out[row][column] += rhs[row][column];
			}
		}
		return (out);
	}

	Matrix operator-(const Matrix &rhs) const {
		Matrix out(*this);

		for (size_t row = 0; row < ROW_AMOUNT; row++) {
			for (size_t column = 0; column < COLUMN_AMOUNT; column++) {
				out[row][column] -= rhs[row][column];
			}
		}
		return (out);
	}

	Matrix pow(long double n) const {
		Matrix out(*this);

		for (size_t row = 0; row < ROW_AMOUNT; row++) {
			for (size_t column = 0; column < COLUMN_AMOUNT; column++) {
				if (out[row][column] != 0)
					out[row][column] = std::pow(out[row][column], n);
			}
		}
		return (out);
	}

	T max() const {
		T val = -INFINITY;

		for (size_t row = 0; row < ROW_AMOUNT; row++) {
			for (size_t column = 0; column < COLUMN_AMOUNT; column++) {
				if (this->data[row][column] > val)
					val = this->data[row][column];
			}
		}
		return (val);
	}

	T min() const {
		T val = INFINITY;

		for (size_t row = 0; row < ROW_AMOUNT; row++) {
			for (size_t column = 0; column < COLUMN_AMOUNT; column++) {
				if (this->data[row][column] < val)
					val = this->data[row][column];
			}
		}
		return (val);
	}
};

template<typename T, size_t ROW_AMOUNT_VEC>
using Vector = Matrix<T, ROW_AMOUNT_VEC, 1>;
using Vector3d = Matrix<double, 3, 1>; // 3 rows * 1 column

#endif //FT_KALMAN_MATRIX_HPP
