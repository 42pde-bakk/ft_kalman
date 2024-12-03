//
// Created by peer on 23-2-24.
//

#ifndef FT_KALMAN_DATA_HPP
#define FT_KALMAN_DATA_HPP

#include "Matrix.hpp"
// #define DT 0.01

class Message;

class Data {
	Vector3d position;
	Vector3d direction;
	Vector3d acceleration;
	Vector3d velocity;
	double speed;

public:
	Data() = default;

	Data(const Data&) = default;

	Data& operator=(const Data&) = default;

	~Data() = default;

	void set_position(const std::vector<double>& vec);

	void set_direction(const std::vector<double>& vec);

	void set_acceleration(const std::vector<double>& vec);

	void set_speed(double sp);

	[[nodiscard]] double get_speed() const;
	[[nodiscard]] const Matrix<double, 3, 1>& get_position() const;
	[[nodiscard]] const Matrix<double, 3, 1>& get_direction() const;
	[[nodiscard]] const Matrix<double, 3, 1>& get_acceleration() const;
	[[nodiscard]] double get_acceleration(size_t row_nb, size_t col_nb) const;

	[[nodiscard]] Vector3d calculate_velocity() const;
	void update_velocity(double timedelta);

	friend std::ostream& operator<<(std::ostream& o, const Data& d);

	void add_message_information(const Message& msg);
};


#endif //FT_KALMAN_DATA_HPP
