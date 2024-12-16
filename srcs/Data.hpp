//
// Created by peer on 23-2-24.
//

#ifndef FT_KALMAN_DATA_HPP
#define FT_KALMAN_DATA_HPP

#include "Matrix.hpp"

class Message;

class Data {
	Vector3d position;
	Vector3d direction;
	Vector3d acceleration;
	Vector3d velocity;
	double speed{0.0};

private:
	void set_position(const std::vector<double>& vec);
	void set_direction(const std::vector<double>& vec);
	void set_acceleration(const std::vector<double>& vec);
	void set_speed(double sp);

public:
	Data() = default;

	Data(const Data&) = default;

	Data& operator=(const Data&) = default;

	~Data() = default;

	[[nodiscard]] double get_speed() const;
	[[nodiscard]] const Vector3d& get_position() const;
	[[nodiscard]] double get_position(size_t row_nb, size_t col_nb) const;
	[[nodiscard]] const Vector3d& get_direction() const;
	[[nodiscard]] double get_direction(size_t row_nb, size_t col_nb) const;
	[[nodiscard]] const Vector3d& get_acceleration() const;
	[[nodiscard]] double get_acceleration(size_t row_nb, size_t col_nb) const;
	[[nodiscard]] const Vector3d& get_velocity() const;
	[[nodiscard]] double get_velocity(size_t row_nb, size_t col_nb) const;
	void update_velocity(double timedelta);

	friend std::ostream& operator<<(std::ostream& o, const Data& d);

	void add_message_information(const Message& msg);
};


#endif //FT_KALMAN_DATA_HPP
