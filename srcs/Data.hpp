//
// Created by peer on 23-2-24.
//

#ifndef FT_KALMAN_DATA_HPP
#define FT_KALMAN_DATA_HPP

#include "Matrix.hpp"

class Data {
	Vector3d position;
	Vector3d direction;
	Vector3d acceleration;
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

	[[nodiscard]] const Matrix<double, 3, 1>& get_position() const;

	friend std::ostream& operator<<(std::ostream& o, const Data& d);
};


#endif //FT_KALMAN_DATA_HPP
