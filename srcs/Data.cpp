//
// Created by peer on 23-2-24.
//

#include "Data.hpp"

void Data::set_position(const std::vector<double>& vec) {
	this->position.set_vector(vec);
}

void Data::set_direction(const std::vector<double>& vec) {
	this->direction.set_vector(vec);
}

void Data::set_acceleration(const std::vector<double>& vec) {
	this->acceleration.set_vector(vec);
}

void Data::set_speed(double sp) {
	this->speed = sp;
}

std::ostream& operator<<(std::ostream& o, const Data& d) {
	o << "Data:\n";
	o << "\tposition:\n" << d.position << "\n";
	o << "\tdirection:\n" << d.direction << "\n";
	o << "\tacceleration:\n" << d.acceleration << "\n";
	o << "\tspeed: " << d.speed << "\n";
	return (o);
}

const Matrix<double, 3, 1>& Data::get_position() const {
	return (this->position);
}

const Matrix<double, 3, 1>& Data::get_direction() const {
	return (this->direction);
}

const Matrix<double, 3, 1>& Data::get_acceleration() const {
	return (this->acceleration);
}

double Data::get_acceleration(size_t row_nb, size_t col_nb) const {
	return (this->acceleration[row_nb][col_nb]);
}

Vector3d Data::calculate_velocity() const {
	Vector3d velocity(std::array<double, 3>({0, 0, 0}));
	double roll = this->direction[0][0];
	double pitch = this->direction[1][0];
	double yaw = this->direction[2][0];

	Matrix<double, 3, 3>	r_x({
		std::array<double, 3>({1, 0, 0}),
		std::array<double, 3>({0, std::cos(roll), -1 * std::sin(roll)}),
		std::array<double, 3>({0, std::sin(roll), std::cos(roll)})
	});
	Matrix<double, 3, 3>	r_y({
		std::array<double, 3>({std::cos(pitch), 0, std::sin(pitch)}),
		std::array<double, 3>({0, 1, 0}),
		std::array<double, 3>({-1 * std::sin(pitch), 0, std::cos(pitch)})
	});
	Matrix<double, 3, 3>	r_z({
		std::array<double, 3>({std::cos(yaw), -1 * std::sin(yaw), 0}),
		std::array<double, 3>({std::sin(yaw), std::cos(yaw), 0}),
		std::array<double, 3>({0, 0, 1})
	});
	Matrix<double, 3, 3>	r = r_z * r_y * r_x;

	double	a_x = r[0][0] * this->get_acceleration(0, 0) + r[0][1] * this->get_acceleration(1, 0) + r[0][2] * this->get_acceleration(2, 0);
	double	a_y = r[1][0] * this->get_acceleration(0, 0) + r[1][1] * this->get_acceleration(1, 0) + r[1][2] * this->get_acceleration(2, 0);
	double	a_z = r[2][0] * this->get_acceleration(0, 0) + r[2][1] * this->get_acceleration(1, 0) + r[2][2] * this->get_acceleration(2, 0);

	velocity[0][0] += a_x * DT;
	velocity[1][0] += a_y * DT;
	velocity[2][0] += a_z * DT;

	return velocity;
}
