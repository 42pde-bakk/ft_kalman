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
