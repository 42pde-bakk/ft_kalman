//
// Created by peer on 23-2-24.
//

#include "Data.hpp"
#include "Message.hpp"

void Data::set_position(const std::vector<double>& vec) {
	this->position.set_vector(vec);
}

void Data::set_direction(const std::vector<double>& vec) {
	this->direction.set_vector(vec);
}

void Data::set_acceleration(const std::vector<double>& vec) {
	this->acceleration.set_vector(vec);
}

void Data::set_speed(const double sp) {
	this->speed = sp;
}

double Data::get_speed() const {
	return (this->speed);
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
double Data::get_position(const size_t row_nb, const size_t col_nb) const {
	return (this->position)[row_nb][col_nb];
}

const Matrix<double, 3, 1>& Data::get_direction() const {
	return (this->direction);
}
double Data::get_direction(const size_t row_nb, const size_t col_nb) const {
	return (this->direction[row_nb][col_nb]);
}

const Matrix<double, 3, 1>& Data::get_acceleration() const {
	return (this->acceleration);
}

double Data::get_acceleration(const size_t row_nb, const size_t col_nb) const {
	return (this->acceleration[row_nb][col_nb]);
}

const Vector3d & Data::get_velocity() const {
	return (this->velocity);
}

double Data::get_velocity(const size_t row_nb, const size_t col_nb) const {
	return (this->velocity[row_nb][col_nb]);
}

void	Data::update_velocity(const double timedelta) {
	const double roll = this->direction[0][0];
	const double pitch = this->direction[1][0];
	const double yaw = this->direction[2][0];

	const Matrix<double, 3, 3>	rollMatrix({
		{1, 0, 0},
		{0, std::cos(roll), -std::sin(roll)},
		{0, std::sin(roll), std::cos(roll)}
	});
	const Matrix<double, 3, 3>	pitchMatrix({
		{std::cos(pitch), 0, std::sin(pitch)},
		{0, 1, 0},
		{-std::sin(pitch), 0, std::cos(pitch)}
	});
	const Matrix<double, 3, 3>	yawMatrix({
		{std::cos(yaw), -std::sin(yaw), 0},
		{std::sin(yaw), std::cos(yaw), 0},
		{0, 0, 1}
	});
	const Matrix<double, 3, 3>	r = yawMatrix * pitchMatrix * rollMatrix;
	const auto a = r * this->acceleration;

	this->velocity[0][0] += a[0][0] * timedelta;
	this->velocity[1][0] += a[1][0] * timedelta;
	this->velocity[2][0] += a[2][0] * timedelta;
}

void Data::add_message_information(const Message& msg) {
	switch (msg.get_message_type()) {
		case MessageType::TRUE_POSITION:
			this->set_position(msg.get_data());
			break;
		case MessageType::DIRECTION:
			this->set_direction(msg.get_data());
			break;
		case MessageType::ACCELERATION:
			this->set_acceleration(msg.get_data());
			break;
		case MessageType::SPEED:
			this->set_speed(msg.get_data().front() / 3.6); // [OK] convert to m/s
			this->velocity[0][0] = this->speed;
			break;
		case MessageType::POSITION:
			this->set_position(msg.get_data());
		default:
			break;
	}
}
