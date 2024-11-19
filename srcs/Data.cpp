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

double Data::get_acceleration(const size_t row_nb, const size_t col_nb) const {
	return (this->acceleration[row_nb][col_nb]);
}

Vector3d Data::calculate_velocity() const {
	const double roll = this->direction[0][0];
	const double pitch = this->direction[1][0];
	const double yaw = this->direction[2][0];

	const Vector3d dir({direction[0][0], direction[1][0], direction[2][0]});

	// First roll:
	const Matrix<double,3, 3> rollMatrix({
		{1, 0, 0},
		{0, std::cos(roll), -std::sin(roll)},
		{0, std::sin(roll), std::cos(roll)}
	});

	const Matrix<double,3, 3> pitchMatrix({
		{std::cos(pitch), 0, -std::sin(pitch)},
		{0, 1, 0},
		{std::sin(pitch), 0, std::cos(pitch)}
	});

	const Matrix<double,3, 3> yawMatrix({
		{std::cos(yaw), -std::sin(yaw), 0},
		{std::sin(yaw), std::cos(yaw), 0},
		{0, 0, 1}
	});

	return (rollMatrix * pitchMatrix * yawMatrix) * dir;


	Vector3d velocity({
		this->speed * std::cos(pitch) * std::cos(yaw),
		this->speed * std::cos(pitch) * std::sin(yaw),
		-this->speed * std::sin(pitch)
	});

	return velocity;
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

	// const double	a_x = r[0][0] * this->acceleration[0][0] + r[0][1] * this->acceleration[1][0] + r[0][2] * this->acceleration[2][0];
	// const double	a_y = r[1][0] * this->acceleration[0][0] + r[1][1] * this->acceleration[1][0] + r[1][2] * this->acceleration[2][0];
	// const double	a_z = r[2][0] * this->acceleration[0][0] + r[2][1] * this->acceleration[1][0] + r[2][2] * this->acceleration[2][0];
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
			break;
		case MessageType::POSITION:
			this->set_position(msg.get_data());
		default:
			break;
	}
}
