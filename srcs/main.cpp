#include <algorithm>

#include "UdpConnection.hpp"
#include <iostream>
#include <arpa/inet.h>
#include <cstring>
#include <cstdio>
#include "Message.hpp"
#include "Data.hpp"
#include "KalmanFilter.hpp"
#include <fstream>
#include "Arguments.hpp"
#include <iomanip>

using Filter = KalmanFilter<9, 3, 9>;

static bool received_position(const std::vector<Message>& messages) {
	return std::any_of(messages.begin(), messages.end(), [](const Message& msg) {
		return msg.get_message_type() == MessageType::POSITION;
	});
}

int run(const Arguments &args) {
	Data data{};
	Filter filter;
	size_t iterations = 0;

	auto connection = UdpConnection(args.port);
	connection.start();

	std::vector<Message> messages = connection.get_messages();
	for (const auto& msg : messages) {
		data.add_message_information(msg);
	}

	const Filter::StateVector beginState{
		data.get_position(0, 0),
		data.get_velocity(0, 0),
		data.get_acceleration(0, 0),

		data.get_position(0, 1),
		data.get_velocity(0, 1),
		data.get_acceleration(0, 1),

		data.get_position(0, 2),
		data.get_velocity(0, 2),
		data.get_acceleration(0, 2),
	};
	filter.set_state(beginState);
	Timestamp last_timestamp = messages[0].get_timestamp();
	connection.send_data(filter.get_state());

	while (true) {
		messages = connection.get_messages();
		if (messages.empty()) {
			break;
		}
		for (const auto& msg : messages) {
			data.add_message_information(msg);
		}
		const Timestamp msg_timestamp = messages[0].get_timestamp();
		const double time_delta = msg_timestamp.since(last_timestamp);

		filter.predict(time_delta);

		// [PdB / OK] Velocity step. Not used for prod.
		// data.update_velocity(time_delta);
		// filter.update(filter.H_velocity, filter.R_velocity, data.get_velocity());

		filter.update(filter.H_acceleration, filter.R_acceleration, data.get_acceleration());

		if (received_position(messages)) {
			filter.update(filter.H_position, filter.R_position, data.get_position());
		}
		connection.send_data(filter.get_state());

		iterations++;
		last_timestamp = msg_timestamp;
	}
	return EXIT_SUCCESS;
}

int main(const int argc, char **argv) {
	const Arguments args(argc, argv);

	run(args);
}
