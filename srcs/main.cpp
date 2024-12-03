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

std::ofstream messagesFile("messages.txt", std::ios::trunc);
using Filter = KalmanFilter<9, 3, 9>;


void send_data(const int socket_fd, sockaddr_in* serverAddr, const Matrix<double, 3, 1>& matrix) {
	std::string state_str;
	for (size_t row = 0; row < 3; row++) {
		state_str += std::to_string(matrix[row][0]);
		if (row != 2) {
			state_str += ' ';
		}
	}

	const ssize_t result = sendto(socket_fd, state_str.c_str(), state_str.length(), 0, (struct sockaddr*) serverAddr,
								  sizeof(*serverAddr));
	if (result == -1) {
		perror("sendto");
		exit(EXIT_FAILURE);
	}
	if (result == 0) {
		exit(EXIT_SUCCESS);
	}
}

static bool received_position(const std::vector<Message>& messages) {
	return std::any_of(messages.begin(), messages.end(), [](const Message& msg) {
		return msg.get_message_type() == MessageType::POSITION;
	});
}

int run(const Arguments &args) {
	std::ofstream statesFile("states.txt", std::ios::trunc);
	Data data{};
	KalmanFilter<9, 3, 9>	filter;
	size_t iterations = 0;

	auto connection = UdpConnection(args.port);
	connection.start();

	std::vector<Message>messages = connection.get_messages();
	std::cerr << "Received " << messages.size() << " initial messages\n";
	for (const auto& msg : messages) {
		messagesFile << msg << '\n';
		data.add_message_information(msg);
	}

	Vector3d velocity = data.calculate_velocity();
	std::cerr << "velocity: " << velocity << "\n";
	const Filter::StateVector beginState{
		data.get_position()[0][0],
		data.get_speed(),
		data.get_acceleration(0, 0),

		data.get_position()[0][1],
		velocity[0][1],
		data.get_acceleration(0, 1),

		data.get_position()[0][2],
		velocity[0][2],
		data.get_acceleration(0, 2),
	};
	std::cout << std::setprecision(12) << "State:\n" << beginState << std::endl;
	filter.set_state(beginState);
	Timestamp last_timestamp = messages[0].get_timestamp();
	connection.send_data(filter.get_state());

	while (true) {
		messages = connection.get_messages();
		// std::cerr << "Received " << messages.size() << " messages\n";
		if (messages.empty()) {
			break;
		}
		for (const auto& msg : messages) {
			data.add_message_information(msg);
			messagesFile << msg << '\n';
		}
		const Timestamp msg_timestamp = messages[0].get_timestamp();
		const double timedelta = msg_timestamp.since(last_timestamp);
		// std::cerr << "timedelta = " << timedelta << "\n";

		// data.update_velocity(timedelta);
		filter.predict(timedelta); // TODO: do I need to send control data?

		filter.update(filter.H_acceleration, filter.R_acceleration, data.get_acceleration());
		if (received_position(messages)) {
			// std::cerr << "We have a position!\n";
			filter.update(filter.H_position, filter.R_position, data.get_position());
			statesFile << "Update: ";
		} else {
			statesFile << "Predict: ";
		}
		connection.send_data(filter.get_state());

		filter.printState(statesFile);
		iterations++;
		last_timestamp = msg_timestamp;
		// std::cerr << "End of loop, state:\n" << filter.get_state() << "\n";
	}

	// std::cout << "Survived " << iterations << " iterations!" << std::endl;

	return EXIT_SUCCESS;
}

int main(const int argc, char **argv) {
	const Arguments args(argc, argv);

	run(args);
}
