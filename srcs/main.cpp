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

std::ofstream of("messages.txt", std::ios::trunc);

Data read_initial_data(std::vector<Message> messages) {
	Data data{};

	for (size_t i = 0; i < messages.size(); i++) {
		auto msg = messages[i];

		switch (msg.get_message_type()) {
			case MessageType::TRUE_POSITION:
				data.set_position(msg.get_data());
				break;
			case MessageType::DIRECTION:
				data.set_direction(msg.get_data());
				break;
			case MessageType::ACCELERATION:
				data.set_acceleration(msg.get_data());
				break;
			case MessageType::SPEED:
				data.set_speed(msg.get_data().front() / 3.6); // [OK] convert to m/s
				break;
			default:
				break;
		}
	}
	return (data);
}

void send_data(const int socket_fd, struct sockaddr_in* serverAddr, const Matrix<double, 3, 1>& matrix) {
	std::string state_str;
	for (size_t row = 0; row < 3; row++) {
		state_str += std::to_string(matrix[row][0]);
		if (row != 2) {
			state_str += ' ';
		}
	}

	const ssize_t result = sendto(socket_fd, state_str.c_str(), state_str.length(), 0, (struct sockaddr*) serverAddr,
								  sizeof(*serverAddr));
	std::cerr << "sent state_str, result = " << result << "\n";
	if (result == -1) {
		perror("sendto");
		exit(EXIT_FAILURE);
	} else if (result == 0) {
		exit(EXIT_SUCCESS);
	}
}

int run(Arguments &args) {
	auto connection = UdpConnection(args.port);

	connection.start();

	KalmanFilter<9, 9, 9>	filter;

	auto last_timestamp_at = Timestamp();
	auto start_timestamp = std::chrono::system_clock::now();

	auto delta = 0;
	size_t iterations = 0;
  
	while (true) {
		auto messages = connection.get_messages();
		if (messages.size() == 0) {
			break;
		}

		auto data = read_initial_data(messages);


		if (iterations == 0) {
			auto velocity = data.calculate_velocity();

			auto state = std::array<double, 9>({
				data.get_position()[0][0],
				data.get_position()[0][1],
				data.get_position()[0][2],
				velocity[0][0],
				velocity[0][1],
				velocity[0][2],
				data.get_acceleration(0, 0),
				data.get_acceleration(0, 1),
				data.get_acceleration(0, 2),
			});
	
			filter.set_state(state);

		} else {
			data.set_speed(filter.get_current_speed());
		}

		auto velocity = data.calculate_velocity();

		auto state = std::array<double, 9>({
			0,
			0,
			0,
			velocity[0][0],
			velocity[0][1],
			velocity[0][2],
			data.get_acceleration(0, 0),
			data.get_acceleration(0, 1),
			data.get_acceleration(0, 2),
		});

		auto input = Matrix<double, 9, 1>(state);


		auto msg_timestamp = messages[0].get_timestamp();

		delta = (msg_timestamp - last_timestamp_at).to_ms();

		const auto mat = filter.predict(delta, input);

		std::cout << mat << std::endl;

		connection.send_data(mat);

		for (size_t i = 0; i < messages.size(); i++)
		{
			std::cout << "[" << i << "] " << messages[i] << "\n";
		}

		iterations++;
		last_timestamp_at = msg_timestamp;

		if (args.iter_limit != std::string::npos && iterations >= args.iter_limit) {
			std::cerr << "Iter limit reached, stopping." << std::endl;
			break;
		}

	}

	auto end_timestamp = std::chrono::system_clock::now();

	std::cout << "Survived for " << 
		(end_timestamp - start_timestamp).count() / 1'000'000'000u - 1 << " seconds, iterations: " << iterations << std::endl;

	std::cout << "DONE!" << std::endl;

	return EXIT_SUCCESS;
}

int main(int argc, char **argv) {
	Arguments args(argc, argv);

	run(args);
}
