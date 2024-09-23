#include "UdpConnection.hpp"
#include <iostream>
#include <arpa/inet.h>
#include <cstring>
#include <cstdio>
#include "Message.hpp"
#include "Data.hpp"
#include "KalmanFilter.hpp"
#include <fstream>

#define DEFAULT_PORT 4242

std::ofstream of("messages.txt", std::ios::trunc);
int get_socket_fd() {
	const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (socket_fd == -1) {
		perror("socket");
		exit(EXIT_FAILURE);
	}
	return (socket_fd);
}

void init_serveraddr(struct sockaddr_in* serverAddr) {
	memset(serverAddr, 0, sizeof(*serverAddr));
	serverAddr->sin_family = AF_INET;
	serverAddr->sin_port = htons(DEFAULT_PORT);
	serverAddr->sin_addr.s_addr = inet_addr("127.0.0.1");
}

void perform_handshake(const int socket_fd, struct sockaddr_in* serverAddr) {
	const char* handshake = "READY";
	const ssize_t result = sendto(socket_fd, handshake, strlen(handshake), 0, (struct sockaddr*) serverAddr,
								  sizeof(*serverAddr));

	if (result == -1) {
		perror("sendto");
		exit(EXIT_FAILURE);
	}
}

Message get_message(const int socket_fd, struct sockaddr_in* serverAddr) {
	char buffer[1024];
	socklen_t addr_len = sizeof(*serverAddr);
	const ssize_t res = recvfrom(socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*) serverAddr, &addr_len);

	if (res == -1) {
		perror("recvfrom");
		exit(EXIT_FAILURE);
	}
	if (res == 0) {
		exit(EXIT_SUCCESS);
	}
	buffer[res] = '\0';
	of << buffer << " END MESSAGE\n";
	Message msg(buffer);
	return (msg);
}


Data read_initial_data(std::vector<Message> messages) {
	Data data{};

	for (size_t i = 0; i < 8; i++) {
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
				data.set_speed(msg.get_data().front());
				break;
			default:
				break;
		}
		std::cerr << "parsing [" << i << "] " << msg << "\n";
	}
	std::cerr << "\n\n\n\nDone reading initial data.\n";
	std::cerr << data << "\n";
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

int main() {
	auto connection = UdpConnection(4242);

	connection.start();

	auto initial_data = read_initial_data(connection.get_messages());

	KalmanFilter	filter(initial_data);

	of << " DONE PARSING INITIAL DATA\n";
	of << '\n' << initial_data << '\n';
	std::cerr << '\n' << initial_data << '\n';
	std::cerr << "Lets start the messaging loop!\n";

	auto last_timestamp_at = Timestamp();
	auto start_timestamp = std::chrono::system_clock::now();

	auto state_transition_matrix = filter.get_state_transition_matrix(1.0);
	std::cerr << state_transition_matrix << "\n";

	auto delta = 0;
	size_t iterations = 0;
  
	while (true) {
		const auto mat = filter.predict(delta, initial_data.get_acceleration());

		std::cout << mat << std::endl;

		connection.send_data(mat);

    
		auto messages = connection.get_messages();
		if (messages.size() == 0) {
			break;
		}

		auto msg_timestamp = messages[0].get_timestamp();

		delta = (msg_timestamp - last_timestamp_at).to_ms();

		for (size_t i = 0; i < messages.size(); i++)
		{
			// std::cerr << "[" << i << "] " << messages[i] << "\n";
		}

		iterations++;
		last_timestamp_at = msg_timestamp;
	}

	auto end_timestamp = std::chrono::system_clock::now();

	std::cout << "Survived for " << 
		(end_timestamp - start_timestamp).count() / 1'000'000'000u - 1 << " seconds, iterations: " << iterations << std::endl;

	std::cout << "DONE!" << std::endl;

	return EXIT_SUCCESS;
}
