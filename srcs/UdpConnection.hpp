#ifndef UDP_CONNECTION_HPP
#define UDP_CONNECTION_HPP

#include <iostream>
#include <arpa/inet.h>
#include <cstring>
#include <cstdio>
#include "Message.hpp"
#include "Data.hpp"
#include "KalmanFilter.hpp"
#include <fstream>
#include <vector>
#include "Message.hpp"
#include <chrono>
#include <thread>

class UdpConnection
{
private:
	unsigned short port;
	int socket_fd;
	struct sockaddr_in serverAddr = {};

	int get_socket_fd() {
		const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);

		if (socket_fd == -1) {
			perror("socket");
			exit(EXIT_FAILURE);
		}
		return (socket_fd);
	}

	void init_serveraddr() {
		memset(&this->serverAddr, 0, sizeof(this->serverAddr));
		this->serverAddr.sin_family = AF_INET;
		this->serverAddr.sin_port = htons(this->port);
		this->serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	}

	void perform_handshake() {
		const char* handshake = "READY";
		const ssize_t result = sendto(
			this->socket_fd,
			handshake,
			strlen(handshake),
			0,
			(struct sockaddr*) &this->serverAddr,
			sizeof(this->serverAddr)
		);

		if (result == -1) {
			perror("sendto");
			exit(EXIT_FAILURE);
		}
	}

public:
	UdpConnection(const UdpConnection& rhs) = default;


	UdpConnection(unsigned short port) {
		this->port = port;
		this->socket_fd = this->get_socket_fd();
		this->init_serveraddr();
	}

	void start() {
		this->perform_handshake();
	}

	std::vector<Message> get_messages() {
		auto messages = std::vector<Message>();

		socklen_t addr_len = sizeof(this->serverAddr);

		auto idx = 0ull;

		while (true)
		{
			auto buffer = std::string();
			buffer.resize(1024);

			std::cout << "START READING" << std::endl;

			const ssize_t res = recvfrom(
				this->socket_fd,
				&buffer[0],
				buffer.length(),
				0,
				(struct sockaddr*) &this->serverAddr,
				&addr_len
			);

			std::cout << "buff: " << buffer << std::endl;

			if (res == -1) {
				perror("recvfrom");
				exit(EXIT_FAILURE);
			}
			if (res == 0) {
				exit(EXIT_SUCCESS);
			}

			if (buffer.starts_with("Trajectory") || buffer.starts_with("Sending")) {
				continue;
			}
			else if (idx == 0 && !buffer.starts_with("MSG_START")) {
				std::cerr << "Error: no start packet" << std::endl;
				exit(1);
			} else if (buffer.starts_with("MSG_START")) {
				idx++;
				continue;
			} else if (buffer.starts_with("MSG_END")) {
				break;
			}

			messages.push_back(Message(buffer));

			idx++;
		}
		
		return messages;
	}

	void send_data(const Matrix<double, 3, 1>& matrix) {
		std::string state_str;
		for (size_t row = 0; row < 3; row++) {
			state_str += std::to_string(matrix[row][0]);
			if (row != 2) {
				state_str += ' ';
			}
		}

		const ssize_t result = sendto(
			this->socket_fd, 
			state_str.c_str(), 
			state_str.length(), 
			0, 
			(struct sockaddr*) &serverAddr,
			sizeof(this->serverAddr)
		);

		std::cerr << "sent state_str, result = " << result << "\n";
		if (result == -1) {
			perror("sendto");
			exit(EXIT_FAILURE);
		} else if (result == 0) {
			exit(EXIT_SUCCESS);
		}
	}

	~UdpConnection() = default;
};


#endif