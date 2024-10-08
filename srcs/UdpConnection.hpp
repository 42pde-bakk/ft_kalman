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
#include <sys/epoll.h>

#define MAX_EPOLL_EVENTS 8

class UdpConnection
{
private:
	unsigned short port;
	int socket_fd;
	int epoll_fd;
	struct sockaddr_in serverAddr = {};
	struct epoll_event events[8];

	int get_socket_fd() {
		const int socket_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);

		if (socket_fd == -1) {
			perror("socket");
			exit(EXIT_FAILURE);
		}
		return (socket_fd);
	}

	int get_epoll_fd(int socket_fd) {
		int epoll_fd = epoll_create(1);

		struct epoll_event event;
		event.events = EPOLLIN; // Can append "|EPOLLOUT" for write events as well
		event.data.fd = socket_fd;
		epoll_ctl(epoll_fd, EPOLL_CTL_ADD, socket_fd, &event);

		return epoll_fd;
	}

	void init_serveraddr() {
		memset(&this->serverAddr, 0, sizeof(this->serverAddr));
		this->serverAddr.sin_family = AF_INET;
		this->serverAddr.sin_port = htons(this->port);
		this->serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	}

	void perform_handshake() {
		std::cout << "Sending handshake." << std::endl;

		const char* handshake = "READY";

		while (1 == 1)
		{
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

			int epoll_res = epoll_wait(this->epoll_fd, this->events, MAX_EPOLL_EVENTS, 2000);
			if (epoll_res == -1) {
				perror("epoll");
			}

			if (epoll_res == 0) {
				std::cout << "No handshake received yet.." << std::endl;
			}
			else {
				break;
			}
		}
	}

public:
	UdpConnection(const UdpConnection& rhs) = default;


	UdpConnection(unsigned short port) {
		this->port = port;
		this->socket_fd = this->get_socket_fd();
		this->epoll_fd = this->get_epoll_fd(this->socket_fd);
		this->init_serveraddr();
	}

	void start() {
		this->perform_handshake();
	}

	std::vector<Message> get_messages() {
		auto messages = std::vector<Message>();

		socklen_t addr_len = sizeof(this->serverAddr);

		auto idx = 0u;

		while (true)
		{
			auto buffer = std::string();
			buffer.resize(1024);

			// std::cout << "START READING" << std::endl;

			int epoll_res = epoll_wait(this->epoll_fd, this->events, MAX_EPOLL_EVENTS, 1000);
			if (epoll_res == -1) {
				perror("epoll");
			}

			if (epoll_res == 0 && idx == 0) {
				std::cerr << "error: ft_kalman: timeout socket_fd" << std::endl;
				return std::vector<Message>();
			}

			if (epoll_res == 0) {
				break;
			}

			const ssize_t res = recvfrom(
				this->socket_fd,
				&buffer[0],
				buffer.length(),
				0,
				(struct sockaddr*) &this->serverAddr,
				&addr_len
			);

			if (res == -1) {
				perror("recvfrom");
				exit(EXIT_FAILURE);
			}
			if (res == 0) {
				exit(EXIT_SUCCESS);
			}

			// std::cout << "buff: " << buffer << std::endl;

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