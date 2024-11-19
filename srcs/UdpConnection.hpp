#ifndef UDP_CONNECTION_HPP
#define UDP_CONNECTION_HPP

#include <arpa/inet.h>
#include "Message.hpp"
#include "KalmanFilter.hpp"
#include <vector>
#include <sys/epoll.h>

#define MAX_EPOLL_EVENTS 8

class UdpConnection {
	unsigned short port;
	int socket_fd;
	int epoll_fd;
	sockaddr_in serverAddr = {};
	epoll_event events[8]{};

	static int get_socket_fd();

	static int get_epoll_fd(int socket_fd);

	void init_serveraddr();

	void perform_handshake();

public:
	UdpConnection(const UdpConnection& rhs) = default;
	UdpConnection(unsigned short port);
	~UdpConnection() = default;

	void start();

	std::vector<Message> get_messages();

	void send_data(const Matrix<double, 9, 1>& matrix);
};


#endif