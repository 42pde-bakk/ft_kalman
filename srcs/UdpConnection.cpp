//
// Created by peer on 11/19/24.
//
#include "UdpConnection.hpp"

#include <cstring>
#include <iostream>

int UdpConnection::get_socket_fd() {
    const int socket_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);

    if (socket_fd == -1) {
        perror("socket");
        exit(EXIT_FAILURE);
    }
    return (socket_fd);
}

int UdpConnection::get_epoll_fd(const int socket_fd) {
    const int epoll_fd = epoll_create(1);

    epoll_event event{};
    event.events = EPOLLIN; // Can append "|EPOLLOUT" for write events as well
    event.data.fd = socket_fd;
    epoll_ctl(epoll_fd, EPOLL_CTL_ADD, socket_fd, &event);

    return epoll_fd;
}

void UdpConnection::init_serveraddr() {
    memset(&this->serverAddr, 0, sizeof(this->serverAddr));
    this->serverAddr.sin_family = AF_INET;
    this->serverAddr.sin_port = htons(this->port);
    this->serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
}

void UdpConnection::perform_handshake() {
    std::cout << "Sending handshake." << std::endl;

    while (true)
    {
        const auto handshake = "READY";
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

        const int epoll_res = epoll_wait(this->epoll_fd, this->events, MAX_EPOLL_EVENTS, 2000);
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

UdpConnection::UdpConnection(const unsigned short port) {
    this->port = port;
    this->socket_fd = UdpConnection::get_socket_fd();
    this->epoll_fd = UdpConnection::get_epoll_fd(this->socket_fd);
    this->init_serveraddr();
}

void UdpConnection::start() {
    this->perform_handshake();
}

std::vector<Message> UdpConnection::get_messages() {
    auto messages = std::vector<Message>();

    socklen_t addr_len = sizeof(this->serverAddr);

    auto idx = 0u;

    while (true)
    {
        auto buffer = std::string();
        buffer.resize(1024);

        // std::cout << "START READING" << std::endl;

        const int epoll_res = epoll_wait(this->epoll_fd, this->events, MAX_EPOLL_EVENTS, 1000);
        if (epoll_res == -1) {
            perror("epoll");
        }

        if (epoll_res == 0 && idx == 0) {
            std::cerr << "error: ft_kalman: timeout socket_fd" << std::endl;
            return {};
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
        if (idx == 0 && !buffer.starts_with("MSG_START")) {
            std::cerr << "Error: no start packet" << std::endl;
            exit(1);
        }
        if (buffer.starts_with("MSG_START")) {
            idx++;
            continue;
        }
        if (buffer.starts_with("MSG_END")) {
            break;
        }

        messages.emplace_back(buffer);

        idx++;
    }

    return messages;
}

void UdpConnection::send_data(const Matrix<double, 9, 1> &matrix) {
    std::string state_str;
    for (size_t row = 0; row < 9; row += 3) {
        state_str += std::to_string(matrix[row][0]);
        if (row != 6) {
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

    if (result == -1) {
        perror("sendto");
        exit(EXIT_FAILURE);
    }
    if (result == 0) {
        exit(EXIT_SUCCESS);
    }
}
