#include <iostream>
#include <arpa/inet.h>
#include <cstring>
#include <cstdio>
#include "Message.hpp"
#define DEFAULT_PORT 4242


int main() {
	char buffer[1024];
	const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (socket_fd == -1) {
		perror("socket");
		exit(EXIT_FAILURE);
	}

	struct sockaddr_in serverAddr{};
	memset(&serverAddr, 0, sizeof(serverAddr));
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(DEFAULT_PORT);
	serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");

	// Send handshake signal
	const char* handshake = "READY";
	if (sendto(socket_fd, handshake, strlen(handshake), 0, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
		perror("sendto");
		exit(EXIT_FAILURE);
	}
	printf("after sendto\n");

	socklen_t addr_len = sizeof(serverAddr);
	for (size_t i = 0; i < 10; i++) {
		const ssize_t res = recvfrom(socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr *)&serverAddr, &addr_len);
		printf("received %zu bytes\n", res);
		buffer[res] = '\0';
		printf("msg = '%s'\n", buffer);
		Message msg(buffer);
	}

	return EXIT_SUCCESS;
}
