//
// Created by peer on 23-2-24.
//

#ifndef FT_KALMAN_MESSAGE_HPP
#define FT_KALMAN_MESSAGE_HPP

#include <string>
#include <vector>

enum class MessageType {
	MSG_START,
	MSG_END,
	SPEED,
	ACCELERATION,
	DIRECTION,
	TRUE_POSITION,
};


class Message {
	MessageType _messageType{};
	std::string	_timestamp{};
	std::vector<double>	data{};

public:
	Message() = default;
	Message(const std::string& msg);

	Message(const Message& other) = default;
	Message& operator=(const Message& other) = default;
	~Message() = default;

};


#endif //FT_KALMAN_MESSAGE_HPP
