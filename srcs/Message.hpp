//
// Created by peer on 23-2-24.
//

#ifndef FT_KALMAN_MESSAGE_HPP
#define FT_KALMAN_MESSAGE_HPP

#include <string>
#include <vector>
#include "Timestamp.hpp"

enum class MessageType {
	MSG_START,
	MSG_END,
	SPEED,
	ACCELERATION,
	DIRECTION,
	TRUE_POSITION,
	POSITION,
};

std::string MessageTypeToString(MessageType type);

class Message {
	MessageType _messageType{};
	Timestamp _timestamp{};
	std::vector<double> data{};

public:
	Message() = default;

	Message(const std::string& msg); // NOLINT(*-explicit-constructor)

	Message(const Message& other) = default;

	Message& operator=(const Message& other) = default;

	~Message() = default;

	[[nodiscard]] MessageType get_message_type() const;

	[[nodiscard]] Timestamp get_timestamp() const;

	[[nodiscard]] const std::vector<double>& get_data() const;

	friend std::ostream& operator<<(std::ostream& o, const Message& msg);

};


#endif //FT_KALMAN_MESSAGE_HPP
