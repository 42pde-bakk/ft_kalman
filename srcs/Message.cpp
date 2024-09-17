//
// Created by peer on 23-2-24.
//

#include <iostream>
#include <sstream>
#include <cstring>
#include "Message.hpp"

MessageType parseMessageTypeFromString(const std::string& str) {
	if (str == "MSG_START") {
		return MessageType::MSG_START;
	} else if (str == "MSG_END") {
		return MessageType::MSG_END;
	} else if (str == "SPEED") {
		return MessageType::SPEED;
	} else if (str == "ACCELERATION") {
		return MessageType::ACCELERATION;
	} else if (str == "DIRECTION") {
		return MessageType::DIRECTION;
	} else if (str == "TRUE POSITION") {
		return MessageType::TRUE_POSITION;
	}
	return MessageType::MSG_END;
}

Message::Message(const std::string& msg) {
	if (msg == "MSG_START") {
		this->_messageType = MessageType::MSG_START;
	} else if (msg == "MSG_END") {
		this->_messageType = MessageType::MSG_END;
	} else if (msg.find("Trajectory Generat") != std::string::npos) {
	} else {
		size_t fpos = msg.find('['),
				lpos = msg.find(']'),
				newlinepos = msg.find('\n');
		this->_timestamp.set(msg.substr(fpos + 1, lpos - fpos - 1));
		const auto type = msg.substr(lpos + 1, newlinepos - lpos - 1);
		this->_messageType = parseMessageTypeFromString(type);
//		std::cerr << "timestamp: {" << _timestamp << "}, msgtype = " << type << "!\n";
		const char* start = msg.c_str() + newlinepos + 1;
		char* pend;
		do {
			double d = std::strtod(start, &pend);
			this->data.push_back(d);
			start = pend + 1;
		} while (strlen(pend) > 1);

//		for (const auto& d : this->data) {
//			printf("\td = %f\n", d);
//		}
	}
}

MessageType Message::get_message_type() const {
	return (this->_messageType);
}

Timestamp Message::get_timestamp() const {
	return (this->_timestamp);
}

const std::vector<double>& Message::get_data() const {
	return (this->data);
}

std::ostream& operator<<(std::ostream& o, const Message& msg) {
	o << "Message:\n";
	o << "\tType: " << MessageTypeToString(msg._messageType) << "\n";
	o << "\tTimestamp: " << msg._timestamp << "\n";
	o << "\tdata: [ ";
	for (const auto& value: msg.data) {
		o << value << " ";
	}
	o << "]\n";

	return (o);
}

std::string MessageTypeToString(MessageType type) {
	static const std::string strings[] = {
			"MSG_START",
			"MSG_END",
			"SPEED",
			"ACCELERATION",
			"DIRECTION",
			"TRUE_POSITION"
	};

	return (strings[(int) type]);
}
