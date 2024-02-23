//
// Created by peer on 23-2-24.
//

#include <iostream>
#include <sstream>
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


std::vector<std::string>	split(const std::string& s, const std::string& delim) {
	size_t start, end = 0;
	std::vector<std::string> vec;

	while (end != std::string::npos) {
		start = s.find_first_not_of(delim, end);
		end = s.find_first_of(delim, start);
		if (end != std::string::npos || start != std::string::npos)
			vec.push_back(s.substr(start, end - start));
	}
	return vec;
}

Message::Message(const std::string& msg) {
	if (msg == "MSG_START") {
		this->_messageType = MessageType::MSG_START;
	} else if (msg == "MSG_END") {
		this->_messageType = MessageType::MSG_END;
	} else if (msg.find("Trajectory Generat") != std::string::npos) {
	} else {
		size_t	fpos = msg.find('['),
				lpos = msg.find(']');
		this->_timestamp = msg.substr(fpos + 1, lpos - fpos);
		this->_messageType = parseMessageTypeFromString(msg.substr(fpos + 1));
		std::cerr << "timestamp: " << _timestamp << ", msgtype = " << msg.substr(lpos + 1) << "\n";
		const char* start = msg.c_str();
		char* pend;
		do {
			double d = std::strtod(start, &pend);
			this->data.push_back(d);
			start = pend;
		} while (pend != nullptr);

		for (const auto& d : this->data) {
			printf("d = %f", d);
		}
	}
}
