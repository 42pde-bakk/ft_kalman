//
// Created by peer on 14-3-24.
//

#include <vector>
#include <iostream>
#include "Timestamp.hpp"

std::vector<std::string>	ft_split(const std::string& s, const std::string& delim);

void Timestamp::set(const std::string& s) {
	const auto vec = ft_split(s, ".:");
	for (int i = 0; i < 4; i++) {
		this->time[i] = std::stol(vec[i], nullptr, 10);
	}
}

Timestamp& Timestamp::operator=(const Timestamp&rhs) {
	this->time[0] = rhs.time[0];
	this->time[1] = rhs.time[1];
	this->time[2] = rhs.time[2];
	this->time[3] = rhs.time[3];

	return *this;
}

// size_t Timestamp::to_ms() const {
// 	return this->hours * 3'600'000 + this->minutes * 60'000 + this->seconds * 1000 + this->milliseconds;
// }

std::ostream& operator<<(std::ostream& o, const Timestamp& t) {
	o << "Timestamp(" << t.hours << "h " << t.minutes << "m " << t.seconds << "s " << t.milliseconds << "ms)";
	return (o);
}

// Timestamp operator-(const Timestamp &lhs, const Timestamp &rhs) {
// 	auto n = Timestamp();
//
// 	n.hours = lhs.hours - rhs.hours;
// 	n.minutes = lhs.minutes - rhs.minutes;
// 	n.seconds = lhs.seconds - rhs.seconds;
// 	n.milliseconds = lhs.milliseconds - rhs.milliseconds;
//
// 	std::cout << n.milliseconds << std::endl;
//
// 	return n;
// }
