//
// Created by peer on 14-3-24.
//

#include <vector>
#include "Timestamp.hpp"

std::vector<std::string>	ft_split(const std::string& s, const std::string& delim);

void Timestamp::set(const std::string& str) {
	const auto vec = ft_split(str, ".:");
	for (int i = 0; i < 4; i++) {
		this->time[i] = std::stoul(vec[i], nullptr, 10);
	}
}

size_t Timestamp::to_ms() {
	return this->hours * 3'600'000 + this->minutes * 60'000 + this->seconds * 1000 + this->milliseconds;
}

std::ostream& operator<<(std::ostream& o, const Timestamp& t) {
	o << "Timestamp " << t.hours << ": " << t.minutes << ": " << t.seconds << "." << t.milliseconds;
	return (o);
}

Timestamp &operator-(Timestamp &lhs, const Timestamp &rhs) {
	lhs.hours -= rhs.hours;
	lhs.minutes -= rhs.minutes;
	lhs.seconds -= rhs.seconds;
	lhs.milliseconds -= rhs.milliseconds;

	return (lhs);
}
