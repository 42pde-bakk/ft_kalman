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

std::ostream& operator<<(std::ostream& o, const Timestamp& t) {
	o << "Timestamp " << t.hours << ": " << t.minutes << ": " << t.seconds << "." << t.milliseconds;
	return (o);
}
