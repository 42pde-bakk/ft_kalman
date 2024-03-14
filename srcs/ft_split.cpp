//
// Created by peer on 14-3-24.
//

#include <vector>
#include <string>

std::vector<std::string>	ft_split(const std::string& s, const std::string& delim) {
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
