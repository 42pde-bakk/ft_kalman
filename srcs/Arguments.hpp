#ifndef ARUGMENTS_HPP
#define ARUGMENTS_HPP

#include <assert.h>
#include <string>
#include <iostream>

enum DebugLevel {
	None,
	Enabled,
	Verbose
};

class Arguments
{
private:
	char *get_next_argument(size_t &idx, size_t argc, char **argv) {
		if (idx >= argc) {
			std::cerr << "ft_kalman: error: " << argv[idx] << " requires a argument" << std::endl;
			exit(1);
		}
		return argv[++idx];
	}
public:
	unsigned short port = 4242;
	size_t iter_limit = (size_t)-1;
	DebugLevel debug = DebugLevel::None;

	Arguments(int argc, char **argv) {
		for (size_t i = 0; i < (size_t)argc; i++)
		{
			auto item = std::string(argv[i]);

			if (item == "-p" || item == "--port") {
				this->port = std::stoi(get_next_argument(i, argc, argv));
				assert(this->port > 0 && this->port < 16384);
			}

			else if (item == "-i" || item == "--iterations") {
				this->iter_limit = std::stoi(get_next_argument(i, argc, argv));
				assert(this->iter_limit > 0);
			}
			else if (item == "-v") {
				this->debug = DebugLevel::Enabled;
			} else if (item == "-vv") {
				this->debug = DebugLevel::Verbose;
			} else if (item == "--help") {
				std::cout << "usage: ft_kalman [ARGS]" << '\n';
				std::cout << "\t-p       port used for connecting to the stream" << '\n';
				std::cout << "\t-i       max iterations done by the program" << '\n';
				std::cout << "\t-v/vv    debug level" << '\n';
				exit(0);
			}
		}
		
	}
	Arguments(const Arguments&) = default;

	Arguments& operator=(const Arguments&) = default;

	~Arguments() = default;
};

#endif