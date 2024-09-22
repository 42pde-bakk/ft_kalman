//
// Created by peer on 14-3-24.
//

#ifndef FT_KALMAN_TIMESTAMP_HPP
#define FT_KALMAN_TIMESTAMP_HPP
#include <string>
#include <ostream>

typedef unsigned int t_time;

class Timestamp {
	union {
		t_time time[4];
		struct {
			t_time	hours,
			                minutes,
							seconds,
							milliseconds;
		};
	};

public:
	Timestamp() = default;
	Timestamp(const Timestamp& rhs) = default;
	~Timestamp() = default;
	void set(const std::string& s);

	size_t to_ms();

	[[nodiscard]] double since(const Timestamp& old) const {
		return 3600 * (this->hours - old.hours) + 60 * (this->minutes - old.minutes) + (this->seconds - old.seconds) + (double)(this->milliseconds - old.milliseconds) / 1000;
	}

	friend std::ostream& operator<<(std::ostream& o, const Timestamp& t);
	friend Timestamp& operator-(Timestamp& lhs, const Timestamp& rhs);
};


#endif //FT_KALMAN_TIMESTAMP_HPP
