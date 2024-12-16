//
// Created by peer on 14-3-24.
//

#ifndef FT_KALMAN_TIMESTAMP_HPP
#define FT_KALMAN_TIMESTAMP_HPP
#include <iostream>
#include <string>

typedef long int t_time;

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

	Timestamp& operator=(const Timestamp &rhs);

	void set(const std::string& s);

	// [[nodiscard]] size_t to_ms() const;

	[[nodiscard]] double since(const Timestamp& old) const {
		return 3600 * (this->hours - old.hours) + 60 * (this->minutes - old.minutes) + (this->seconds - old.seconds) + static_cast<double>(this->milliseconds - old.milliseconds) / 1000.0;
	}

	friend std::ostream& operator<<(std::ostream& o, const Timestamp& t);
	// friend Timestamp operator-(const Timestamp& lhs, const Timestamp& rhs);
};


#endif //FT_KALMAN_TIMESTAMP_HPP
