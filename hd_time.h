#pragma once

#include "hd_types.h"
#include <chrono>

ts_t getCurrentTime() {
	using namespace std::chrono;
	return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}
