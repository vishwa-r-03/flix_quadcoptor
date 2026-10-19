#pragma once

#define LOG_RATE 100
#define LOG_DURATION 10
#define LOG_SIZE LOG_DURATION * LOG_RATE

struct LogEntry {
	const char *name;
	float *value;
};

// i hate the way it is handled
extern float logBuffer[LOG_SIZE][14];
