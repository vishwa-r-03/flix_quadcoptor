// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Utility functions

#pragma once

#include <Arduino.h>
#include <math.h>


extern const float ONE_G;
extern float t;

float mapf(float x, float in_min, float in_max, float out_min, float out_max);

bool invalid(float x);

bool valid(float x) ;

// Wrap angle to [-PI, PI)
float wrapAngle(float angle);

// Trim and split string by spaces
void splitString(String& str, String& token0, String& token1, String& token2);

// Rate limiter
class Rate {
public:
	float rate;
	float last = 0;
	Rate(float rate) : rate(rate) {}

	operator bool() {
		if (t - last >= 1 / rate) {
			last = t;
			return true;
		}
		return false;
	}
};

// Delay filter for boolean signals - ensures the signal is on for at least 'delay' seconds
class Delay {
public:
	float delay;
	float start = NAN;
	Delay(float delay) : delay(delay) {}

	bool update(bool on) {
		if (!on) {
			start = NAN;
			return false;
		} else if (isnan(start)) {
			start = t;
		}
		return t - start >= delay;
	}
};
