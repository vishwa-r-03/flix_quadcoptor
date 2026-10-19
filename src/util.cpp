#include "util.h"

const float ONE_G = 9.80665;

float wrapAngle(float angle) {
	angle = fmodf(angle, 2 * PI);
	if (angle > PI) {
		angle -= 2 * PI;
	} else if (angle < -PI) {
		angle += 2 * PI;
	}
	return angle;
}

void splitString(String& str, String& token0, String& token1, String& token2) {
	str.trim();
	if (str.isEmpty()) return;
	char chars[str.length() + 1];
	str.toCharArray(chars, str.length() + 1);
	token0 = strtok(chars, " ");
	token1 = strtok(NULL, " ");
	token2 = strtok(NULL, "");
	if (token1.c_str() == NULL) token1 = "";
	if (token2.c_str() == NULL) token2 = "";
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool invalid(float x) {
	return !isfinite(x);
}

bool valid(float x) {
	return isfinite(x);
}