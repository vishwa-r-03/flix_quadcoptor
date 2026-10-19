// Copyright (c) 2024 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Fail-safe functions

#include "quaternion.h"
#include "util.h"

extern float controlTime;
extern float controlRoll, controlPitch, controlThrottle, controlYaw;

// missing declaration fixes
void descend();
extern Quaternion attitudeTarget;
extern float thrustTarget;
extern float dt;
extern float controlMode;

extern bool armed;
extern float t;
extern int mode;
// constants.h
extern const int RAW;
extern const int ACRO;
extern const int STAB;
extern const int AUTO;

void rcLossFailsafe();
void autoFailsafe();

float rcLossTimeout = 1;
float descendTime = 10;

void failsafe() {
	rcLossFailsafe();
	autoFailsafe();
}

// RC loss failsafe
void rcLossFailsafe() {
	if (!armed) return;
	if (t - controlTime > rcLossTimeout) {
		descend();
	}
}

// Smooth descend on RC lost
void descend() {
	mode = AUTO;
	attitudeTarget = Quaternion();
	thrustTarget -= dt / descendTime;
	if (thrustTarget < 0) {
		thrustTarget = 0;
		armed = false;
	}
}

// Allow pilot to interrupt automatic flight
void autoFailsafe() {
	static float roll, pitch, yaw, throttle;
	if (roll != controlRoll || pitch != controlPitch || yaw != controlYaw || abs(throttle - controlThrottle) > 0.05) {
		// controls changed and mode switch is not configured
		if (mode == AUTO && invalid(controlMode)) mode = STAB; // regain control by the pilot
	}
	roll = controlRoll;
	pitch = controlPitch;
	yaw = controlYaw;
	throttle = controlThrottle;
}
