// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Main firmware file

#include "vector.h"
#include "quaternion.h"
#include "util.h"

extern float t, dt;
extern float controlRoll, controlPitch, controlYaw, controlThrottle, controlMode;
extern Vector gyro, acc;
extern Vector rates;
extern Quaternion attitude;
extern bool landed;
extern float motors[4];

// fix missing declarations
void setupParameters();
void setupPower();
void setupLED();
void setLED(bool on);
void setupMotors();
void setupWiFi();
void setupIMU();
void setupRC();
void readIMU();
void step();
void readRC();
void estimate();
void control();void sendMotors();
void handleInput();
void processMavlink();
void readVoltage();
void logData();
void syncParameters();
void print(const char* format, ...);


void setup() {
	Serial.begin(115200);
	print("Initializing flix\n");
	setupParameters();
	setupPower();
	setupLED();
	setLED(true);
	setupMotors();
	setupWiFi();
	setupIMU();
	setupRC();
	setLED(false);
	print("Initializing complete\n");
}

void loop() {
	readIMU();
	step();
	readRC();
	estimate();
	control();
	sendMotors();
	handleInput();
	processMavlink();
	readVoltage();
	logData();
	syncParameters();
}
