#include <Servo.h>
#include "DroneMotor.h"

#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MID_PULSE_LENGTH 1500 // Middle pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs

DroneMotor::DroneMotor() {
	// don't do this pls
}

DroneMotor::DroneMotor(Servo e, int p) {
	esc = e;
}

void DroneMotor::setSpeed(float s) {
	if (s < 1000) {
		s = 1000;
	}
	if (s > 2000) {
		s = 2000;
	}
	esc.writeMicroseconds(s);
	speed = s;
}

float DroneMotor::getSpeed() {
	return speed;
}
