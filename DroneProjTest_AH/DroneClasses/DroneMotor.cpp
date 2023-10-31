#include <stdexcept>
#include <Servo.h>
#include "DroneMotor.h"

#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MID_PULSE_LENGTH 1500 // Middle pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs

DroneMotor::DroneMotor() {
	throw std::invalid_argument("The DroneMotor constructor requires a single Servo argument. Use DroneMotor(Servo e) to create a DroneMotor object.");
}

DroneMotor::DroneMotor(Servo e) {
	esc = e;
}

void DroneMotor::calibrate(int p) {
	esc.attach(pin, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
	esc.write(MAX_PULSE_LENGTH);
}

void DroneMotor::setSpeed(float s) {
	if (s < 1000 || s > 2000) {
		throw std::invalid_argument("Speed argument must be a float between 1000 and 2000.");
	}
	esc.writeMicroseconds(MIN_PULSE_LENGTH + (MAX_PULSE_LENGTH - MIN_PULSE_LENGTH) * s);
	speed = s;
}

float DroneMotor::getSpeed() {
	return speed;
}