#include <Servo.h>

#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MID_PULSE_LENGTH 1500 // Middle pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs

Servo esc1, esc2, esc3, esc4;
int data = 0;

void setup() {
	Serial.begin(9600);

	esc1.attach(4, 1000, 2000);
	esc2.attach(5, 1000, 2000);
	esc3.attach(6, 1000, 2000);

	Serial.println("Wrote Max");
	esc1.write(MAX_PULSE_LENGTH);
	esc2.write(MAX_PULSE_LENGTH);
	esc3.write(MAX_PULSE_LENGTH);

	delay(1000);
	// esc1.write(MIN_PULSE_LENGTH);
}

void loop() {
	data = 0;
	if (Serial.available() > 0) {
		data = Serial.parseInt();
	}
	if (data != 0) {
		Serial.print(data);
		esc1.write(data);
		esc2.write(data);
		esc3.write(data);
	}
}
