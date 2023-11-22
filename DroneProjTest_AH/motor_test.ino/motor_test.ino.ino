#include <Servo.h>

Servo ESC1; // top right
Servo ESC2; // top left
Servo ESC3; // bottom left
Servo ESC4; // bottom right

void setup() {
	// Attach the ESC on pin 9
	// starts at 12 on 0 - 180 scale
	ESC1.attach(3, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds)
	ESC2.attach(5, 1000, 2000);
	ESC3.attach(9, 1000, 2000);
	ESC4.attach(11, 1000, 2000);

	ESC1.write(0);
	ESC2.write(0);
	ESC3.write(0);
	ESC4.write(0);
	
	delay(500);

	Serial.begin(9600);
	int i = 15;
	ESC1.write(i);
	delay(500);
	ESC1.write(0);

	ESC2.write(i);
	delay(500);
	ESC2.write(0);
	
	ESC3.write(i);
	delay(500);
	ESC3.write(0);
	
	ESC4.write(i);
	delay(500);
	ESC4.write(0);

	/*
	float ontime = 100;
	float offtime = 0;
	for (int i = 20; i <= 180; i += 20) {
		ESC1.write(i);
		delay(ontime);
		ESC1.write(0);
		delay(offtime);
	}

	for (int i = 20; i <= 180; i += 20) {
		ESC2.write(i);
		delay(ontime);
		ESC2.write(0);
		delay(offtime);
	}

	for (int i = 20; i <= 180; i += 20) {
		ESC3.write(i);
		delay(ontime);
		ESC3.write(0);
		delay(offtime);
	}

	for (int i = 20; i <= 180; i += 20) {
		ESC4.write(i);
		delay(ontime);
		ESC4.write(0);
		delay(offtime);
	}
	*/
	
	Serial.println("Done");
	
	/*
	for (int i = 20; i <= 80; i+= 10) {
			ESC2.write(i);
			delay(250);
			Serial.println(i);
	}

	for (int i = 20; i <= 80; i+= 10) {
			ESC3.write(i);
			delay(250);
			Serial.println(i);
	}

	for (int i = 20; i <= 80; i+= 10) {
			ESC4.write(i);
			delay(250);
			Serial.println(i);
	}
	*/
}

void loop() {
	ESC1.write(0);
	ESC2.write(0);
	ESC3.write(0);
	ESC4.write(0);
}