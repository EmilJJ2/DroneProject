#include <Servo.h>

#define MIN_PULSE_LENGTH 1000 // Minimum motor pulse length in µs
#define MID_PULSE_LENGTH 1500 // Middle motor pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum motor pulse length in µs

Servo motor1, motor2, motor3, motor4;

void setup() {
  Serial.begin(19200);
  SetupESC();
}

void loop() {
  motor1.write(1100);
  delay(4000);
  motor1.write(1200);
  delay(4000);
  motor1.write(1300);
  delay(4000);
  motor1.write(1400);

}

void SetupESC(){
  // connect motors to digital 4,5,6,7 pins
  motor1.attach(4, 1000, 2000);
  motor2.attach(5, 1000, 2000);
  motor3.attach(6, 1000, 2000);
  motor4.attach(7, 1000, 2000);

  motor1.write(MAX_PULSE_LENGTH);
  motor2.write(MAX_PULSE_LENGTH);
  motor3.write(MAX_PULSE_LENGTH);
  motor4.write(MAX_PULSE_LENGTH);
  Serial.println("MAX");

  delay(8000);

  motor1.write(MIN_PULSE_LENGTH);
  motor2.write(MIN_PULSE_LENGTH);
  motor3.write(MIN_PULSE_LENGTH);
  motor4.write(MIN_PULSE_LENGTH);
  Serial.println("MIN");


}

