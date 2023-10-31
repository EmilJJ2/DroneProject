#include <Servo.h>

/*
x: left / right
y: forward / backward
z: up / down (altitude)


drone motors:
1   2
4   3
CW:  13
CCW: 24


controller:
- (L)eft / (R)ight stick
- (H)orizontal / (V)ertical directions

controller inputs are directly proportional to velocities

LH: z ang velocity -> angVel[2] -> decrease 13, increase 24
LV: z velocity     -> vel[2]    -> increase 1234
RH: x velocity     -> vel[0]    -> decrease 23, increase 14
RV: y velocity     -> vel[1]    -> decrease 12, increase 34


1 = -angVel[2] + vel[2] + vel[0] - vel[1]
2 =  angVel[2] + vel[2] - vel[0] - vel[1]
3 = -angVel[2] + vel[2] - vel[0] + vel[1]
4 =  angVel[2] + vel[2] + vel[0] + vel[1]

|| To-Do || 
Velocity: X, Y, Z | Z set left stick | X,Y,Z found integrating Vel_earth from Accel_earth found from Accel_drone using rotation matrix
Angle: pitch, roll, yaw | angles set using left and right stick | angles found are absolute

Keep track of set motor speeds. Change by adding or subtracting, not setting new values. 

*/

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

  delay(1000);

  // esc1.write(MIN_PULSE_LENGTH);

}

void loop() {
  data = 0;
  Serial.println(esc1.read());

  if (Serial.available() > 0) { data = Serial.parseInt(); }

  if (data != 0) { Serial.print(data); esc1.write(data); esc2.write(data); esc3.write(data); }

}
