#ifndef DroneMotor_h
#define DroneMotor_h
#include <Servo.h>

class DroneMotor {
    public:
        DroneMotor();
        DroneMotor(Servo e, int p);

        void setSpeed(float s);
		float getSpeed();
	private:
		Servo esc;
		float speed;
        int pin;
};
#endif