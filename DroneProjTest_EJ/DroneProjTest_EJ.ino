#include <Wire.h>
#include <Servo.h>

const int MPU = 0x68; // mpu6050 i2c address
const float mPI = 3.141592653;

// translational
float posX, posY, posZ; // xyz position
float velX, velY, velZ; // velocity
float accX, accY, accZ; // acceleration

// angular
float AngX, AngY, AngZ; // angle (around xyz axes)
float AngVelX, AngVelY, AngVelZ; // angular velocity
float AngAccX, AngAccY, AngAccZ; // angular acceleration

Servo ESC;






float GyroX, GyroY, GyroZ; // current value of gryoscope
float accAngleX, accAngleY; // ?
float gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float accErrorX, accErrorY, gyroErrorX, gyroErrorY, gyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;





// calculates trapezoidal integral
class Integrator {
	public:
        // initialize integral to 0
        Integrator(float startingVal) {
            lastVal = startingVal;
            integral = 0;
        }
        // add value to integral and update lastVal
        // timeStep is time since last call to addValue
        void addValue(float newVal, float timeStep) {
            integral += timeStep * (lastVal + newVal)/2;
            lastVal = newVal;
        }
        // get current value of integral
        float getIntegral() {
            return integral;
        }
	private:
        // running integral
        float integral;
        // last value added to integral
        float lastVal;
};


void setupServo() {
    // (pin, min pulse width, max pulse width in microseconds)
    // motor 1 attached to pin 9
	ESC.attach(9, 1000, 2000);
	Serial.begin(9600);
    ESC.write(20);
}

// calculates derivative
float getDerivative(float val, float lastVal, float timeStep) {
	return (val - lastVal)/timeStep;
}

void getAccel() {
	// read accelerometer data
	Wire.beginTransmission(MPU);
	Wire.write(0x3B); // start with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(MPU, 6, true); // read 6 registers total, each axis value is stored in 2 registers
	// for a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
	AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
	AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
	AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
}

// prepare to get gyro data
void getGryo() {
	Wire.beginTransmission(MPU);
	Wire.write(0x43); // gyro data first register address 0x43
	Wire.endTransmission(false);
	Wire.requestFrom(MPU, 6, true); // read 4 registers total, each axis value is stored in 2 registers
    // for a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
	// the higher the range, the less precise the returned degree value
	GyroX = (Wire.read() << 8 | Wire.read()) / 131;
	GyroY = (Wire.read() << 8 | Wire.read()) / 131;
	GyroZ = (Wire.read() << 8 | Wire.read()) / 131;
}

void calcAccelAngle() {
    // calculate roll and pitch
	accAngleY = atan(-AccX/sqrt(AccY*AccY + AccZ*AccZ)) * 180 / mPI;
	accAngleX = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ)) * 180 / mPI;
}

void calcAngle() {
    // gyro current angle calculate
	// integrates the change in gyro angle over time
	// deg = deg + (deg/s)*s
	gyroAngleX = gyroAngleX + GyroX*elapsedTime;
	gyroAngleY = gyroAngleY + GyroY*elapsedTime;
}

// calculates time between each loop for integral
void calcTime() {
	previousTime = currentTime;        // previous time is stored before the actual time read
	currentTime = millis();            // current time actual time read
	elapsedTime = (currentTime - previousTime) / 1000; // convert to seconds
}

void getInput() {
    Serial.write("here we would get controller input");
}


void setup() {
	Wire.begin();                      // initialize comunication
	Wire.beginTransmission(MPU);       // start communication with MPU6050 // MPU=0x68
	Wire.write(0x6B);                  // talk to the register 6B
	Wire.write(0x00);                  // make reset - place a 0 into the 6B register
	Wire.endTransmission(true);        // end  transmission

	// write to Gyro config
	Wire.beginTransmission(MPU);
	Wire.write(0x1B);
	// 8 in hex represents Bit3 and turns the gyro into 500 deg/s
	// 0 resets to 250 deg/s (I THINK)
	Wire.write(0x0);
	Wire.endTransmission();

	// write to accel config
	Wire.beginTransmission(MPU);
	Wire.write(0x1C);
	Wire.write(0x0);
	Wire.endTransmission();
}


void loop() {
	// updates elapsedTime
	calcTime();

    // stores values in accX, accY, accZ
	getAccel();

    // calculates accAngleX and accAngleY
	calcAccelAngle();

    // prepare to get gyro data
	getGyro();

    // calculates gyroAngleX and gyroAngleY
	calcAngle();

	// complimentary filter
	roll = 0.99 * gyroAngleX + 0.01 * accAngleX;

    getInput();

	Serial.println();
}