#include <Wire.h>
#include <Servo.h>

const int MPU = 0x68; // mpu6050 i2c address
const float mPI = 3.141592653;

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
*/

// translational {x, y, z}
float[3] pos;
float[3] vel;
float[3] acc;

// angular {x, y, x}
float[3] ang;
float[3] angVel;
float[3] angAcc;

float[3] gyro; // current value of gryoscope - {x, y, z}
float accAngleXZ, accAngleYZ; // acceleration on each plane
float[3] gyroAngle; // {x, y, z}
float roll, pitch, yaw; // ?
float[3] accError;
float[3] gyroError;

float[4] motorVal;

float elapsedTime, currentTime, previousTime;
int c = 0;

Servo ESC1; // top left
Servo ESC2; // top right
Servo ESC3; // bottom right
Servo ESC4; // bottom left


// calculates trapezoidal integral
class Integrator {
	public:
        Integrator(float startingVal) {
            lastVal = startingVal;
            // initialize integral to 0
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

class PID {
    public:
        PID(float startingVal, float pw, float iw, float dw) {
            p = startingVal;
            Integrator i = new Integrator(p);
            lastVal = p;
            d = 0;

            pWeight = pw;
            iWeight = iw;
            dWeight = dw;
        }

        void addValue(float newVal, float timeStep) {
            p = newVal;
            i.addValue();
            d = getDerivative(lastVal, newVal, timeStep);
            lastVal = newVal;
        }

        float calcValue() {
            return (p * pWeight + i * iWeight + d * dWeight);
        }
    private:
        float p, i, d, lastVal;
}


void setupSensors() {
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

void setupPID() {
    angZ_PID = new PID(0, 0.8, 0.1, 0.1);
    velZ_PID = new PID(0, 0.8, 0.1, 0.1);
    velX_PID = new PID(0, 0.8, 0.1, 0.1);
    velY_PID = new PID(0, 0.8, 0.1, 0.1);
}

void setupServo() {
    // (pin, min pulse width, max pulse width in microseconds)
    // motor 1 attached to pin 9
	ESC1.attach(9, 1000, 2000);
	Serial.begin(9600);
    ESC1.write(0);
    // TODO: attach ESC2, ESC3, ESC4
}

// calculates derivative
float getDerivative(float lastVal, float curVal, float timeStep) {
	return (curVal - lastVal) / timeStep;
}

void getAccel() {
	// read accelerometer data
	Wire.beginTransmission(MPU);
	Wire.write(0x3B); // start with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(MPU, 6, true); // read 6 registers total, each axis value is stored in 2 registers
	// for a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
	acc[0] = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
	acc[1] = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
	acc[2] = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
}

// prepare to get gyro data
void getGryo() {
	Wire.beginTransmission(MPU);
	Wire.write(0x43); // gyro data first register address 0x43
	Wire.endTransmission(false);
	Wire.requestFrom(MPU, 6, true); // read 4 registers total, each axis value is stored in 2 registers
    // for a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
	// the higher the range, the less precise the returned degree value
	gyro[0] = (Wire.read() << 8 | Wire.read()) / 131.0;
	gyro[1] = (Wire.read() << 8 | Wire.read()) / 131.0;
	gyro[2] = (Wire.read() << 8 | Wire.read()) / 131.0;
}

void calcAccelAngle() {
    // calculate roll and pitch
    accAngleXZ = atan(acc[1] / dist(acc[0], acc[2])) * 180 / mPI;
	accAngleYZ = -atan(acc[0] / dist(acc[1], acc[2])) * 180 / mPI;
}

void calcAngle() {
    // gyro current angle calculate
	// integrates the change in gyro angle over time
	// deg = deg + (deg/s)*s
	gyroAngle[0] += gyro[0] * elapsedTime;
	gyroAngle[1] += gyro[1] * elapsedTime;
}

// calculates time between each loop for integral
void calcTime() {
	previousTime = currentTime;        // previous time is stored before the actual time read
	currentTime = millis();            // current time actual time read
	elapsedTime = (currentTime - previousTime) / 1000; // convert to seconds
}

void getInput() {
    Serial.write("here we would get controller input and store in angVel[2], vel[2], vel[0], vel[1]");
    /*
    angVel[2] = lh * scalingConstant
    vel[2] = lv * scalingConstant
    vel[0] = rh * scalingConstant
    vel[1] = rv * scalingConstant
    */
}

void calcPID() {
    angZ_PID.addValue(angVel[2], elapsedTime);
    velZ_PID.addValue(vel[0], elapsedTime);
    velX_PID.addValue(vel[1], elapsedTime);
    velY_PID.addValue(vel[2], elapsedTime);

    angVel[2] = angZ_PID.calcValue();
    vel[2] = velZ_PID.calcValue();
    vel[0] = velX_PID.calcValue();
    vel[1] = velY_PID.calcValue();
}

// TODO: find exact equations turning position & angular velocity into motor values
// for example moving in x direction is not a linear function, prob uses sine / cosine
void updateMotors() {
    // filler text
    /*
    
    motorVal[0] = -angVel[2] + vel[2] + vel[0] - vel[1]
    motorVal[1] =  angVel[2] + vel[2] - vel[0] - vel[1]
    motorVal[2] = -angVel[2] + vel[2] - vel[0] + vel[1]
    motorVal[3] =  angVel[2] + vel[2] + vel[0] + vel[1]
    
    ESC1.write(motorVal[0] * scalingConstant)
    ESC2.write(motorVal[1] * scalingConstant)
    ESC3.write(motorVal[2] * scalingConstant)
    ESC4.write(motorVal[3] * scalingConstant)

    */
}




void setup() {
    setupSensors();
    setupPID();
}


void loop() {
	// updates elapsedTime
	calcTime();

    // stores values in acc[]
	getAccel();

    // calculates accAngleXZ and accAngleYZ
	calcAccelAngle();

    // prepare to get gyro data
	getGyro();

    // calculates gyroAngle
    // TODO: turn into integrator
	calcAngle();


	// complimentary filter
	roll = 0.99 * gyroAngle[0] + 0.01 * accAngle[0];

    // get input from controllers, update angVel[2] and vel
    getInput();

    // add new values to PID, update angVel[2] and vel with PID values
    calcPID();

    // calculate each motor speed and update based on angVel and vel
    updateMotors();
}

// distance between two points, can also be used for vector magnitude
float dist(x1, y1, x2 = 0, y2 = 0) {
    dx = x2 - x1;
    dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}