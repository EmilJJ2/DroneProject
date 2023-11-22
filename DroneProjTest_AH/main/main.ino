/*
1   2
 \ /
  ^
 / \
3   4
*/

// || INCLUDES || //
#include <Servo.h>
#include <Wire.h>
#include <Integrator.h>
#include <PID.h>
#include <Kalman.h>

// ||||||||||||||||||||||| //
// || SET GLOBAL VALUES || //
// ||||||||||||||||||||||| //

#define MIN_PULSE_LENGTH 1000 // minimum motor pulse length in µs
#define MID_PULSE_LENGTH 1500 // middle motor pulse length in µs
#define MAX_PULSE_LENGTH 2000 // maximum motor pulse length in µs

Servo motor1, motor2, motor3, motor4;
int angleYInput, angleXInput, angleZInput, velZInput, powerInput; // controller inputs
bool powerSwitch = false;

unsigned long int remoteTimeDifference;
int strx[15], ppm[15], ch[7], store_x;

const int MPU = 0x68; // MPU6050 I2C address
float mPI = 3.141592653;
int sensorBiasConst = 200;
bool gyroBiasCalibrating = true;
bool accBiasCalibrating = true;

float accXCurr, accYCurr, accZCurr, accXPrev, accYPrev, accZPrev, accXBias, accYBias, accZBias;
float gyroXCurr, gyroYCurr, gyroZCurr, gyroXPrev, gyroYPrev, gyroZPrev, gyroXBias, gyroYBias, gyroZBias;
float accAngleX, accAngleY, accAngleZ, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float accErrorX, accErrorY, gyroErrorX, gyroErrorY, gyroErrorZ;
float elapsedTime, currentTime, previousTime;
PID pidAngleX, pidAngleY, pidAngleZ, pidVelZ;
float angleXError, angleYError, angleZError, velZError;
Kalman kalmanAngleX, kalmanAngleY, kalmanAngleZ;
float angleX, angleY, angleZ;
float outputAngleX, outputAngleY, outputAngleZ, outputVelZ;
Integrator velZ;
float motor1Speed, motor2Speed, motor3Speed, motor4Speed;
float input = 0;
bool startup = true;

int accBiasCount = 0;
int gyroBiasCount = 0;

// ||||||||||| //
// || SETUP || //
// ||||||||||| //
void setup() {
	Serial.begin(19200);

	setupRemoteInput();
	setupESC();
	setupIMU();
	setupPID();
}

// |||||||||| //
// || LOOP || //
// |||||||||| //

void loop() {
	calcTime();
	defineRemoteValues();
	getSensorData();

	if (doneCalibrating()) {
		if (startup) { // run once on startup
			setupMotorVals();
			startup = false;
		}
		getKalmanAngles();
		calcPID();
	}

	// PowerSwitch(); // must be last!!
	scaleMotorSpeeds();
	setMotorSpeeds();
	sendMotorSpeeds();
	Serial.println(angleXInput);
}

// |||||||||||||||||||| //
// || BASE FUNCTIONS || //
// |||||||||||||||||||| //

void setupRemoteInput() { // set remote input
	pinMode(2, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(2), storeRemoteValues, FALLING); // enabling interrupt at pin 2
}

void setupESC(){
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

	delay(4000);

	motor1.write(MIN_PULSE_LENGTH);
	motor2.write(MIN_PULSE_LENGTH);
	motor3.write(MIN_PULSE_LENGTH);
	motor4.write(MIN_PULSE_LENGTH);
	Serial.println("MIN");
	delay(4000);
}

void setupIMU() {
	Wire.begin();                // initialize comunication
	Wire.beginTransmission(MPU); // start communication with MPU6050 -> MPU = 0x68
	Wire.write(0x6B);            // talk to the register 6B
	Wire.write(0x00);            // make reset - place a 0 into the 6B register
	Wire.endTransmission(true);  // end the transmission

	// write to gyro config
	Wire.beginTransmission(MPU);
	Wire.write(0x1B); // 8 in Hex represents Bit3 and turns the gyro into 500 deg/s | 0 resets to 250 deg/s (I THINK)
	Wire.write(0x0);
	Wire.endTransmission();

	// write to accel config
	Wire.beginTransmission(MPU);
	Wire.write(0x1C);
	Wire.write(0x0);
	Wire.endTransmission();
}

// initialize PID values
void setupPID() {
	pidAngleX = PID(0, 0.8, 0.1, 0.1);
	pidAngleY = PID(0, 0.8, 0.1, 0.1);
	pidAngleZ = PID(0, 0.8, 0.1, 0.1);
	pidVelZ   = PID(0, 0.8, 0.1, 0.1);
}

// calculate elapsed time
void calcTime() {
	previousTime = currentTime; // store previous time before actual time read
	currentTime = millis();     // read current time
	elapsedTime = (currentTime - previousTime) / 1000; // convert to seconds
}

// get readings from remote and output in degrees
// sets angleXInput, angleYInput, angleZInput, velZInput
void defineRemoteValues() {
	// assign 6 channel values after separation space
	assignRemoteValues();
	normalizeRemoteValues();
	interpretRemoteValues();
	readPowerSwitchRemoteInput();
}

// get data from sensors
void getSensorData() {
	getAccData();
	getGyroData();
}

bool doneCalibrating() {
	return !accBiasCalibrating && !gyroBiasCalibrating;
}

void setupMotorVals() {
	const float kStartSpeed = 1150; // test to find right value
	motor1Speed = kStartSpeed;
	motor2Speed = kStartSpeed;
	motor3Speed = kStartSpeed;
	motor4Speed = kStartSpeed;
}

void getKalmanAngles(){
	angleX = kalmanAngleX.getAngle(accAngleX, gyroXCurr, elapsedTime);
	angleY = kalmanAngleY.getAngle(accAngleY, gyroYCurr, elapsedTime);
}

void calcPID() {
	findErrors();

	pidAngleX.addValue(angleXError, elapsedTime);
	pidAngleY.addValue(angleYError, elapsedTime);
	pidAngleZ.addValue(angleZError, elapsedTime);
	pidVelZ.addValue(velZError, elapsedTime);
}

void sendMotorSpeeds() {
	// two negative, two positive
	motor1.write(motor1Speed);
	motor2.write(motor2Speed);
	motor3.write(motor3Speed);
	motor4.write(motor4Speed);
}

void powerSwitch() {
	if (!powerSwitch) { switchOff(); }
}

// unused, takes input from serial and sends it to the motors with a delay in between
void testMotors() {
	input = Serial.parseFloat();
	if (input != 0) {
		motor1.write(input);
		delay(1000);
		motor2.write(input);
		delay(1000);
		motor3.write(input);
		delay(1000);
		motor4.write(input);
	}
}


// ||||||||||||||||||||||||||| //
// || BASE HELPER FUNCTIONS || //
// ||||||||||||||||||||||||||| //

// get data from accelerometer
void getAccData() {
	float accXRaw, accYRaw, accZRaw;
	const float kSSF_Acc_Val = 16384.0; // sensitivity Sensor Factor for accelerometer from datasheet

	// set previous values | this is to keep track of the values found in both the current loop and the last loop for the IIR Filter
	accXPrev = accXCurr;
	accYPrev = accYCurr;
	accZPrev = accZCurr;

	// read accelerometer data | accelerometer outputs in g units | reading of a means a*(9.8 m/s^2)
	Wire.beginTransmission(MPU);
	Wire.write(0x3B); // start with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(MPU, 6, true); // read 6 registers total, each axis value is stored in 2 registers
	// for a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
	accXRaw = (Wire.read() << 8 | Wire.read()) / kSSF_Acc_Val; // x-axis value
	accYRaw = (Wire.read() << 8 | Wire.read()) / kSSF_Acc_Val; // y-axis value
	accZRaw = (Wire.read() << 8 | Wire.read()) / kSSF_Acc_Val; // z-axis value

	// bias removal [must occur after RAW data retrieval and before RAW data is used]
	// this counts the first 200 data values and uses them to set the expected initial values
	if (accBiasCount < sensorBiasConst) {
		// finds bias
		accXBias += accXRaw;
		accYBias += accYRaw;
		accZBias += accZRaw;
		accBiasCount++;
	} else {
		accBiasCalibrating = false;
		// removes bias
		accXRaw -= (accXBias / sensorBiasConst);
		accYRaw -= (accYBias / sensorBiasConst);
		accZRaw -= (accZBias / sensorBiasConst) - 1;

		// IIR filter
		accXCurr = IIRFilter(accXRaw, accXPrev);
		accYCurr = IIRFilter(accYRaw, accYPrev);
		accZCurr = IIRFilter(accZRaw, accZPrev);

		// accel angle calculations
		accAngleY = atan(-accXCurr/sqrt(accYCurr*accYCurr + accZCurr*accZCurr)) * 180 / mPI;
		accAngleX = atan(accYCurr/sqrt(accXCurr*accXCurr + accZCurr*accZCurr)) * 180 / mPI;

		// calculate velZ
		velZ.addValue(accZCurr * 9.8 - 9.8, elapsedTime);
	}
}

// get data from gyroscope
void getGyroData() {
	float gyroXRaw, gyroYRaw, gyroZRaw;
	const float kSSF_Gyro_Val = 131; // Sensitivity Sensor Factor for the gyroscope from the datasheet

	gyroXPrev = gyroXCurr;
	gyroYPrev = gyroYCurr;
	gyroZPrev = gyroZCurr;

	// read gyro data | gyro outputs in deg/s
	Wire.beginTransmission(MPU);
	Wire.write(0x43); // gyro data first register address 0x43
	Wire.endTransmission(false);
	Wire.requestFrom(MPU, 6, true); // read 4 registers total, each axis value is stored in 2 registers
	gyroXRaw = (Wire.read() << 8 | Wire.read()) / kSSF_Gyro_Val; // for a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
	gyroYRaw = (Wire.read() << 8 | Wire.read()) / kSSF_Gyro_Val; // the higher the range, the less precise the returned degree value
	gyroZRaw = (Wire.read() << 8 | Wire.read()) / kSSF_Gyro_Val;

	if (gyroBiasCount < sensorBiasConst) {
		// finds bias
		gyroXBias += gyroXRaw;
		gyroYBias += gyroYRaw;
		gyroZBias += gyroZRaw;
		gyroBiasCount++;

	} else {
		gyroBiasCalibrating = false;
		// removes bias
		gyroXRaw -= (gyroXBias / sensorBiasConst);
		gyroYRaw -= (gyroYBias / sensorBiasConst);
		gyroZRaw -= (gyroZBias / sensorBiasConst);
	}

	gyroXCurr = IIRFilter(gyroXRaw, gyroXPrev); // this was not working for some reason so it is commented out
	gyroYCurr = IIRFilter(gyroYRaw, gyroYPrev);
	gyroZCurr = IIRFilter(gyroZRaw, gyroZPrev);
}

// sets motor speeds to 0
void switchOff() {
	const float kPWMOff = 1000;
	motor1Speed = kPWMOff;
	motor2Speed = kPWMOff;
	motor3Speed = kPWMOff;
	motor4Speed = kPWMOff;
}

// normalize remote values to be between -500 and 500
void normalizeRemoteValues() {
	int deadzone = 8;

	// working values
	angleXInput = ch[1];
	velZInput = ch[2];
	angleYInput = ch[3];
	angleZInput = ch[4];
	powerInput = ch[5];

	// set to base value
	angleXInput -= 500;
	velZInput -= 500;
	angleYInput -= 500;
	angleZInput -= 500;

	// deadzone
	if (angleXInput >= -deadzone && angleXInput <= deadzone) { angleXInput = 0; }
	if (angleYInput >= -deadzone && angleYInput <= deadzone) { angleYInput = 0; }
	if (angleZInput >= -deadzone && angleZInput <= deadzone) { angleZInput = 0; }
	if (velZInput >= -deadzone && velZInput <= deadzone) { velZInput = 0; }
}

// convert remote values to degrees
void interpretRemoteValues() {
	float max_sensor_reading = 500;
	float max_angle_output = 20; // degrees
	float angle_const = max_angle_output / max_sensor_reading;

	// change receiver readings to degrees
	angleXInput = float(angleXInput * angle_const); // float() required for int division to work correctly
	angleYInput = float(angleYInput * angle_const);
	angleZInput = float(angleZInput * angle_const);
	velZInput = float(velZInput * angle_const);
}

// read power switch from remote
void readPowerSwitchRemoteInput() {
	if (powerInput < 50 && powerInput > -50) {
		powerSwitch = true; // up on the ch5 switch sends a signal around 0, which I set to be on
	} else { powerSwitch = false; } // if the switch is anything but up, or not working, power is off
}

// store all values from temporary array to ppm array
void storeRemoteValues(){
	// this code reads value from RC reciever from PPM pin (Pin 2 or 3)
	// this code gives channel values from 0-1000 values
	unsigned int a, c;
	a = micros(); // store time value a when pin value falling
	c = a - remoteTimeDifference; // calculating time in between two peaks
	remoteTimeDifference = a;
	strx[store_x] = c; // storing 15 value in array
	store_x++;
	if (store_x == 15) {
		for (int j = 0; j < 15; j++) {
			ppm[j] = strx[j];
		}
		store_x = 0;
	}
}

// take stored remote values and output to loop
void assignRemoteValues() {
	int i, j, k;
	for (k = 14; k >= 0; k--) {
		if (ppm[k] > 5000) {
			j = k;
		}
	} // detecting separation space 10000 µs in that another array | CHANGING IT TO 5000 µs FIXED IT FOR EMIL
	for (i = 1; i <= 6; i++){
		ch[i] = ppm[i+j] - 1000;
	}
}

// implement IIR filter
float IIRFilter(float previous_value, float current_value) {
	float alpha = 0.2;
	float output = alpha*previous_value + (1-alpha)*current_value;
	return output;
}

// calculate error (difference between desired and actual values)
void findErrors() {
	angleXError = angleXInput - angleX;
	angleYError = angleYInput - angleY;
	angleZError = angleZInput - angleZ;
	velZError = velZInput - velZ.getIntegral();
}

// scale motor speeds down to max range (+- 20 degrees and +- 0.2 m/s)
void scaleMotorSpeeds() {
	const float kMaxDegree = 20;
	const float kMaxZVelocity = 0.2;
	outputAngleX = pidAngleX.calcValue() * 500 / kMaxDegree;
	outputAngleY = pidAngleY.calcValue() * 500 / kMaxDegree;
	outputAngleZ = pidAngleZ.calcValue() * 500 / kMaxDegree;
	outputVelZ = pidVelZ.calcValue() * 500 / kMaxZVelocity;
}

// calculate change in motor speeds based on output angles and z vel
void setMotorSpeeds() { // TODO: FIX THIS
	float velOutputConst = 2/5;
	float angleOutputConst = 1/5;

	// add change in motor speeds
	motor1Speed += velOutputConst * outputVelZ + angleOutputConst*(outputAngleX - outputAngleY + outputAngleZ);
	motor2Speed += velOutputConst * outputVelZ - angleOutputConst*(outputAngleX - outputAngleY - outputAngleZ);
	motor3Speed += velOutputConst * outputVelZ + angleOutputConst*(outputAngleX + outputAngleY - outputAngleZ);
	motor4Speed += velOutputConst * outputVelZ - angleOutputConst*(outputAngleX + outputAngleY + outputAngleZ);
	
	cutoffMotorSpeeds(); // make sure motor speeds are within cuttoff points
}

// set motor speeds between 1100 and 1900
void cutoffMotorSpeeds() {
	const float kMaxMotorSpeed = 1900;
	const float kMinMotorSpeed = 1100; // TODO: CHECK!!
	if (motor1Speed > kMaxMotorSpeed) { motor1Speed = kMaxMotorSpeed; }
	if (motor2Speed > kMaxMotorSpeed) { motor2Speed = kMaxMotorSpeed; }
	if (motor3Speed > kMaxMotorSpeed) { motor3Speed = kMaxMotorSpeed; }
	if (motor4Speed > kMaxMotorSpeed) { motor4Speed = kMaxMotorSpeed; }

	if (motor1Speed < kMinMotorSpeed) { motor1Speed = kMinMotorSpeed; }
	if (motor2Speed < kMinMotorSpeed) { motor2Speed = kMinMotorSpeed; }
	if (motor3Speed < kMinMotorSpeed) { motor3Speed = kMinMotorSpeed; }
	if (motor4Speed < kMinMotorSpeed) { motor4Speed = kMinMotorSpeed; }
}