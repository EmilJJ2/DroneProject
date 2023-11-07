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

#define MIN_PULSE_LENGTH 1000 // Minimum motor pulse length in µs
#define MID_PULSE_LENGTH 1500 // Middle motor pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum motor pulse length in µs

Servo motor1, motor2, motor3, motor4;
int angleYInput, angleXInput, angleZInput, velZInput, powerInput; // Controller Inputs
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
Integrator velZ;
float motor1Speed, motor2Speed, motor3Speed, motor4Speed;
float input = 0;

int accBiasCount = 0;
int gyroBiasCount = 0;
// ||||||||||| //
// || SETUP || //
// ||||||||||| //

void setup() {
  Serial.begin(19200);

  // SetupRemoteInput();

  // SetupESC();

  SetupIMU();

  // SetupPID();

}
// |||||||||| //
// || LOOP || //
// |||||||||| //

void loop() {

  CalcTime();
  
  // DefineRemoteValues();

  GetAccData();

  GetGyroData();

  GetKalmanAngles();

  // CalcPID();

  // SwitchOff(); // Must be last!!

}

// |||||||||||||||||||| //
// || BASE FUNCTIONS || //
// |||||||||||||||||||| //

void TestMotors() {
  // Takes input from Serial and sends it to the motors with a delay in between 
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

void SetupRemoteInput(){ //Set Remote Input
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), StoreRemoteValues,  FALLING); // enabling interrupt at pin 2
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

void SetupIMU() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register       
  Wire.endTransmission(true);        //end the transmission

  // Write to Gyro config
  Wire.beginTransmission(MPU);  
  Wire.write(0x1B); // 8 in Hex represents Bit3 and turns the gyro into 500 deg/s | 0 resets to 250 deg/s (I THINK)
  Wire.write(0x0); 
  Wire.endTransmission();

  // Write to Accel config
  Wire.beginTransmission(MPU);  
  Wire.write(0x1C);
  Wire.write(0x0); 
  Wire.endTransmission();
} 

void SetupPID() {
    pidAngleX = PID(0, 0.8, 0.1, 0.1);
    pidAngleY = PID(0, 0.8, 0.1, 0.1);
    pidAngleZ = PID(0, 0.8, 0.1, 0.1);

    pidVelZ = PID(0, 0.8, 0.1, 0.1);
} 

void DefineRemoteValues(){ //Get remote readings and output in degrees

  //Assign 6 channel values after separation space
  AssignRemoteValues();

  NormalizeRemoteValues();

  InterpretRemoteValues();

  ReadPowerSwitchRemoteInput();

}

void GetAccData() {
  float accXRaw, accYRaw, accZRaw;
  const float kSSF_Acc_Val = 16384.0; // Sensitivity Sensor Factor for the accelerometer from the datasheet

  // Set previous values | This is to keep track of the values found in both the current loop and the last loop for the IIR Filter
  accXPrev = accXCurr;
  accYPrev = accYCurr;
  accZPrev = accZCurr;
  
  // Read accelerometer data | Accelerometer outputs in g units. So a reading of a means a*(9.8 m/s^2)
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  accXRaw = (Wire.read() << 8 | Wire.read()) / kSSF_Acc_Val; // X-axis value
  accYRaw = (Wire.read() << 8 | Wire.read()) / kSSF_Acc_Val; // Y-axis value
  accZRaw = (Wire.read() << 8 | Wire.read()) / kSSF_Acc_Val; // Z-axis value
  
  // Bias removal [Must occur after RAW data retrieval and before RAW data is used]
  // This counts the first 200 data values and uses them to set the expected initial values
  if (accBiasCount < sensorBiasConst) {
    // Finds Bias
    accXBias += accXRaw;
    accYBias += accYRaw;
    accZBias += accZRaw;
    accBiasCount++;
  } else {
    accBiasCalibrating = false;
    // Removes Bias
    accXRaw -= (accXBias / sensorBiasConst);
    accYRaw -= (accYBias / sensorBiasConst);
    accZRaw -= (accZBias / sensorBiasConst) - 1;
  }

  // IIR Filter
  accXCurr = IIRFilter(accXRaw, accXPrev);
  accYCurr = IIRFilter(accYRaw, accYPrev);
  accZCurr = IIRFilter(accZRaw, accZPrev);

  // Accel Angle Calculations
  accAngleY = atan(-accXCurr/sqrt(accYCurr*accYCurr + accZCurr*accZCurr)) * 180 / mPI;
  accAngleX = atan(accYCurr/sqrt(accXCurr*accXCurr + accZCurr*accZCurr)) * 180 / mPI;

  // Calculate Vel Z
  velZ.addValue(accZCurr, elapsedTime);
  

}

void GetGyroData() {
  float gyroXRaw, gyroYRaw, gyroZRaw;
  const float kSSF_Gyro_Val = 131; // Sensitivity Sensor Factor for the gyroscope from the datasheet

  gyroXPrev = gyroXCurr;
  gyroYPrev = gyroYCurr;
  gyroZPrev = gyroZCurr;

  // Read gyro data | Gyro outputs in deg/s
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  gyroXRaw = (Wire.read() << 8 | Wire.read()) / kSSF_Gyro_Val; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  gyroYRaw = (Wire.read() << 8 | Wire.read()) / kSSF_Gyro_Val; // The higher the range, the less precise the returned degree value
  gyroZRaw = (Wire.read() << 8 | Wire.read()) / kSSF_Gyro_Val;

  if (gyroBiasCount < sensorBiasConst) {
    // Finds Bias
    gyroXBias += gyroXRaw;
    gyroYBias += gyroYRaw;
    gyroZBias += gyroZRaw;
    gyroBiasCount++;

  } else {
    gyroBiasCalibrating = false;
    // Removes Bias
    gyroXRaw -= (gyroXBias / sensorBiasConst);
    gyroYRaw -= (gyroYBias / sensorBiasConst); 
    gyroZRaw -= (gyroZBias / sensorBiasConst);
  }

  gyroXCurr = IIRFilter(gyroXRaw, gyroXPrev); // This was not working for some reason so it is commented out
  gyroYCurr = IIRFilter(gyroYRaw, gyroYPrev);
  gyroZCurr = IIRFilter(gyroZRaw, gyroZPrev);

}

void GetKalmanAngles(){
  angleX = kalmanAngleX.getAngle(accAngleX, gyroXCurr, elapsedTime);
  angleY = kalmanAngleY.getAngle(accAngleY, gyroYCurr, elapsedTime);
}


void FindErrors() {

  angleXError = angleXInput - angleX;
  angleYError = angleYInput - angleY;
  angleZError = angleZInput - angleZ;

  velZError = velZInput - velZ.getIntegral();

}


void CalcPID() {
    //FindErrors();

    pidAngleX.addValue(angleXError, elapsedTime);
    pidAngleY.addValue(angleYError, elapsedTime);
    pidAngleZ.addValue(angleZError, elapsedTime);
    pidVelZ.addValue(velZError, elapsedTime);

}

void SetMotorSpeeds() { // STILL HAS TO BE CHANGED
  float velOutputConst = 1000;
  float angleOutputConst = 1000;
  motor1Speed = velZ.getIntegral() + angleX - angleY + angleZ;
  motor2Speed = velZ.getIntegral() - angleX - angleY - angleZ;
  motor3Speed = velZ.getIntegral() + angleX + angleY - angleZ;
  motor4Speed = velZ.getIntegral() - angleX + angleY + angleZ;

  CutoffMotorSpeeds();
}

void CutoffMotorSpeeds() {
  const float kMaxMotorSpeed = 1900;
  const float kMinMotorSpeed = 1100; // CHECK!!
  if (motor1Speed > kMaxMotorSpeed) { motor1Speed = kMaxMotorSpeed; }
  if (motor2Speed > kMaxMotorSpeed) { motor2Speed = kMaxMotorSpeed; }
  if (motor3Speed > kMaxMotorSpeed) { motor3Speed = kMaxMotorSpeed; }
  if (motor4Speed > kMaxMotorSpeed) { motor4Speed = kMaxMotorSpeed; }

  if (motor1Speed < kMinMotorSpeed) { motor1Speed = kMinMotorSpeed; }
  if (motor2Speed < kMinMotorSpeed) { motor2Speed = kMinMotorSpeed; }
  if (motor3Speed < kMinMotorSpeed) { motor3Speed = kMinMotorSpeed; }
  if (motor4Speed < kMinMotorSpeed) { motor4Speed = kMinMotorSpeed; }
}

void SwitchOff() {
  if (!powerSwitch) {
    motor1.write(1000);
    motor2.write(1000);
    motor3.write(1000);
    motor4.write(1000);
  }
}

// ||||||||||||||||||||||||||| //
// || BASE HELPER FUNCTIONS || //
// ||||||||||||||||||||||||||| //

// REMOTE FUNCTIONS
void NormalizeRemoteValues() {

  int deadzone = 8;

    // Working Values
  angleXInput = ch[1];
  velZInput = ch[2];
  angleYInput = ch[3];
  angleZInput = ch[4];
  powerInput = ch[5];

  // Set to base Value
  angleXInput -= 500;
  velZInput -= 500;
  angleYInput -= 500;
  angleZInput -= 500;

  // Deadzone
  if (angleXInput >= -deadzone && angleXInput <= deadzone) { angleXInput = 0; }
  if (velZInput >= -deadzone && velZInput <= deadzone) { velZInput = 0; }
  if (angleYInput >= -deadzone && angleYInput <= deadzone) { angleYInput = 0; }
  if (angleZInput >= -deadzone && angleZInput <= deadzone) { angleZInput = 0; }
}

void InterpretRemoteValues() {
  float max_sensor_reading = 500;
  float max_angle_output = 20; // degrees
  float angle_const = max_angle_output / max_sensor_reading;

  // Change reciever readings to degrees
  angleXInput = float(angleXInput * angle_const); // float() required for int division to work correctly
  velZInput = float(velZInput * angle_const);
  angleYInput = float(angleYInput * angle_const);
  angleZInput = float(angleZInput * angle_const);
}

void ReadPowerSwitchRemoteInput() {
  if (powerInput < 50 && powerInput > -50) {
    powerSwitch = true; // Up on the ch5 switch sends a signal around 0, which I set to be on
  } else { powerSwitch = false; } // If the switch is anything but up, or not working, power is off
}

void CalcTime() {
	previousTime = currentTime;        // previous time is stored before the actual time read
	currentTime = millis();            // current time actual time read
	elapsedTime = (currentTime - previousTime) / 1000; // convert to seconds
}

void StoreRemoteValues(){ //Store all values from temporary array
  //this code reads value from RC reciever from PPM pin (Pin 2 or  3)
  //this code gives channel values from 0-1000 values
  unsigned int a, c;
  a = micros(); //store time value a when pin value falling
  c = a - remoteTimeDifference;      //calculating  time inbetween two peaks
  remoteTimeDifference = a;        // 
  strx[store_x]=c;     //storing 15 value in  array
  store_x = store_x + 1;
  if(store_x==15){
    for(int j=0;j<15;j++){
      ppm[j]=strx[j];
    }
    store_x = 0;
  }
}

void AssignRemoteValues(){ //Take stored remote values and output to loop
  int  i,j,k=0;
  for(k=14;k>-1;k--){
    if(ppm[k]>5000) {
      j=k;
    }
  }  //detecting separation  space 10000us in that another array  | CHANGING IT TO 5000us FIXED IT FOR EMIL                   
  for(i=1;i<=6;i++){
    ch[i]=(ppm[i+j]-1000);
  }
}     

float IIRFilter(float previous_value, float current_value) {
  float alpha = 0.2;
  float output = alpha*previous_value + (1-alpha)*current_value;
  return output;
}