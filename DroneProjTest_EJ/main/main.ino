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

enum class DroneState { POWEROFF, HOVER, SETDOWN, CALIBRATING, STANDBY };
enum class CH5 { UP, MID, DOWN };

Servo motor1, motor2, motor3, motor4;
float angleYInput, angleXInput, angleZInput, velZInput, powerInput; // Controller Inputs | TODO: CHECK IF THIS SHOULD BE FLOAT OR INT

unsigned long int remoteTimeDifference;
int strx[15], ppm[15], ch[7], store_x;

const int MPU = 0x68; // MPU6050 I2C address
float mPI = 3.141592653;
int sensorBiasConst = 200;
bool gyroBiasCalibrating = true;
bool accBiasCalibrating = true;

float accXCurr, accYCurr, accZCurr, accXPrev, accYPrev, accZPrev=1, accXBias, accYBias, accZBias;
float gyroXCurr, gyroYCurr, gyroZCurr, gyroXPrev, gyroYPrev, gyroZPrev, gyroXBias, gyroYBias, gyroZBias;
float accAngleX, accAngleY, accAngleZ, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float accErrorX, accErrorY, gyroErrorX, gyroErrorY, gyroErrorZ;
float dt, currentTime, previousTime;
PID pidAngleX, pidAngleY, pidAngleZ, pidVelZ;
float angleXError, angleYError, angleZError, velZError;
Kalman kalmanAngleX, kalmanAngleY, kalmanAngleZ;
float angleX, angleY, angleZ;
float outputAngleX, outputAngleY, outputAngleZ, outputVelZ;
Integrator velZ;
float motor1Speed, motor2Speed, motor3Speed, motor4Speed;
float input = 0;


int accBiasCount = 0;
int gyroBiasCount = 0;

int testVal = 0;

// SETPOINTS
float setRoll, setPitch, setYaw, setVelZ;


// ||||||||||| //
// || SETUP || //
// ||||||||||| //

void setup() {
  Serial.begin(9600);

  DroneState currentState = DroneState::CALIBRATING;
  CH5 CH5State = CH5::MID;

  SetupRemoteInput();

  SetupESC();

  SetupIMU();

  SetupPID();

  InitializeSetpoint();

}
// |||||||||| //
// || LOOP || //
// |||||||||| //

void loop() {

  CalcTime();
  
  DefineRemoteValues();

  GetSensorData();

  switch(currentState) {
    case DroneState::POWEROFF:
      SwitchOff();
      SendMotorSpeeds();
      exit(0);
      break;
    case DroneState::HOVER:
      GetKalmanAngles();
      CalcPID();
      break;
    case DroneState::SETDOWN:
      GetKalmanAngles();
      CalcPID();
      break;
    case DroneState::STANDBY:
      GetKalmanAngles();
      CalcPID();
      break;
    case DroneState::CALIBRATING:
      break;
  }

  SendMotorSpeeds();
  
  Serial.print("angle_x:");
  Serial.print(angleX);
  Serial.print(",");
  Serial.print("angle_y:");
  Serial.print(angleYInput);
  Serial.print(",");
  Serial.print("power:");
  Serial.println(powerInput);



}

// ||||||||||||||||||||| //
// || SETUP FUNCTIONS || //
// ||||||||||||||||||||| //

void SetupRemoteInput(){ //Set Remote Input
  Serial.println("|| Remote Setup ||");
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), StoreRemoteValues,  FALLING); // enabling interrupt at pin 2
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

void SetupESC(){
  // connect motors to digital 4,5,6,7 pins
  Serial.println("|| ESC Calibration ||");
  motor1.attach(4, 1000, 2000);
  motor2.attach(5, 1000, 2000);
  motor3.attach(6, 1000, 2000);
  motor4.attach(7, 1000, 2000);

  motor1.write(MAX_PULSE_LENGTH);
  motor2.write(MAX_PULSE_LENGTH);
  motor3.write(MAX_PULSE_LENGTH);
  motor4.write(MAX_PULSE_LENGTH);
  Serial.println("MAX MOTOR SPEED");

  delay(4000);

  motor1.write(MIN_PULSE_LENGTH);
  motor2.write(MIN_PULSE_LENGTH);
  motor3.write(MIN_PULSE_LENGTH);
  motor4.write(MIN_PULSE_LENGTH);
  Serial.println("MIN MOTOR SPEED");
  delay(4000);


}

void SetupIMU() {
  Serial.println("|| IMU Setup ||");
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
    Serial.println("|| PID Setup ||");
    pidAngleX = PID(0, 0.8, 0.1, 0.1);
    pidAngleY = PID(0, 0.8, 0.1, 0.1);
    pidAngleZ = PID(0, 0.8, 0.1, 0.1);

    pidVelZ = PID(0, 0.8, 0.1, 0.1);
} 

void InitializeSetpoint() {
  setRoll = 0; // degrees
  setYaw = 0; // degrees
  setPitch = 0; // degrees
  setVelZ = 0.5; // m/s
}

// |||||||||||||||||||| //
// || LOOP FUNCTIONS || //
// |||||||||||||||||||| //

void CalcTime() {
	previousTime = currentTime;        // previous time is stored before the actual time read
	currentTime = millis();            // current time actual time read
	dt = (currentTime - previousTime) / 1000; // convert to seconds
}


void DefineRemoteValues(){ //Get remote readings and output in degrees

  AssignRemoteValues();

  NormalizeRemoteValues();

  //InterpretRemoteValues();

  SetCurrentInputs();

  ReadPowerSwitchRemoteInput();

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

// void InterpretRemoteValues() {
//   float max_sensor_reading = 500;
//   float max_angle_output = 20; // degrees
//   float angle_const = max_angle_output / max_sensor_reading;

//   // Change reciever readings to degrees
//   // TODO: Make sure its a float
//   angleXInput = float(angleXInput * angle_const); // float() required for int division to work correctly
//   velZInput = float(velZInput * angle_const);
//   angleYInput = float(angleYInput * angle_const);
//   angleZInput = float(angleZInput * angle_const);
// }

// For Hover Startup
void SetCurrentInputs() {
  angleXInput = 0;
  angleYInput = 0;
  angleZInput = 0;

  // velZ has 4 seconds to lift off and then stops. Allows to give time for liftoff without tracking distance
  float msToElevate = 4000;
  velZInput = (setVelZ - (currentTime/msToElevate) * setVelZ) <= 0 ? 0 : (setVelZ - (currentTime/msToElevate) * setVelZ) ;
}

void ReadPowerSwitchRemoteInput() {
  if (powerInput < 50 && powerInput > -50) {
    CH5State = CH5::UP;
  } else if (powerInput < 550 && powerInput > 450) {
    CH5State = CH5::MID;
  } else { 
    CH5State = CH5::DOWN;
  }

  /*
   * DroneState { POWEROFF, HOVER, SETDOWN, CALIBRATING, STANDBY };
   * CH5 { UP, MID, DOWN };
   */
  
  /*
  calibrating -> standby    doneCalibrating() == true
  standby -> hover          mid
  hover -> powerOff         down
  hover -> setDown          up (need to pass elapsed time since standby)
  setDown -> powerOff       down
  setDown -> standby        4000ms pass
  */
  if (currentState == DroneState::CALIBRATING && doneCalibrating()) { // TODO: check doneCalibrating works
    currentState = DroneState::STANDBY;
  } else if (currentState == DroneState::STANDBY && CH5State == CH5::MID) {
    currentState = DroneState::HOVER;
  } else if (currentState == DroneState::HOVER && CH5State == CH5::DOWN) {
    currentState = DroneState::POWEROFF;
  } else if (currentState == DroneState::HOVER && CH5State == CH5::UP) {
    currentState = DroneState::SETDOWN;
  } else if (currentState == DroneState::SETDOWN && CH5State == CH5::DOWN) {
    currentState = DroneState::POWEROFF;
  } else if (currentState == DroneState::SETDOWN && setDownDone()) { // TODO: finish setDownDone() function
    currentState = DroneState::SETDOWN;
  } else {
    // catch all branch in case something goes wrong
    currentState = DroneState::POWEROFF;
  }
}

void GetSensorData() {
  GetAccData();
  GetGyroData();
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
    accZRaw -= (accZBias / sensorBiasConst) - 1; // TODO: Check if -1 is necessary

    // IIR Filter
    accXCurr = IIRFilter(accXPrev, accXRaw);
    accYCurr = IIRFilter(accYPrev, accYRaw);
    accZCurr = IIRFilter(accZPrev, accZRaw);


    // Accel Angle Calculations
    accAngleY = atan(-accXCurr/sqrt(accYCurr*accYCurr + accZCurr*accZCurr)) * 180 / mPI;
    accAngleX = atan(accYCurr/sqrt(accXCurr*accXCurr + accZCurr*accZCurr)) * 180 / mPI;

    // Calculate Vel Z
    if (abs(accZCurr - 1) < 0.01) { accZCurr = 1; }
    velZ.addValue(accZCurr - 1, dt);
  }
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

  gyroXCurr = IIRFilter(gyroXPrev, gyroXRaw); // This was not working for some reason so it is commented out
  gyroYCurr = IIRFilter(gyroYPrev, gyroYRaw);
  gyroZCurr = IIRFilter(gyroZPrev, gyroZRaw);

}

bool CheckCalibrating() {
  currentState = DroneState::STANDBY;
  return !accBiasCalibrating && !gyroBiasCalibrating;
}

/*
void SetupMotorVals() {
  const float kStartSpeed = 1070; // Test to find right value
  motor1Speed = kStartSpeed;
  motor2Speed = kStartSpeed;
  motor3Speed = kStartSpeed;
  motor4Speed = kStartSpeed;
}
*/

void GetKalmanAngles(){
  angleX = kalmanAngleX.getAngle(accAngleX, gyroXCurr, dt);
  angleY = kalmanAngleY.getAngle(accAngleY, gyroYCurr, dt);
}

void CalcPID() {
    FindErrors();

    pidAngleX.addValue(angleXError, dt);
    pidAngleY.addValue(angleYError, dt);
    pidAngleZ.addValue(angleZError, dt);
    pidVelZ.addValue(velZError, dt);

}

void FindErrors() {

  angleXError = angleXInput - angleX;
  angleYError = angleYInput - angleY;
  angleZError = angleZInput - angleZ;

  velZError = velZInput - velZ.getIntegral();

}

void SwitchOff() {
  const float kPWMOff = 1000;
  motor1Speed = kPWMOff;
  motor2Speed = kPWMOff;
  motor3Speed = kPWMOff;
  motor4Speed = kPWMOff;
}

void ScaleMotorSpeeds() {
  const float kMaxDegree = 20;
  const float kMaxZVelocity = 0.2;
  outputAngleX = pidAngleX.calcValue() * 500 / kMaxDegree;
  outputAngleY = pidAngleY.calcValue() * 500 / kMaxDegree;
  outputAngleZ = pidAngleZ.calcValue() * 500 / kMaxDegree;
  outputVelZ = velZ.getIntegral() * 500 / kMaxZVelocity;
}

void SetMotorSpeeds() { // STILL HAS TO BE CHANGED
  float velOutputConst = 2/5;
  float angleOutputConst = 1/5;

  motor1Speed += velOutputConst * outputVelZ + angleOutputConst*(outputAngleX - outputAngleY + outputAngleZ);
  motor2Speed += velOutputConst * outputVelZ - angleOutputConst*(outputAngleX - outputAngleY - outputAngleZ);
  motor3Speed += velOutputConst * outputVelZ + angleOutputConst*(outputAngleX + outputAngleY - outputAngleZ);
  motor4Speed += velOutputConst * outputVelZ - angleOutputConst*(outputAngleX + outputAngleY + outputAngleZ);

  CutoffMotorSpeeds(); // Make sure motor speeds are within cuttoff points
}

void SendMotorSpeeds() {
  // Two negative, two positive
  motor1.write(motor1Speed);
  motor2.write(motor2Speed);
  motor3.write(motor3Speed);
  motor4.write(motor4Speed);
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

// ||||||||||||||||||||| //
// || MISC FUNCTIONS || //
// ||||||||||||||||||||| //

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

float IIRFilter(float previous_value, float current_value) {
  float alpha = 0.2;
  float output = alpha*previous_value + (1-alpha)*current_value;
  return output;
}