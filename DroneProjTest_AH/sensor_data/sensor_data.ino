#include <Wire.h>

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
int accBiasCount = 0;
int gyroBiasCount = 0;
Integrator velZ;


float IIRFilter(float previous_value, float current_value) {
  float alpha = 0.2;
  float output = alpha*previous_value + (1-alpha)*current_value;
  return output;
}

// Everything in here just initiates information flow from the IMU to the Arduino
void setup() {
  Serial.begin(19200);
  SetupIMU();

}

void loop() {
  CalcTime();
  GetAccData();
  GetGyroData();

  Serial.print(accAngleX);
}

void CalcTime() {
	previousTime = currentTime;        // previous time is stored before the actual time read
	currentTime = millis();            // current time actual time read
	elapsedTime = (currentTime - previousTime) / 1000; // convert to seconds
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

  // Gyro Angle Calculations
  // Integrates the change in gyro angle over time || NOT ACCURATE
  // deg = deg + (deg/s)*s
  if (!gyroBiasCalibrating) {
    gyroAngleX = gyroAngleX + gyroXCurr*elapsedTime;  // Still drags a lot of error with it
    gyroAngleY = gyroAngleY + gyroYCurr*elapsedTime;
    gyroAngleZ = gyroAngleZ + gyroZCurr*elapsedTime;
  }

}