#include <Wire.h>

const int MPU = 0x68; // MPU6050 I2C address

float AccX_curr, AccY_curr, AccZ_curr, AccX_prev, AccY_prev, AccZ_prev, AccX_bias, AccY_bias, AccZ_bias;
float GyroX_curr, GyroY_curr, GyroZ_curr, GyroX_prev, GyroY_prev, GyroZ_prev, GyroX_bias, GyroY_bias, GyroZ_bias;
float acc_Angle_X, acc_Angle_Y, gyro_Angle_X, gyro_Angle_Y, gyro_Angle_Z;
float roll, pitch, yaw;
float Acc_Error_X, Acc_Error_Y, Gyro_Error_X, Gyro_Error_Y, Gyro_Error_Z;
float elapsed_Time, current_Time, previous_Time;
int count = 0;


float IIRFilter(float previous_value, float current_value) {
  float alpha = 0.2;
  float output = alpha*previous_value + (1-alpha)*current_value;
  return output;
}

// Everything in here just initiates information flow from the IMU to the Arduino
void setup() {
  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register       
  Wire.endTransmission(true);        //end the transmission

  // Write to Gyro config
  Wire.beginTransmission(MPU);  
  Wire.write(0x1B);
  // 8 in Hex represents Bit3 and turns the gyro into 500 deg/s
  // 0 resets to 250 deg/s (I THINK)
  Wire.write(0x0); 
  Wire.endTransmission();

  // Write to Accel config
  Wire.beginTransmission(MPU);  
  Wire.write(0x1C);
  Wire.write(0x0); 
  Wire.endTransmission();


}

void loop() {
  GetSensorData();

}

void GetSensorData() {

  float AccX_raw, AccY_raw, AccZ_raw;
  float GyroX_raw, GyroY_raw, GyroZ_raw;
  float mPI = 3.141592653;
  float SSF_acc_val = 16384.0; // Sensitivity Sensor Factor for the accelerometer from the datasheet
  float SSF_gyro_val = 131; // Sensitivity Sensor Factor for the gyroscope from the datasheet
  int bias_const = 200;

  // Set previous values | This is to keep track of the values found in both the current loop and the last loop for the IIR Filter
  AccX_prev = AccX_curr;
  AccY_prev = AccY_curr;
  AccZ_prev = AccZ_curr;

  GyroX_prev = GyroX_curr;
  GyroY_prev = GyroY_curr;
  GyroZ_prev = GyroZ_curr;

  // Time Tracker | This is to keep track of time to integrate the Gyro output
  previous_Time = current_Time;        // Previous time is stored before the actual time read
  current_Time = millis();            // Current time actual time read
  elapsed_Time = (current_Time - previous_Time) / 1000; // Convert to seconds
  
  // Read accelerometer data | Accelerometer outputs in g units. So a reading of a means a*(9.8 m/s^2)
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX_raw = (Wire.read() << 8 | Wire.read()) / SSF_acc_val; // X-axis value
  AccY_raw = (Wire.read() << 8 | Wire.read()) / SSF_acc_val; // Y-axis value
  AccZ_raw = (Wire.read() << 8 | Wire.read()) / SSF_acc_val; // Z-axis value
  
  // Read gyro data | Gyro outputs in deg/s
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX_raw = (Wire.read() << 8 | Wire.read()) / SSF_gyro_val; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY_raw = (Wire.read() << 8 | Wire.read()) / SSF_gyro_val; // The higher the range, the less precise the returned degree value
  GyroZ_raw = (Wire.read() << 8 | Wire.read()) / SSF_gyro_val;
  
  // Bias removal [Must occur after RAW data retrieval and before RAW data is used]
  // This counts the first 200 data values and uses them to set the expected initial values
  if (count < bias_const) {
    // Finds Bias
    AccX_bias += AccX_raw;
    AccY_bias += AccY_raw;
    AccZ_bias += AccZ_raw;

    GyroX_bias += GyroX_raw;
    GyroY_bias += GyroY_raw;
    GyroZ_bias += GyroZ_raw;

    Serial.print("| CALIBRATING |");
    count++;
  } else {
    // Removes Bias
    AccX_raw -= (AccX_bias / bias_const);
    AccY_raw -= (AccY_bias / bias_const);
    AccZ_raw -= (AccZ_bias / bias_const) - 1;
    
    GyroX_raw -= (GyroX_bias / bias_const);
    GyroY_raw -= (GyroY_bias / bias_const); 
    GyroZ_raw -= (GyroZ_bias / bias_const);
    
  }

  // IIR Filter
  AccX_curr = IIRFilter(AccX_raw, AccX_prev);
  AccY_curr = IIRFilter(AccY_raw, AccY_prev);
  AccZ_curr = IIRFilter(AccZ_raw, AccZ_prev);
  
  GyroX_curr = IIRFilter(GyroX_raw, GyroX_prev); // This was not working for some reason so it is commented out
  GyroY_curr = IIRFilter(GyroY_raw, GyroY_prev);
  GyroZ_curr = IIRFilter(GyroZ_raw, GyroZ_prev);
  

  // Accel Angle Calculations
  acc_Angle_Y = atan(-AccX_curr/sqrt(AccY_curr*AccY_curr + AccZ_curr*AccZ_curr)) * 180 / mPI;
  acc_Angle_X = atan(AccY_curr/sqrt(AccX_curr*AccX_curr + AccZ_curr*AccZ_curr)) * 180 / mPI;

  // Gyro Angle Calculations
  // Integrates the change in gyro angle over time || NOT ACCURATE
  // deg = deg + (deg/s)*s
  gyro_Angle_X = gyro_Angle_X + GyroX_curr*elapsed_Time;  // Still drags a lot of error with it
  gyro_Angle_Y = gyro_Angle_Y + GyroY_curr*elapsed_Time;

  // COMPLIMENTARY FILTER | This is also not really working that well right now
  roll = 0.99 * gyro_Angle_X + 0.01 * acc_Angle_X;

}
