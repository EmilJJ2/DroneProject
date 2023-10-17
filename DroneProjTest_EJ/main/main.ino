// || INCLUDES || //
#include <Servo.h>
#include <Wire.h>

// || SET GLOBAL VALUES || //

#define MIN_PULSE_LENGTH 1000 // Minimum motor pulse length in µs
#define MID_PULSE_LENGTH 1500 // Middle motor pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum motor pulse length in µs

Servo motor1, motor2, motor3, motor4;
int pitch_input, roll_input, yaw_input, elevator_input, power_input; // Controller Inputs
bool power_switch = false;

unsigned long int remote_time_difference;
int strx[15], ppm[15], ch[7], store_x;

const int MPU = 0x68; // MPU6050 I2C address

float AccX_curr, AccY_curr, AccZ_curr, AccX_prev, AccY_prev, AccZ_prev, AccX_bias, AccY_bias, AccZ_bias;
float GyroX_curr, GyroY_curr, GyroZ_curr, GyroX_prev, GyroY_prev, GyroZ_prev, GyroX_bias, GyroY_bias, GyroZ_bias;
float acc_Angle_X, acc_Angle_Y, gyro_Angle_X, gyro_Angle_Y, gyro_Angle_Z;
float roll, pitch, yaw;
float Acc_Error_X, Acc_Error_Y, Gyro_Error_X, Gyro_Error_Y, Gyro_Error_Z;
float elapsed_Time, current_Time, previous_Time;
int count = 0;

// || SETUP || //

void setup() {
  SetupRemoteInput();

  SetupESC();

  SetupIMU();

}

// || LOOP || //

void loop() {
  DefineRemoteValues();

  GetSensorData();

}

// || BASE FUNCTIONS || //

void SetupRemoteInput(){ //Set Remote Input
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), StoreRemoteValues,  FALLING);
  // enabling interrupt at pin 2
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

void DefineRemoteValues(){ //Get remote readings and output in degrees

  //Define Variables
  int deadzone = 8;
  float max_sensor_reading = 500;
  float max_angle_output = 20; // degrees
  float angle_const = max_angle_output / max_sensor_reading;

  //Assign 6 channel values after separation space
  AssignRemoteValues();

  // Working Values
  roll_input = ch[1];
  elevator_input = ch[2];
  pitch_input = ch[3];
  yaw_input = ch[4];
  power_input = ch[5];

  // Set to base Value
  roll_input -= 500;
  elevator_input -= 500;
  pitch_input -= 500;
  yaw_input -= 500;

  // Deadzone
  if (roll_input >= -deadzone && roll_input <= deadzone) { roll_input = 0; }
  if (elevator_input >= -deadzone && elevator_input <= deadzone) { elevator_input = 0; }
  if (pitch_input >= -deadzone && pitch_input <= deadzone) { pitch_input = 0; }
  if (yaw_input >= -deadzone && yaw_input <= deadzone) { yaw_input = 0; }

  // Change reciever readings to degrees
  roll_input = float(roll_input * angle_const); // float() required for int division to work correctly
  elevator_input = float(elevator_input * angle_const);
  pitch_input = float(pitch_input * angle_const);
  yaw_input = float(yaw_input * angle_const);

  // Power Switch
  if (power_input < 50 && power_input > -50) {
    power_switch = true; // Up on the ch5 switch sends a signal around 0, which I set to be on
  } else { power_switch = false; } // If the switch is anything but up, or not working, power is off
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

// || BASE HELPER FUNCTIONS || //

void StoreRemoteValues(){ //Store all values from temporary array
  //this code reads value from RC reciever from PPM pin (Pin 2 or  3)
  //this code gives channel values from 0-1000 values
  unsigned int a, c;
  a = micros(); //store time value a when pin value falling
  c = a - remote_time_difference;      //calculating  time inbetween two peaks
  remote_time_difference = a;        // 
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
