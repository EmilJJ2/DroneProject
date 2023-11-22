#include <Wire.h>

int angleYInput, angleXInput, angleZInput, velZInput, powerInput; // Controller Inputs
bool powerSwitch = false;
unsigned long int remoteTimeDifference;
int strx[15], ppm[15], ch[7], store_x;
float elapsedTime, currentTime, previousTime;
float input = 0;
bool startup = true;
int accBiasCount = 0;
int gyroBiasCount = 0;

void setup() {
  Serial.begin(19200);
  setupRemoteInput();
}

void loop() {
  defineRemoteValues();
  /*
  Serial.print(angleYInput);
  Serial.print(" | ");
  Serial.print(angleXInput);
  Serial.print(" | ");
  Serial.println(angleZInput);
  */
}

void setupRemoteInput() { // set remote input
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), storeRemoteValues,  FALLING); // enabling interrupt at pin 2
}

void defineRemoteValues() { // get remote readings and output in degrees
  // assign 6 channel values after separation space
  assignRemoteValues();
  normalizeRemoteValues();
  interpretRemoteValues();
  readPowerSwitchRemoteInput();
}

void normalizeRemoteValues() {

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

  Serial.print(ch[1]);Serial.print("\	");
  Serial.print(ch[2]);Serial.print("\	");
  Serial.print(ch[3]);Serial.print("\	");
  Serial.print(ch[4]);Serial.print("\	");
  Serial.print(ch[5]);Serial.print("\	");
  Serial.println(ch[6]);

  // deadzone
  if (angleXInput >= -deadzone && angleXInput <= deadzone) { angleXInput = 0; }
  if (velZInput >= -deadzone && velZInput <= deadzone) { velZInput = 0; }
  if (angleYInput >= -deadzone && angleYInput <= deadzone) { angleYInput = 0; }
  if (angleZInput >= -deadzone && angleZInput <= deadzone) { angleZInput = 0; }
}

void interpretRemoteValues() {
  float max_sensor_reading = 500;
  float max_angle_output = 20; // degrees
  float angle_const = max_angle_output / max_sensor_reading;

  // change receiver readings to degrees
  angleXInput = float(angleXInput * angle_const); // float() required for int division to work correctly
  velZInput = float(velZInput * angle_const);
  angleYInput = float(angleYInput * angle_const);
  angleZInput = float(angleZInput * angle_const);
}

void readPowerSwitchRemoteInput() {
  if (powerInput < 50 && powerInput > -50) {
    powerSwitch = true; // Up on the ch5 switch sends a signal around 0, which I set to be on
  } else { powerSwitch = false; } // If the switch is anything but up, or not working, power is off
}

void calcTime() {
	previousTime = currentTime;        // previous time is stored before the actual time read
	currentTime = millis();            // current time actual time read
	elapsedTime = (currentTime - previousTime) / 1000; // convert to seconds
}

void storeRemoteValues(){ //Store all values from temporary array
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

void assignRemoteValues(){ //Take stored remote values and output to loop
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