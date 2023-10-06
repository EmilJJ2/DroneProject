// || INCLUDES || //
#include <string>
// || SET GLOBAL VALUES || //
int pitch_input, roll_input, yaw_input, elevator_input, power_input; // Controller Inputs


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

// || BASE FUNCTIONS || //

void recieverSetup() {
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), read_me,  FALLING);
}

void readRecieverValues() {
  int ch1[15], ch[7],i;
  int pitch_bias, roll_input_bias, yaw_bias, elevator_bias;
  int deadzone = 8;
  int bias_const = 50; // I removed this because it didn't work. But integer division may have been the problem
  int count = 0;
  float max_sensor_reading = 500;
  float max_angle_output = 20; // degrees
  float angle_const = max_angle_output / max_sensor_reading;

  read_rc();

  // Working Values
  roll_input = ch[1];
  elevator_input = ch[2];
  pitch_input = ch[3];
  yaw_input = ch[4];

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
}


// || BASE HELPER FUNCTIONS || //

void  read_me()  {
  int  i,j,k=0;
  int* x = new int[15];
  unsigned long int a,b,c;
  //this code reads value from RC reciever from PPM pin (Pin 2 or  3)
  //this code gives channel values from 0-1000 values 
  a=micros(); //store time value a when pin value falling
  c=a-b;      //calculating  time inbetween two peaks
  b=a;        // 
  x[i]=c;     //storing 15 value in  array
  i=i+1;       if(i==15){for(int j=0;j<15;j++) {ch1[j]=x[j];} i=0;}//copy  store all values from temporary array another array after 15 reading 
  delete[] x;
}

int* read_rc(int ch1[], int x[]){
  int  i,j,k=0;
  int* ch = new int[15];
    for(k=14;k>-1;k--){if(ch1[k]>5000){j=k;}}  //detecting separation  space 10000us in that another array  | CHANGING IT TO 5000us FIXED IT FOR EMIL                   
    for(i=1;i<=6;i++){ch[i]=(ch1[i+j]-1000);} //assign 6 channel values after separation space
  return ch;
}     

// matrix value for rotation. make a 3 long matrix

class Tuple() { // Tuple to represent 3 values at a time
  public:
    float x;
    float y;
    float z;

    String toString() {
      return "[" x ", " y ", " z"]"
    }
}

class Tuple() { // Tuple to represent full state vector (I don't really think that this is needed)
  public:
    float x;
    float y;
    float z;
    float p;
    float r;
    float q;
    float dx;
    float dy;
    float dz;
    float dp;
    float dr;
    float dq;
}

int[] 


// Have to produce rotation matrix to change desired velocity in Earth-Fixed Basis 
// to the basis of the orientation of the quadcopter. Same for angular

// Controller sends requested angle
// Uses angle to set motor power to values to ideally achieve this angle
// Sense motor power and use to create a model of the drone angle
// Take data from sensors and combine with model to find accurate drone angle
// Adjust motor power to change to these values