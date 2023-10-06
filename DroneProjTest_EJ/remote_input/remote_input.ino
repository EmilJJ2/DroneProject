unsigned long int time_inbetween;
int pitch_input, roll_input, yaw_input, elevator_input, power_input;
bool power_switch = false;

int x[15], ppm[15], ch[7], store_x;
//Define Variables
//specifing  arrays and variables to store values 

void setup() {
  Serial.begin(19200);
  SetRemoteInput();
}

void loop() {
  GetRemoteValue();
  Serial.print("roll: ");Serial.print(roll_input);Serial.print("\	");
  Serial.print("elev: ");Serial.print(elevator_input);Serial.print("\	");
  Serial.print("pitch: ");Serial.print(pitch_input);Serial.print("\	");
  Serial.print("yaw: ");Serial.print(yaw_input);Serial.print("\	");
  Serial.print("Power: ");Serial.print(power_switch);Serial.println("\	");
}

void SetRemoteInput(){ //Set Remote Input
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), StoreValuesRemote,  FALLING);
  // enabling interrupt at pin 2
}

void GetRemoteValue(){ //Get remote readings

  //Define Variables
  int deadzone = 8;
  float max_sensor_reading = 500;
  float max_angle_output = 20; // degrees
  float angle_const = max_angle_output / max_sensor_reading;

  //Assign 6 channel values after separation space
  AssignValuesRemote();

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

void StoreValuesRemote(){ //Copy  store all values from temporary array another array after 15 reading  
  //this code reads value from RC reciever from PPM pin (Pin 2 or  3)
  //this code gives channel values from 0-1000 values
  unsigned int a, c;
  a = micros(); //store time value a when pin value falling
  c = a - time_inbetween;      //calculating  time inbetween two peaks
  time_inbetween = a;        // 
  x[store_x]=c;     //storing 15 value in  array
  store_x = store_x + 1;
  if(store_x==15){
    for(int j=0;j<15;j++){
      ppm[j]=x[j];
    }
    store_x = 0;
  }
}

void AssignValuesRemote(){ //Assign 6 channel values after separation space
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
