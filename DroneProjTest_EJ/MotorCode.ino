#include <Servo.h>


Servo ESC;     // create servo object to control the ESC
int temp;

void setup() {
  // Attach the ESC on pin 9
  // starts at 12 on 0 - 180 scale
  // 20 -> 193.548387 rpm w/ propellor
  ESC.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  Serial.begin(9600);
  temp = 1;
  ESC.write(20);
  Serial.println("20 :)");
  delay(10000);
  ESC.write(0);
}

void loop() {
  if (temp == 1){
    
    /*
    for (int i = 0; i <= 20; i+= 10) {
      ESC.write(i);    // Send the signal to the ESC
      delay(500);
      Serial.println(i);
    }
    */
    temp = 0;
  }
  
}