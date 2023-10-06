#include <Servo.h>


Servo ESC1; 
Servo ESC2;
Servo ESC3;
Servo ESC4;    // create servo object to control the ESC
int temp;

void setup() {
  // Attach the ESC on pin 9
  // starts at 12 on 0 - 180 scale
  // 20 -> 193.548387 rpm w/ propellor
  ESC1.attach(8,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC2.attach(9,1000,2000);
  ESC3.attach(10,1000,2000);
  ESC4.attach(11,1000,2000);

  Serial.begin(9600);
  temp = 1;
  ESC1.write(20);
  ESC2.write(20);
  ESC3.write(20);
  ESC4.write(20);

  ESC1.write(0);
  ESC2.write(0);
  ESC3.write(0);
  ESC4.write(0);

  ESC1.arm();
}

void loop() {
  if (temp == 1){
    Serial.println("loop");
    
    /*
    for (int i = 0; i <= 20; i+= 2) {
      Serial.println(i);
      delay(500);
      ESC1.write(i);    // Send the signal to the ESC
      delay(500);
      ESC2.write(i);
      delay(500);
      ESC3.write(i);
      delay(500);
      ESC4.write(i);
      
    }
    */
    ESC2.write(10);
    ESC3.write(10);

  }

  delay(200);
  ++temp;

  if (temp == 20) { 
    ESC1.write(0); 
    ESC2.write(0); 
    ESC3.write(0); 
    ESC4.write(0);
    Serial.println("ENDED");
  }
}
