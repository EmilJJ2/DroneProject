
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

// Have to produce rotation matrix to change desired velocity in Earth-Fixed Basis 
// to the basis of the orientation of the quadcopter. Same for angular

// Controller sends requested angle
// Uses angle to set motor power to values to ideally achieve this angle
// Sense motor power and use to create a model of the drone angle
// Take data from sensors and combine with model to find accurate drone angle
// Adjust motor power to change to these values