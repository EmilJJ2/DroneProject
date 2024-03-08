#include "Integrator.h"

Integrator::Integrator() {
    lastVal = 0.0;
    integral = 0.0;
}

Integrator::Integrator(float startingVal) {
    lastVal = startingVal;
    // initialize integral to 0
    integral = 0.0;
}
// add value to integral and update lastVal
// timeStep is time since last call to addValue
void Integrator::addValue(float newVal, float timeStep) {
    newVal = (newVal) * 9.8;
    integral += timeStep * (lastVal + newVal)/2.0; // Mid Point Reimann Sum??
    lastVal = newVal;
}

void Integrator::addRotatedValue(float x, float y, float z, float pitch, float roll, float timeStep) {
    float PI = 3.1415926535897932384626433832795;
    pitch = pitch / 180 * PI;
    roll = roll / 180 * PI;
    
    float newVal = -x*sin(pitch) + y*cos(pitch)*sin(roll) + z*cos(pitch)*cos(roll);
    newVal = (newVal - 1.0) * 9.80665;
    integral += timeStep * (lastVal + newVal)/2.0; // Mid Point Reimann Sum??
    lastVal = newVal;
}
