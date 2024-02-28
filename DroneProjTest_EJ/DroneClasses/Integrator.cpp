#include "Integrator.h"

Integrator::Integrator() {
    lastVal = 0;
    integral = 0;
}

Integrator::Integrator(float startingVal) {
    lastVal = startingVal;
    // initialize integral to 0
    integral = 0;
}
// add value to integral and update lastVal
// timeStep is time since last call to addValue
void Integrator::addValue(float newVal, float timeStep) {
    integral += timeStep * (lastVal + newVal)/2; // Mid Point Reimann Sum??
    lastVal = newVal;
}

void Integrator::addRotatedValue(float x, float y, float z, float pitch, float roll, float timeStep) {
    float newVal = -x*sin(pitch) + y*cos(pitch)*sin(roll) + z*cos(pitch)*cos(roll);
    
    integral += timeStep * (lastVal + newVal)/2; // Mid Point Reimann Sum??
    lastVal = newVal;
}
