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
    integral += timeStep * (lastVal + newVal)/2; // trapezoidal integral (riemann sum)
    lastVal = newVal;
}