#include "PID.h"

// arguments: lastVal, curVal, timeStep
float getDerivative(float lastVal, float curVal, float timeStep) {
	return (curVal - lastVal) / timeStep;
}

PID::PID() {
    pVal = 0;
    lastError = 0;
    dVal = 0;
    pWeight, iWeight, dWeight = 0;
}

// arguments: startingError, pWeight, iWeight, dWeight
PID::PID(float startingError, float pw, float iw, float dw) {
    pVal = startingError;
    iVal = Integrator(startingError);
    lastError = startingError;
    dVal = 0;

    pWeight = pw;
    iWeight = iw;
    dWeight = dw;
}

// arguments: newError, timeStep
void PID::addValue(float newError, float timeStep) {
    pVal = newError;
    iVal.addValue(newError, timeStep);
    dVal = getDerivative(lastError, newError, timeStep);
    lastError = newError;
}

float PID::calcValue() {
 return (pVal * pWeight + iVal.getIntegral() * iWeight + dVal * dWeight);
}