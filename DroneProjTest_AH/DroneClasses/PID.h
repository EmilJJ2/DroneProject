#ifndef PID_h
#define PID_h
#include "Integrator.h"
class PID {
    public:
        PID();

        PID(float startingVal, float pw, float iw, float dw);

        void addValue(float newVal, float timeStep);

        float calcValue();

    private:
        float pVal, dVal, pWeight, iWeight, dWeight, lastError;
        Integrator iVal;
};

#endif