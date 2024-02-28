#ifndef Integrator_h
#define Integrator_h

#include <math.h>
class Integrator {
    public:
        // Default constructor needed to allow for use as PID variable. I think...
        Integrator();

        Integrator(float startingVal);

        void addValue(float newVal, float timeStep);
        
        void addRotatedValue(float x, float y, float z, float pitch, float roll, float timeStep);

        float getIntegral() { return integral; };


	private:
        // running integral
        float integral;
        // last value added to integral
        float lastVal;
};
#endif