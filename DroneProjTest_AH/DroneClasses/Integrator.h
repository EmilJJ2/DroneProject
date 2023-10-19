#ifndef Integrator_h
#define Integrator_h
class Integrator {
    public:
        // Default constructor needed to allow for use as PID variable. I think...
        Integrator();

        Integrator(float startingVal);

        void addValue(float newVal, float timeStep);

        float getIntegral() { return integral; };
	private:
        // running integral
        float integral;
        // last value added to integral
        float lastVal;
};
#endif