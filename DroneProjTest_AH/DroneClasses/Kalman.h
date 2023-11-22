#ifndef KALMAN_h
#define KALMAN_h
class Kalman {
public:
    Kalman();
    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    float getAngle(float newAngle, float newRate, float dt);
    void setAngle(float newAngle) { angle = newAngle; }; // Used to set angle, this should be set as the starting angle
    float getRate() { return rate; }; // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    void setQangle(float newQ_angle) { Q_angle = newQ_angle; };
    void setQbias(float newQ_bias) { Q_bias = newQ_bias; };
    void setRmeasure(float newR_measure) { R_measure = newR_measure; };

    float getQangle() { return Q_angle; };
    float getQbias() { return Q_bias; };
    float getRmeasure() { return R_measure; };
    float getAngle() { return angle; };
private:
    /* Kalman filter variables */
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    float K[2]; // Kalman gain - This is a 2x1 vector
    float y; // Angle difference
    float S; // Estimate error
};

#endif