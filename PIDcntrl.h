#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "config.h"

class PIDController {
private:
    float integral = 0;
    float prevError = 0;
    unsigned long prevTime = micros();

public:
    const float kp, ki, kd;

    PIDController(float p, float i, float d) : kp(p), ki(i), kd(d) {}

    float compute(float error) {
        unsigned long currTime = micros();
        float deltaT = (currTime - prevTime) / 1e6f;
        prevTime = currTime;
        
        integral += error * deltaT;
        float derivative = (error - prevError) / deltaT;
        prevError = error;
        
        return (kp * error) + (ki * integral) + (kd * derivative);
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }
};

#endif
