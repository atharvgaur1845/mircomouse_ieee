#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "config.h"

class MotorController {
private:
    const int pwm, in1, in2;
    
public:
    MotorController(int pwmPin, int in1Pin, int in2Pin) : 
        pwm(pwmPin), in1(in1Pin), in2(in2Pin) {
        pinMode(pwm, OUTPUT);
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
    }

    void setPower(float power) {
        bool direction = power >= 0;
        power = constrain(fabs(power), 30, 255);
        
        digitalWrite(in1, direction ? HIGH : LOW);
        digitalWrite(in2, direction ? LOW : HIGH);
        analogWrite(pwm, power);
    }

    void brake() {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        analogWrite(pwm, 0);
    }
};

#endif
