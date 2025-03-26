#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

class MotorController {
private:
    int pwmPin;
    int in1Pin;
    int in2Pin;
    float currentPower;

public:
    MotorController(int pwm, int in1, int in2) : pwmPin(pwm), in1Pin(in1), in2Pin(in2), currentPower(0) {
        pinMode(pwmPin, OUTPUT);
        pinMode(in1Pin, OUTPUT);
        pinMode(in2Pin, OUTPUT);
    }

    void setPower(float power) {
        currentPower = power;
        if (power > 0) {
            digitalWrite(in1Pin, HIGH);
            digitalWrite(in2Pin, LOW);
        } else if (power < 0) {
            digitalWrite(in1Pin, LOW);
            digitalWrite(in2Pin, HIGH);
        } else {
            digitalWrite(in1Pin, LOW);
            digitalWrite(in2Pin, LOW);
        }
        analogWrite(pwmPin, abs(power) * 255);
    }

    float getCurrentPower() {
        return currentPower;
    }

    void stop() {
        setPower(0);
    }
};

#endif // MOTOR_CONTROLLER_H
