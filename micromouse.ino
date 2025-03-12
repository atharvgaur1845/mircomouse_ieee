#include "config.h"
#include "PIDController.h"
#include "LaneCentering.h"
#include "MotorController.h"
#include <VL53L0X.h>
#include <Wire.h>

// Hardware definitions
VL53L0X sensorLeft, sensorRight;
MotorController leftMotor(5, 30, 31);
MotorController rightMotor(3, 28, 29);
LaneCentering laneSystem(sensorLeft, sensorRight);
PIDController pid(2.5, 0.1, 0.5);

// Shared variables
volatile int posi_left = 0;
volatile int posi_right = 0;
float kp = 2.5, ki = 0.1, kd = 0.5;
float tpmm = 52.0;
const int cell_width = 180;

void setup() {
    Wire.begin();
    Serial.begin(115200);
    
    // Sensor initialization (from original code)
    pinMode(2, OUTPUT); digitalWrite(2, LOW);
    pinMode(3, OUTPUT); digitalWrite(3, LOW);
    pinMode(4, OUTPUT); digitalWrite(4, LOW);
    delay(10);
    
    digitalWrite(2, HIGH);
    sensorLeft.init();
    sensorLeft.setAddress(0x30);
    
    digitalWrite(3, HIGH);
    sensorRight.init();
    sensorRight.setAddress(0x31);
    
    // Encoder setup
    attachInterrupt(digitalPinToInterrupt(ENCA), []{
        CRITICAL_SECTION_START();
        posi_left += digitalRead(ENCB) ? -1 : 1;
        CRITICAL_SECTION_END();
    }, CHANGE);
    
    attachInterrupt(digitalPinToInterrupt(ENCD), []{
        CRITICAL_SECTION_START();
        posi_right += digitalRead(ENCD) ? 1 : -1;
        CRITICAL_SECTION_END();
    }, CHANGE);
}

void loop() {
    static unsigned long lastUpdate = 0;
    if(micros() - lastUpdate > 10000) { // 10ms control loop
        float error = laneSystem.calculateError();
        float correction = pid.compute(error);
        
        leftMotor.setPower(150 - correction);
        rightMotor.setPower(150 + correction);
        
        lastUpdate = micros();
    }
}
