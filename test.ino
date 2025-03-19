 /*
 * MicroMouse.ino - Main sketch for Teensy 4.1 micromouse using floodfill algorithm
 * This file integrates the VL53L0X sensors, motor control, and floodfill navigation
 */

#include "config.h"
#include "PIDController.h"
#include "LaneCentering.h"
#include "MotorController.h"
#include "Floodfill.h"
#include <VL53L0X.h>
#include <Wire.h>
#include <QueueArray.h>

// Hardware setup
VL53L0X sensorLeft, sensorRight, sensorFront;
MotorController leftMotor(5, 30, 31);  // Using your pin configuration
MotorController rightMotor(3, 28, 29);
LaneCentering laneSystem(sensorLeft, sensorRight);
PIDController pid(2.5, 0.1, 0.5);      // Using your PID values

// Shared variables
volatile int posi_left = 0;
volatile int posi_right = 0;
float tpmm = 52.0;                     // Ticks per mm - calibrate for your encoders
const int cell_width = 180;            // Cell width in mm

// Create floodfill solver
Floodfill mazeSolver(
    sensorLeft, sensorRight, sensorFront,
    leftMotor, rightMotor, 
    laneSystem, pid,
    posi_left, posi_right,
    tpmm, cell_width
);

// Optional: Add button to start the maze solving
const int START_BUTTON_PIN = 12;   // Change to your button pin
bool mazeRunning = false;

// LED indicator for different states
const int STATUS_LED_PIN = 13;     // Built-in LED on most boards

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    
    // Initialize I2C bus
    Wire.begin();
    
    // Setup status LED
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    
    // Setup VL53L0X sensors with separate reset pins
    Serial.println("Initializing sensors...");
    
    // Reset all sensors to ensure clean startup
    pinMode(2, OUTPUT); digitalWrite(2, LOW);  // Left sensor
    pinMode(3, OUTPUT); digitalWrite(3, LOW);  // Right sensor 
    pinMode(4, OUTPUT); digitalWrite(4, LOW);  // Front sensor (add this sensor)
    delay(10);
    
    // Initialize left sensor with unique address
    digitalWrite(2, HIGH);
    delay(10);
    sensorLeft.init();
    sensorLeft.setAddress(0x30);
    sensorLeft.setTimeout(500);
    
    // Initialize right sensor with unique address
    digitalWrite(3, HIGH);
    delay(10);
    sensorRight.init();
    sensorRight.setAddress(0x31);
    sensorRight.setTimeout(500);
    
    // Initialize front sensor with unique address
    digitalWrite(4, HIGH);
    delay(10);
    sensorFront.init();
    sensorFront.setAddress(0x32);
    sensorFront.setTimeout(500);
    
    // Set up timing budget (33ms default, but can adjust for speed/accuracy tradeoff)
    sensorLeft.setMeasurementTimingBudget(20000);   // 20ms for faster readings
    sensorRight.setMeasurementTimingBudget(20000);  // 20ms for faster readings
    sensorFront.setMeasurementTimingBudget(20000);  // 20ms for faster readings
    
    // Set up continuous reading mode for faster response
    sensorLeft.startContinuous();
    sensorRight.startContinuous();
    sensorFront.startContinuous();
    
    // Configure encoder interrupts
    attachInterrupt(digitalPinToInterrupt(ENCA), []{
        CRITICAL_SECTION_START();
        posi_left += digitalRead(ENCB) ? -1 : 1;
        CRITICAL_SECTION_END();
    }, CHANGE);
    
    attachInterrupt(digitalPinToInterrupt(ENCC), []{
        CRITICAL_SECTION_START();
        posi_right += digitalRead(ENCD) ? 1 : -1;
        CRITICAL_SECTION_END();
    }, CHANGE);
    
    // Initialize start button
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    
    // Initialize the floodfill solver
    mazeSolver.init();
    
    // Indicate successful initialization with LED pattern
    for (int i = 0; i < 3; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
    }
    
    Serial.println("Initialization complete!");
    Serial.println("Press the start button to begin maze solving.");
}

void loop() {
    // Check for start button press
    if (!mazeRunning && digitalRead(START_BUTTON_PIN) == LOW) {
        delay(200);  // Simple debounce
        if (digitalRead(START_BUTTON_PIN) == LOW) {
            Serial.println("Starting maze run!");
            mazeRunning = true;
            
            // Initial LED indication that we're starting
            digitalWrite(STATUS_LED_PIN, HIGH);
            delay(500);
            digitalWrite(STATUS_LED_PIN, LOW);
            delay(500);
        }
    }
    
    // Maze solving mode
    if (mazeRunning && !mazeSolver.isFinished()) {
        // LED blinking pattern depends on state
        static unsigned long lastBlink = 0;
        if (millis() - lastBlink > 500) {
            digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
            lastBlink = millis();
        }
        
        // Update the maze solver (one step of the algorithm)
        mazeSolver.update();
        
        // Small delay between updates for mapping phase
        // Speed run phase handles its own timing in navigateSpeedRun()
        delay(20);
    }
    
    // If maze is finished, rapid LED blinking
    if (mazeSolver.isFinished()) {
        static unsigned long lastFinishBlink = 0;
        if (millis() - lastFinishBlink > 100) {
            digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
            lastFinishBlink = millis();
        }
    }
    
    // Manual control mode when not solving maze
    if (!mazeRunning) {
        // Process any serial commands for manual testing
        if (Serial.available()) {
            char cmd = Serial.read();
            
            switch (cmd) {
                case 'f':  // Forward
                    leftMotor.setPower(150);
                    rightMotor.setPower(150);
                    break;
                    
                case 'b':  // Backward
                    leftMotor.setPower(-150);
                    rightMotor.setPower(-150);
                    break;
                    
                case 'l':  // Left
                    leftMotor.setPower(-100);
                    rightMotor.setPower(100);
                    break;
                    
                case 'r':  // Right
                    leftMotor.setPower(100);
                    rightMotor.setPower(-100);
                    break;
                    
                case 's':  // Stop
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    break;
                    
                case 'd':  // Distance readings
                    Serial.print("Left: ");
                    Serial.print(sensorLeft.readRangeContinuousMillimeters());
                    Serial.print("mm, Right: ");
                    Serial.print(sensorRight.readRangeContinuousMillimeters());
                    Serial.print("mm, Front: ");
                    Serial.print(sensorFront.readRangeContinuousMillimeters());
                    Serial.println("mm");
                    break;
                    
                case 'g':  // Go - start maze solving
                    mazeRunning = true;
                    Serial.println("Starting maze run!");
                    break;
            }
        }
        
        // Simple lane following when not actively solving
        static unsigned long lastUpdate = 0;
        if (micros() - lastUpdate > 10000) { // 10ms control loop
            // Use lane centering for basic wall following
            float error = laneSystem.calculateError();
            float correction = pid.compute(error);
            
            // Apply correction if motors are running
            if (leftMotor.getPower() != 0 || rightMotor.getPower() != 0) {
                int baseSpeed = 150;
                leftMotor.setPower(baseSpeed - correction);
                rightMotor.setPower(baseSpeed + correction);
            }
            
            lastUpdate = micros();
        }
    }
}
