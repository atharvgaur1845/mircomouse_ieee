#ifndef LANE_CENTERING_H
#define LANE_CENTERING_H

#include "config.h"
#include <VL53L0X.h>  // Missing include

class LaneCentering {
private:
    VL53L0X& leftSensor;
    VL53L0X& rightSensor;
    const uint8_t samples = 5;
    
    int filteredReading(VL53L0X& sensor) {
        int sum = 0;
        for(int i=0; i < samples; i++) {  // Incomplete for loop
            sum += sensor.readRangeSingleMillimeters();
            delay(1);
        }
        return sum / samples;
    }
    
public:
    LaneCentering(VL53L0X& left, VL53L0X& right) : leftSensor(left), rightSensor(right) {}
    
    float calculateError() {
        int leftDist = filteredReading(leftSensor);
        int rightDist = filteredReading(rightSensor);
        
        // If both walls are detected
        if (leftDist < 200 && rightDist < 200) {
            // Return offset from center (positive means too far right)
            return (leftDist - rightDist);
        }
        // If only left wall
        else if (leftDist < 200) {
            // Try to maintain a fixed distance from left wall
            return (leftDist - 100) * 2;
        }
        // If only right wall
        else if (rightDist < 200) {
            // Try to maintain a fixed distance from right wall
            return (100 - rightDist) * 2;
        }
        
        // No walls detected, just go straight
        return 0;
    }
};

#endif // LANE_CENTERING_H
