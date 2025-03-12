#ifndef LANE_CENTERING_H
#define LANE_CENTERING_H

#include "config.h"
#include <VL53L0X.h>

class LaneCentering {
private:
    VL53L0X& leftSensor;
    VL53L0X& rightSensor;
    
    const uint8_t samples = 5;
    
    int filteredReading(VL53L0X& sensor) {
        int sum = 0;
        for(int i=0; i<samples; i++) {
            sum += sensor.readRangeSingleMillimeters();
            delay(1);
        }
        return sum/samples;
    }

public:
    LaneCentering(VL53L0X& left, VL53L0X& right) : 
        leftSensor(left), rightSensor(right) {}

    float calculateError() {
        int left = filteredReading(leftSensor);
        int right = filteredReading(rightSensor);
        
        if(left < cell_width && right < cell_width) {
            return (left - right) * tpmm;
        }
        if(left < cell_width) return (cell_width/2 - left) * 2;
        if(right < cell_width) return -(cell_width/2 - right) * 2;
        return 0;
    }
};

#endif
