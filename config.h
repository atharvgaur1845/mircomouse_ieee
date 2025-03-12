#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <VL53L0X.h>

// Encoder positions
extern volatile int posi_left;
extern volatile int posi_right;

// PID Constants
extern float kp, ki, kd;
extern float kp_center, ki_center, kd_center;
extern float tpmm;

// Motor Parameters
extern const int base_speed;
extern const int cell_width;

#endif
