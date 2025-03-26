// Floodfill.h - Header file for floodfill algorithm implementation
#ifndef FLOODFILL_H
#define FLOODFILL_H

#include <Arduino.h>
#include <VL53L0X.h>
#include "motor_cntrl.h"
#include "lanecentering.h"
#include "PIDcntrl.h"
#include <QueueArray.h> // Available in the Arduino library manager

// Maze size definition - standard micromouse maze is 16x16
#define MAZE_SIZE 16
#define GOAL_X 7  // Center X coordinate (0-indexed)
#define GOAL_Y 7  // Center Y coordinate (0-indexed)

// Define directions for clarity
enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

// States for the solving process
enum SolveState {
    EXPLORING,  // Initial exploration to find the goal
    RETURNING,  // Return to the start
    SPEEDRUN,   // Optimized run to the goal
    FINISHED    // Maze solved
};

// Simple cell coordinate structure
struct Cell {
    int x;
    int y;
    Cell(int x, int y) : x(x), y(y) {}
    Cell() : x(0), y(0) {}
};

class Floodfill {
public:
    // Constructor
    Floodfill(VL53L0X &left, VL53L0X &right, VL53L0X &front,
              MotorController &left_motor, MotorController &right_motor,
              LaneCentering &lane_system, PIDController &controller,
              volatile int &left_encoder, volatile int &right_encoder,
              float ticks_per_mm, int cell_width_mm) : 
        leftSensor(left), rightSensor(right), frontSensor(front),
        leftMotor(left_motor), rightMotor(right_motor),
        laneSystem(lane_system), pid(controller),
        posi_left(left_encoder), posi_right(right_encoder),
        tpmm(ticks_per_mm), cell_width(cell_width_mm) {}
    
    // Main functions
    void init();           // Initialize the solver
    void update();         // Update one step (call this in main loop)
    bool isFinished() { return state == FINISHED; }
    
private:
    // Hardware references
    VL53L0X &leftSensor;
    VL53L0X &rightSensor;
    VL53L0X &frontSensor;
    MotorController &leftMotor;
    MotorController &rightMotor;
    LaneCentering &laneSystem;
    PIDController &pid;
    volatile int &posi_left;
    volatile int &posi_right;
    
    // Constants
    const float tpmm;            // Ticks per millimeter for encoders
    const int cell_width;        // Cell width in mm
    const int WALL_THRESHOLD = 100;  // Distance in mm to detect a wall
    const int FORWARD_SPEED = 150;   // PWM for forward movement
    const int TURN_SPEED = 100;      // PWM for turning
    const float ROBOT_WIDTH = 150.0; // Robot width in mm (for turning calculations)
    
    // Speed run constants
    const int SPEED_RUN_FORWARD = 220;  // Faster speed for speed run
    const int SPEED_RUN_TURN = 150;     // Faster turning for speed run
    
    // Maze state
    uint8_t walls[MAZE_SIZE][MAZE_SIZE] = {0};  // Wall information (bit-packed: N=1, E=2, S=4, W=8)
    uint8_t visited[MAZE_SIZE][MAZE_SIZE] = {0}; // Visited cells
    uint16_t distance[MAZE_SIZE][MAZE_SIZE];    // Distance values from floodfill
    
    // Current robot state
    int currentX = 0;
    int currentY = 0;
    Direction currentDir = NORTH;
    SolveState state = EXPLORING;
    
    // Internal functions
    void updateWalls();
    void setWall(int x, int y, Direction dir);
    bool isWall(int x, int y, Direction dir);
    void floodFill(int goalX, int goalY);
    Direction chooseNextDirection();
    Direction chooseNextDirection(int x, int y);  // Overload for lookahead
    void moveForward();
    void turnLeft();
    void turnRight();
    void navigateNextCell();
    
    // Speed run optimized functions
    void navigateSpeedRun();          // Fast navigation for speed run
    int canMoveForwardMultipleCells(); // Check if we can move straight through multiple cells
    void moveForwardFast(int cells);  // Move multiple cells forward at high speed
    void turnLeftFast();              // Optimized left turn
    void turnRightFast();             // Optimized right turn
};

#endif // FLOODFILL_H
