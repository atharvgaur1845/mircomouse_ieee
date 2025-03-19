// Floodfill.cpp - Implementation of the floodfill algorithm for maze solving

#include "Floodfill.h"
#include <Arduino.h>

// Initialize maze data structures and starting values
void Floodfill::init() {
    // Initialize maze walls - only outer walls are known initially
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            walls[x][y] = 0;
            visited[x][y] = 0;
            distance[x][y] = 65535; // High initial value
        }
    }
    
    // Set outer walls of the maze
    for (int x = 0; x < MAZE_SIZE; x++) {
        setWall(x, 0, SOUTH);
        setWall(x, MAZE_SIZE-1, NORTH);
    }
    for (int y = 0; y < MAZE_SIZE; y++) {
        setWall(0, y, WEST);
        setWall(MAZE_SIZE-1, y, EAST);
    }
    
    // Set initial position and direction
    currentX = 0;
    currentY = 0;
    currentDir = NORTH;
    state = EXPLORING;
    
    // Initial flood fill from goal
    floodFill(GOAL_X, GOAL_Y);
}

// Detect walls around current cell using sensors
void Floodfill::updateWalls() {
    // Mark current cell as visited
    visited[currentX][currentY] = 1;
    
    // Read all distance sensors
    int frontDist = frontSensor.readRangeSingleMillimeters();
    int leftDist = leftSensor.readRangeSingleMillimeters();
    int rightDist = rightSensor.readRangeSingleMillimeters();
    
    // Add walls where detected
    if (frontDist < WALL_THRESHOLD) {
        setWall(currentX, currentY, currentDir);
    }
    
    if (leftDist < WALL_THRESHOLD) {
        Direction leftDir = (Direction)((currentDir + 3) % 4); // Left is -90° from current
        setWall(currentX, currentY, leftDir);
    }
    
    if (rightDist < WALL_THRESHOLD) {
        Direction rightDir = (Direction)((currentDir + 1) % 4); // Right is +90° from current
        setWall(currentX, currentY, rightDir);
    }
    
    // Debug output - show current position and detected walls
    Serial.print("Position (");
    Serial.print(currentX);
    Serial.print(",");
    Serial.print(currentY);
    Serial.print(") Walls: ");
    if (isWall(currentX, currentY, NORTH)) Serial.print("N ");
    if (isWall(currentX, currentY, EAST)) Serial.print("E ");
    if (isWall(currentX, currentY, SOUTH)) Serial.print("S ");
    if (isWall(currentX, currentY, WEST)) Serial.print("W ");
    Serial.println();
}

// Mark a wall in the internal maze representation
void Floodfill::setWall(int x, int y, Direction dir) {
    if (x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE) return;
    
    // Set wall for current cell
    walls[x][y] |= (1 << dir);
    
    // Set corresponding wall for adjacent cell
    int newX = x, newY = y;
    Direction oppositeDir;
    
    switch (dir) {
        case NORTH: 
            newY = y + 1; 
            oppositeDir = SOUTH; 
            break;
        case EAST:  
            newX = x + 1; 
            oppositeDir = WEST; 
            break;
        case SOUTH: 
            newY = y - 1; 
            oppositeDir = NORTH; 
            break;
        case WEST:  
            newX = x - 1; 
            oppositeDir = EAST; 
            break;
    }
    
    // Set the opposite wall in the adjacent cell if it's within bounds
    if (newX >= 0 && newY >= 0 && newX < MAZE_SIZE && newY < MAZE_SIZE) {
        walls[newX][newY] |= (1 << oppositeDir);
    }
}

// Check if a wall exists at a specific location and direction
bool Floodfill::isWall(int x, int y, Direction dir) {
    if (x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE) return true;
    return (walls[x][y] & (1 << dir)) != 0;
}

// Core floodfill algorithm
void Floodfill::floodFill(int goalX, int goalY) {
    // Reset all distances to a high value
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            distance[x][y] = 65535;
        }
    }
    
    // Use queue for breadth-first search
    QueueArray<Cell> queue;
    
    // Start from the goal
    distance[goalX][goalY] = 0;
    queue.push(Cell(goalX, goalY));
    
    // Breadth-first fill of distances
    while (!queue.isEmpty()) {
        Cell current = queue.front();
        queue.pop();
        
        // Check all four neighboring cells
        for (int dir = 0; dir < 4; dir++) {
            // Skip if there's a wall in this direction
            if (isWall(current.x, current.y, (Direction)dir)) continue;
            
            // Calculate neighbor coordinates
            int nx = current.x, ny = current.y;
            switch (dir) {
                case NORTH: ny++; break;
                case EAST:  nx++; break;
                case SOUTH: ny--; break;
                case WEST:  nx--; break;
            }
            
            // If this is a better path, update distance and queue cell
            if (nx >= 0 && ny >= 0 && nx < MAZE_SIZE && ny < MAZE_SIZE &&
                distance[nx][ny] > distance[current.x][current.y] + 1) {
                
                distance[nx][ny] = distance[current.x][current.y] + 1;
                queue.push(Cell(nx, ny));
            }
        }
    }
}

// Determine best next direction to move based on floodfill values
Direction Floodfill::chooseNextDirection() {
    return chooseNextDirection(currentX, currentY);
}

// Overloaded version for look-ahead path planning
Direction Floodfill::chooseNextDirection(int x, int y) {
    int minDist = 65535; // High initial value
    Direction bestDir = currentDir; // Default to current direction
    
    // Check all four possible directions
    for (int dir = 0; dir < 4; dir++) {
        // Skip if there's a wall
        if (isWall(x, y, (Direction)dir)) continue;
        
        // Calculate the coordinates in this direction
        int nx = x, ny = y;
        switch (dir) {
            case NORTH: ny++; break;
            case EAST:  nx++; break;
            case SOUTH: ny--; break;
            case WEST:  nx--; break;
        }
        
        // If this cell has a lower distance, it's on the path to the goal
        if (nx >= 0 && ny >= 0 && nx < MAZE_SIZE && ny < MAZE_SIZE && 
            distance[nx][ny] < minDist) {
            minDist = distance[nx][ny];
            bestDir = (Direction)dir;
        }
    }
    
    return bestDir;
}

// Move one step towards the goal
void Floodfill::moveForward() {
    // Calculate target ticks for one cell forward movement
    int targetTicks = cell_width * tpmm;
    
    // Reset encoder values
    posi_left = 0;
    posi_right = 0;
    
    // Drive forward while using PID for lane centering
    while ((posi_left + posi_right) / 2 < targetTicks) {
        float error = laneSystem.calculateError();
        float correction = pid.compute(error);
        
        // Apply motor power with PID correction for straight movement
        leftMotor.setPower(FORWARD_SPEED - correction);
        rightMotor.setPower(FORWARD_SPEED + correction);
        delay(10);
    }
    
    // Stop motors
    leftMotor.setPower(0);
    rightMotor.setPower(0);
    
    // Update position based on direction
    switch (currentDir) {
        case NORTH: currentY++; break;
        case EAST:  currentX++; break;
        case SOUTH: currentY--; break;
        case WEST:  currentX--; break;
    }
}

// Execute a left turn
void Floodfill::turnLeft() {
    // Reset encoder counts
    posi_left = 0;
    posi_right = 0;
    
    // Calculate required encoder ticks for a 90-degree turn
    int turnTicks = (PI * ROBOT_WIDTH * tpmm) / 4; // Quarter circle
    
    // Execute turn by running motors in opposite directions
    while (abs(posi_right) < turnTicks) {
        leftMotor.setPower(-TURN_SPEED);
        rightMotor.setPower(TURN_SPEED);
        delay(10);
    }
    
    // Stop motors
    leftMotor.setPower(0);
    rightMotor.setPower(0);
    
    // Update direction (turning left reduces direction value by 1)
    currentDir = (Direction)((currentDir + 3) % 4);
}

// Execute a right turn
void Floodfill::turnRight() {
    // Reset encoder counts
    posi_left = 0;
    posi_right = 0;
    
    // Calculate required encoder ticks for a 90-degree turn
    int turnTicks = (PI * ROBOT_WIDTH * tpmm) / 4; // Quarter circle
    
    // Execute turn by running motors in opposite directions
    while (abs(posi_left) < turnTicks) {
        leftMotor.setPower(TURN_SPEED);
        rightMotor.setPower(-TURN_SPEED);
        delay(10);
    }
    
    // Stop motors
    leftMotor.setPower(0);
    rightMotor.setPower(0);
    
    // Update direction (turning right increases direction value by 1)
    currentDir = (Direction)((currentDir + 1) % 4);
}

// Navigate to next cell during exploration/mapping
void Floodfill::navigateNextCell() {
    Direction nextDir = chooseNextDirection();
    
    // Calculate turn needed
    int dirDiff = (nextDir - currentDir + 4) % 4;
    
    // Execute appropriate turn
    switch (dirDiff) {
        case 0: // No turn needed
            break;
        case 1: // Turn right
            turnRight();
            break;
        case 2: // Turn around (two right turns)
            turnRight();
            turnRight();
            break;
        case 3: // Turn left
            turnLeft();
            break;
    }
    
    // Move to next cell
    moveForward();
}

// Check how many cells we can move forward in a straight line
int Floodfill::canMoveForwardMultipleCells() {
    int x = currentX;
    int y = currentY;
    Direction dir = currentDir;
    int count = 0;
    
    // Look ahead up to 3 cells
    for (int i = 0; i < 3; i++) {
        // Check if there's a wall ahead
        if (isWall(x, y, dir)) break;
        
        // Move to next cell in current direction
        switch (dir) {
            case NORTH: y++; break;
            case EAST:  x++; break;
            case SOUTH: y--; break;
            case WEST:  x--; break;
        }
        
        count++;
        
        // Check if we need to turn at the next cell
        Direction nextDir = chooseNextDirection(x, y);
        if (nextDir != dir) break;
    }
    
    return count;
}

// Move multiple cells forward at high speed
void Floodfill::moveForwardFast(int cells) {
    // Calculate target ticks for multiple cells
    int targetTicks = cells * cell_width * tpmm;
    
    // Reset encoder values
    posi_left = 0;
    posi_right = 0;
    
    // Acceleration phase
    for (int speed = FORWARD_SPEED; speed < SPEED_RUN_FORWARD; speed += 10) {
        float error = laneSystem.calculateError();
        float correction = pid.compute(error);
        
        leftMotor.setPower(speed - correction);
        rightMotor.setPower(speed + correction);
        delay(5);
    }
    
    // High-speed phase - move at top speed for most of the distance
    while ((posi_left + posi_right) / 2 < targetTicks * 0.7) {
        float error = laneSystem.calculateError();
        float correction = pid.compute(error);
        
        leftMotor.setPower(SPEED_RUN_FORWARD - correction);
        rightMotor.setPower(SPEED_RUN_FORWARD + correction);
        delay(2); // Faster response time
    }
    
    // Deceleration phase
    for (int speed = SPEED_RUN_FORWARD; speed > FORWARD_SPEED; speed -= 10) {
        float error = laneSystem.calculateError();
        float correction = pid.compute(error);
        
        leftMotor.setPower(speed - correction);
        rightMotor.setPower(speed + correction);
        delay(5);
    }
    
    // Final approach at regular speed
    while ((posi_left + posi_right) / 2 < targetTicks) {
        float error = laneSystem.calculateError();
        float correction = pid.compute(error);
        
        leftMotor.setPower(FORWARD_SPEED - correction);
        rightMotor.setPower(FORWARD_SPEED + correction);
        delay(2);
    }
    
    // Graceful slow-down but don't fully stop
    leftMotor.setPower(FORWARD_SPEED * 0.6);
    rightMotor.setPower(FORWARD_SPEED * 0.6);
    
    // Update position based on cells moved
    for (int i = 0; i < cells; i++) {
        switch (currentDir) {
            case NORTH: currentY++; break;
            case EAST:  currentX++; break;
            case SOUTH: currentY--; break;
            case WEST:  currentX--; break;
        }
    }
}

// Optimized fast left turn for speed runs
void Floodfill::turnLeftFast() {
    // Reset encoder counts
    posi_left = 0;
    posi_right = 0;
    
    // Calculate required encoder ticks for a 90-degree turn
    int turnTicks = (PI * ROBOT_WIDTH * tpmm) / 4; // Quarter circle
    
    // Execute turn by running motors in opposite directions at higher speed
    while (abs(posi_right) < turnTicks * 0.7) {
        leftMotor.setPower(-SPEED_RUN_TURN);
        rightMotor.setPower(SPEED_RUN_TURN);
        delay(2); // Faster response time
    }
    
    // Slow down for precise alignment
    while (abs(posi_right) < turnTicks) {
        leftMotor.setPower(-TURN_SPEED * 0.7);
        rightMotor.setPower(TURN_SPEED * 0.7);
        delay(2);
    }
    
    // Don't stop completely - maintain momentum
    leftMotor.setPower(FORWARD_SPEED * 0.6);
    rightMotor.setPower(FORWARD_SPEED * 0.6);
    
    // Update direction (turning left reduces direction value by 1)
    currentDir = (Direction)((currentDir + 3) % 4);
}

// Optimized fast right turn for speed runs
void Floodfill::turnRightFast() {
    // Reset encoder counts
    posi_left = 0;
    posi_right = 0;
    
    // Calculate required encoder ticks for a 90-degree turn
    int turnTicks = (PI * ROBOT_WIDTH * tpmm) / 4; // Quarter circle
    
    // Execute turn by running motors in opposite directions at higher speed
    while (abs(posi_left) < turnTicks * 0.7) {
        leftMotor.setPower(SPEED_RUN_TURN);
        rightMotor.setPower(-SPEED_RUN_TURN);
        delay(2); // Faster response time
    }
    
    // Slow down for precise alignment
    while (abs(posi_left) < turnTicks) {
        leftMotor.setPower(TURN_SPEED * 0.7);
        rightMotor.setPower(-TURN_SPEED * 0.7);
        delay(2);
    }
    
    // Don't stop completely - maintain momentum
    leftMotor.setPower(FORWARD_SPEED * 0.6);
    rightMotor.setPower(FORWARD_SPEED * 0.6);
    
    // Update direction (turning right increases direction value by 1)
    currentDir = (Direction)((currentDir + 1) % 4);
}

// Optimized navigation for speed runs
void Floodfill::navigateSpeedRun() {
    // Look ahead to optimize path
    Direction nextDir = chooseNextDirection();
    
    // Check if we can move straight through multiple cells
    int straightCells = 0;
    if (nextDir == currentDir) {
        straightCells = canMoveForwardMultipleCells();
    }
    
    if (straightCells > 1) {
        // We can move straight through multiple cells at high speed
        moveForwardFast(straightCells);
    } else {
        // Calculate turn needed
        int dirDiff = (nextDir - currentDir + 4) % 4;
        
        // Execute appropriate turn with optimized speed run turns
        switch (dirDiff) {
            case 0: // No turn needed
                // Just move forward at high speed
                moveForwardFast(1);
                break;
                
            case 1: // Turn right
                // Start turning while still moving forward for smooth transition
                leftMotor.setPower(FORWARD_SPEED * 0.8);
                rightMotor.setPower(FORWARD_SPEED * 0.3);
                delay(50);
                
                // Complete the turn with optimized fast turn
                turnRightFast();
                
                // Continue forward without stopping
                moveForwardFast(1);
                break;
                
            case 2: // Turn around (U-turn)
                // Execute a tight U-turn by chaining two fast right turns
                // First half of U-turn
                turnRightFast();
                
                // Brief stabilization
                leftMotor.setPower(FORWARD_SPEED * 0.4);
                rightMotor.setPower(FORWARD_SPEED * 0.4);
                delay(30);
                
                // Second half of U-turn
                turnRightFast();
                
                // Continue forward
                moveForwardFast(1);
                break;
                
            case 3: // Turn left
                // Start turning while still moving forward for smooth transition
                leftMotor.setPower(FORWARD_SPEED * 0.3);
                rightMotor.setPower(FORWARD_SPEED * 0.8);
                delay(50);
                
                // Complete the turn with optimized fast turn
                turnLeftFast();
                
                // Continue forward without stopping
                moveForwardFast(1);
                break;
        }
    }
}

// Main update function - called from the loop
void Floodfill::update() {
    // State machine for the different phases of maze solving
    switch (state) {
        case EXPLORING:
            // Update wall knowledge during exploration
            updateWalls();
            
            if (currentX == GOAL_X && currentY == GOAL_Y) {
                // Reached goal, prepare to return to start
                Serial.println("Goal reached! Returning to start");
                floodFill(0, 0); // Recalculate distances with start as the goal
                state = RETURNING;
            } else {
                floodFill(GOAL_X, GOAL_Y);
                navigateNextCell();
            }
            break;
            
        case RETURNING:
            // Update wall knowledge during return trip
            updateWalls();
            
            if (currentX == 0 && currentY == 0) {
                // Back at start, prepare for speed run
                Serial.println("Back at start! Preparing for speed run");
                floodFill(GOAL_X, GOAL_Y);
                
                // Brief pause to prepare for speed run
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                delay(1000);
                
                state = SPEEDRUN;
                Serial.println("Starting speed run!");
            } else {
                navigateNextCell();
            }
            break;
            
        case SPEEDRUN:
            // During speed run, we rely on the mapped maze rather than checking walls
            // This allows for much faster navigation
            
            if (currentX == GOAL_X && currentY == GOAL_Y) {
                // Speed run complete
                Serial.println("Speed run complete!");
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                state = FINISHED;
            } else {
                // Use optimized high-speed navigation
                navigateSpeedRun();
            }
            break;
            
        case FINISHED:
            // Nothing to do when finished
            break;
    }
}
