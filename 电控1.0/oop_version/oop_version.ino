// =============================================================================
// LIBRARIES AND CONFIGURATION
// =============================================================================

// Arduino/ESP32 Core Libraries
#include <Arduino.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <rtc_wdt.h>
#include <ESP32Servo.h>
#include <EEPROM.h>

// --- Robot Physical Dimensions ---
const float length_a = 55.0;    // 大腿长度
const float length_b = 77.5;    // 小腿长度
const float length_c = 27.5;    // 髋关节到肩部的偏移
const float length_side = 71.0; // 机身两侧腿的间距

const float MOVE_SPEED = 2;
const float TURN_SPEED = 1.5;
const float SIT_SPEED = 1;
const float STAND_SPEED =1;

// --- Motion Constants ---
const float z_default = -50.0;  // 站立时Z坐标
const float z_up = -30.0;       // 抬腿时Z坐标
const float z_boot = -28.0;     // 坐下时Z坐标
const float x_default = 62.0;   // 站立时X坐标
const float y_start = 0.0;      // Y坐标起始值
const float y_step = 40.0;      // 步长

// --- Pin Definitions ---
const int SERVO_PINS[4][3] = {{18, 5, 19}, {2, 4, 15}, {33, 25, 32}, {14, 27, 13}};

// Helper constant for clarity
const float KEEP_COORD = 1e6; // A large number to signify "keep current coordinate"

const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
const float turn_x1 = (temp_a - length_side) / 2;  // 转弯时腿1的X目标坐标
const float turn_y1 = y_start + y_step / 2;        // 转弯时腿1的Y目标坐标
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha); // 转弯时腿0的X目标坐标
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side; // 转弯时腿0的Y目标坐标

int Foot[4] = {0,0,0,0};
int Elbow[4] = {0,0,0,0};
int Shdr[4] = {0,0,0,0};


// =============================================================================
// CLASS DEFINITIONS
// =============================================================================

/**
 * @class Leg
 * @brief Manages all properties and actions for a single leg.
 *
 * Encapsulates 3 servos, inverse kinematics, and position control for one leg
 * of the quadruped robot.
 */
class Leg {
public:
    // Public properties
    float site_now[3];    // Current coordinates [x, y, z]portTICK_PERIOD_MS
    float site_expect[3]; // Target coordinates [x, y, z]

private:
    // Private properties
    Servo servos[3];
    int8_t offsets[3];
    int foot = 0;
    int elbow = 0;
    int shdr = 0;
    float temp_speed[3];
    uint8_t leg_id;

public:
    // Default constructor
    Leg() {}

    /**
     * @brief Initializes and attaches the leg's servos.
     * @param id The ID of the leg (0-3).
     * @param pins An array of 3 pin numbers for the servos.
     */
    void attach(uint8_t id, const int pins[3]) {
        leg_id = id;
        foot = Foot[id];
        elbow = Elbow[id];
        shdr = Shdr[id];
        for (int i = 0; i < 3; i++) {
            offsets[i] = EEPROM.read(leg_id * 3 + i);
            servos[i].attach(pins[i], 500, 2500);
            servos[i].write(90 + offsets[i]);
            delay(20);
        }
    }
    
    /**
     * @brief Sets the target coordinates for the leg's foot.
     * @param x Target X coordinate.
     * @param y Target Y coordinate.
     * @param z Target Z coordinate.
     * @param speed The speed of the movement.
     */
    void setTargetSite(float x, float y, float z, float speed) {
        float path_len[3] = {0};

        if (x != KEEP_COORD) path_len[0] = x - site_now[0];
        if (y != KEEP_COORD) path_len[1] = y - site_now[1];
        if (z != KEEP_COORD) path_len[2] = z - site_now[2];

        float total_path = sqrt(pow(path_len[0], 2) + pow(path_len[1], 2) + pow(path_len[2], 2));

        if (total_path > 0.01) { // Avoid division by zero
            temp_speed[0] = path_len[0] / total_path * speed;
            temp_speed[1] = path_len[1] / total_path * speed;
            temp_speed[2] = path_len[2] / total_path * speed;
        } else {
            temp_speed[0] = temp_speed[1] = temp_speed[2] = 0;
        }

        if (x != KEEP_COORD) site_expect[0] = x;
        if (y != KEEP_COORD) site_expect[1] = y;
        if (z != KEEP_COORD) site_expect[2] = z;
    }

    /**
     * @brief Updates the leg's current position and servo angles.
     * This should be called in a high-frequency loop (e.g., every 15ms).
     */
    void update() {
        for (int i = 0; i < 3; i++) {
            if (abs(site_now[i] - site_expect[i]) >= abs(temp_speed[i])) {
                site_now[i] += temp_speed[i];
            } else {
                site_now[i] = site_expect[i];
            }
        }
        
        float alpha, beta, gamma;
        cartesianToPolar(alpha, beta, gamma, site_now[0], site_now[1], site_now[2]);
        polarToServo(alpha, beta, gamma);
    }
    
    /**
     * @brief Checks if the leg has reached its target coordinates.
     * @return True if at target, false otherwise.
     */
    bool isAtTarget() {
        return (site_now[0] == site_expect[0]) &&
               (site_now[1] == site_expect[1]) &&
               (site_now[2] == site_expect[2]);
    }

private:
    /**
     * @brief Inverse Kinematics: Converts Cartesian coordinates (x,y,z) to joint angles.
     */
    void cartesianToPolar(float &alpha, float &beta, float &gamma, float x, float y, float z) {
        float v, w;
        w = (x >= 0 ? 1 : -1) * sqrt(pow(x, 2) + pow(y, 2));
        v = w - length_c;
        alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
        beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
        gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
        
        alpha = alpha * 180.0 / PI;
        beta = beta * 180.0 / PI;
        gamma = gamma * 180.0 / PI;
    }

    /**
     * @brief Converts joint angles to servo angles and writes them to the servos.
     */
    void polarToServo(float alpha, float beta, float gamma) {
        // These transformations are specific to the robot's construction
        if (leg_id == 0) { // Right-Front
            alpha = 90 - alpha - elbow;
            beta = beta - foot;
            gamma = 90 + gamma + shdr;
        } else if (leg_id == 1) { // Right-Rear
            alpha = 90 + alpha - elbow;
            beta = 180 - beta + foot;
            gamma = 90 - gamma - shdr;
        } else if (leg_id == 2) { // Left-Front
            alpha = 90 + alpha + elbow;
            beta = 180 - beta + foot;
            gamma = 90 - gamma - shdr;
        } else if (leg_id == 3) { // Left-Rear
            alpha = 90 - alpha - elbow;
            beta = beta - foot;
            gamma = 90 + gamma + shdr;
        }
        
        servos[0].write(alpha + offsets[0]);
        servos[1].write(beta + offsets[1]);
        servos[2].write(gamma + offsets[2]);
    }
};

/**
 * @class Robot
 * @brief Manages the entire robot, including its 4 legs and high-level gaits.
 */
class Robot {
public:
    Leg legs[4];

public:
    Robot() {}

    /**
     * @brief Initializes the robot, including all legs and EEPROM.
     */
    void init() {
        EEPROM.begin(64);
        for (int i = 0; i < 4; i++) {
            legs[i].attach(i, SERVO_PINS[i]);
        }
        // Initialize leg positions to a sitting state
        legs[0].site_now[0] = legs[0].site_expect[0] = x_default;
        legs[0].site_now[1] = legs[0].site_expect[1] = y_start + y_step;
        legs[0].site_now[2] = legs[0].site_expect[2] = z_boot;

        legs[1].site_now[0] = legs[1].site_expect[0] = x_default;
        legs[1].site_now[1] = legs[1].site_expect[1] = y_start + y_step;
        legs[1].site_now[2] = legs[1].site_expect[2] = z_boot;

        legs[2].site_now[0] = legs[2].site_expect[0] = x_default;
        legs[2].site_now[1] = legs[2].site_expect[1] = y_start;
        legs[2].site_now[2] = legs[2].site_expect[2] = z_boot;

        legs[3].site_now[0] = legs[3].site_expect[0] = x_default;
        legs[3].site_now[1] = legs[3].site_expect[1] = y_start;
        legs[3].site_now[2] = legs[3].site_expect[2] = z_boot;
    }

    /**
     * @brief Updates all legs. Should be called in the high-frequency servo task.
     */
    void update() {
        for (int i = 0; i < 4; i++) {
            legs[i].update();
        }
    }

    /**
     * @brief Waits until all legs have reached their target positions.
     */
    void waitForAllLegs() {
        while (!legs[0].isAtTarget() || !legs[1].isAtTarget() || !legs[2].isAtTarget() || !legs[3].isAtTarget()) {
            vTaskDelay(1);
        }
    }

    // --- High-Level Gait Functions ---

    void stand() {
        for (int i = 0; i < 4; i++) {
            legs[i].setTargetSite(KEEP_COORD, KEEP_COORD, z_default, STAND_SPEED);
        }
        waitForAllLegs();
    }

    void sit() {
        for (int i = 0; i < 4; i++) {
            legs[i].setTargetSite(KEEP_COORD, KEEP_COORD, z_boot, SIT_SPEED);
        }
        waitForAllLegs();
    }
    
    void step_forward(unsigned int steps) {
    for (unsigned int i = 0; i < steps; i++) {
        if (legs[2].site_now[1] == y_start) { // Phase 1: Move legs 2 & 1
            legs[2].setTargetSite(x_default, y_start, z_up, MOVE_SPEED);
            waitForAllLegs();
            legs[2].setTargetSite(x_default, y_start + 2 * y_step, z_up, MOVE_SPEED);
            waitForAllLegs();
            legs[2].setTargetSite(x_default, y_start + 2 * y_step, z_default, MOVE_SPEED);
            waitForAllLegs();

            // Shift body
            legs[0].setTargetSite(x_default, y_start, z_default, MOVE_SPEED);
            legs[1].setTargetSite(x_default, y_start + 2 * y_step, z_default, MOVE_SPEED);
            legs[2].setTargetSite(x_default, y_start + y_step, z_default, MOVE_SPEED);
            legs[3].setTargetSite(x_default, y_start + y_step, z_default, MOVE_SPEED);
            waitForAllLegs();

            legs[1].setTargetSite(x_default, y_start + 2 * y_step, z_up, MOVE_SPEED);
            waitForAllLegs();
            legs[1].setTargetSite(x_default, y_start, z_up, MOVE_SPEED);
            waitForAllLegs();
            legs[1].setTargetSite(x_default, y_start, z_default, MOVE_SPEED);
            waitForAllLegs();
        } else { // Phase 2: Move legs 0 & 3
            legs[0].setTargetSite(x_default, y_start, z_up, MOVE_SPEED);
            waitForAllLegs();
            legs[0].setTargetSite(x_default, y_start + 2 * y_step, z_up, MOVE_SPEED);
            waitForAllLegs();
            legs[0].setTargetSite(x_default, y_start + 2 * y_step, z_default, MOVE_SPEED);
            waitForAllLegs();

            // Shift body
            legs[0].setTargetSite(x_default, y_start + y_step, z_default, MOVE_SPEED);
            legs[1].setTargetSite(x_default, y_start + y_step, z_default, MOVE_SPEED);
            legs[2].setTargetSite(x_default, y_start, z_default, MOVE_SPEED);
            legs[3].setTargetSite(x_default, y_start + 2 * y_step, z_default, MOVE_SPEED);
            waitForAllLegs();

            legs[3].setTargetSite(x_default, y_start + 2 * y_step, z_up, MOVE_SPEED);
            waitForAllLegs();
            legs[3].setTargetSite(x_default, y_start, z_up, MOVE_SPEED);
            waitForAllLegs();
            legs[3].setTargetSite(x_default, y_start, z_default, MOVE_SPEED);
            waitForAllLegs();
        }
    }
}

void step_backward(unsigned int steps){
    for(unsigned int i = 0; i < steps; i++){  // 修正了原代码中的循环变量错误
        if (legs[3].site_now[1] == y_start){
            legs[3].setTargetSite(x_default, y_start, z_up, MOVE_SPEED);
            waitForAllLegs();
            legs[3].setTargetSite(x_default, y_start + 2 * y_step, z_up, MOVE_SPEED);
            waitForAllLegs();
            legs[3].setTargetSite(x_default, y_start + 2 * y_step, z_default, MOVE_SPEED);
            waitForAllLegs();

            // Shift body
            legs[0].setTargetSite(x_default, y_start + 2 * y_step, z_default, MOVE_SPEED);
            legs[1].setTargetSite(x_default, y_start, z_default, MOVE_SPEED);
            legs[2].setTargetSite(x_default, y_start + y_step, z_default, MOVE_SPEED);
            legs[3].setTargetSite(x_default, y_start + y_step, z_default, MOVE_SPEED);
            waitForAllLegs();

            legs[0].setTargetSite(x_default, y_start + 2 * y_step, z_up, MOVE_SPEED);
            waitForAllLegs();
            legs[0].setTargetSite(x_default, y_start, z_up, MOVE_SPEED);
            waitForAllLegs();
            legs[0].setTargetSite(x_default, y_start, z_default, MOVE_SPEED);
            waitForAllLegs();
        }
        else{
            legs[1].setTargetSite(x_default, y_start, z_up, MOVE_SPEED);
            waitForAllLegs();
            legs[1].setTargetSite(x_default, y_start + 2 * y_step, z_up, MOVE_SPEED);
            waitForAllLegs();
            legs[1].setTargetSite(x_default, y_start + 2 * y_step, z_default, MOVE_SPEED);
            waitForAllLegs();

            // Shift body
            legs[0].setTargetSite(x_default, y_start + y_step, z_default, MOVE_SPEED);
            legs[1].setTargetSite(x_default, y_start + y_step, z_default, MOVE_SPEED);
            legs[2].setTargetSite(x_default, y_start + 2 * y_step, z_default, MOVE_SPEED);
            legs[3].setTargetSite(x_default, y_start, z_default, MOVE_SPEED);
            waitForAllLegs();

            legs[2].setTargetSite(x_default, y_start, z_up, MOVE_SPEED);
            waitForAllLegs();
            legs[2].setTargetSite(x_default, y_start, z_up, MOVE_SPEED);
            waitForAllLegs();
            legs[2].setTargetSite(x_default, y_start, z_default, MOVE_SPEED);
            waitForAllLegs();
        }
    }
}


  void turn_left(unsigned int steps){
     while(steps-->0){
      if(legs[3].site_now[1]==y_start){
      legs[3].setTargetSite(x_default,y_start,z_up,TURN_SPEED);
      waitForAllLegs();
      legs[0].setTargetSite(turn_x1,turn_y1,z_default,TURN_SPEED);
      legs[1].setTargetSite(turn_x0,turn_y0,z_default,TURN_SPEED);
      legs[2].setTargetSite(turn_x1,turn_y1,z_default,TURN_SPEED);
      legs[3].setTargetSite(turn_x0,turn_y0,z_up,TURN_SPEED);
      waitForAllLegs();
      legs[3].setTargetSite(turn_x0,turn_y0,z_default,TURN_SPEED);
      waitForAllLegs();
      legs[0].setTargetSite(turn_x1,turn_y1,z_default,TURN_SPEED);
      legs[1].setTargetSite(turn_x0,turn_y0,z_default,TURN_SPEED);
      legs[2].setTargetSite(turn_x1,turn_y1,z_default,TURN_SPEED);
      legs[3].setTargetSite(turn_x0,turn_y0,z_default,TURN_SPEED);
      waitForAllLegs();
      legs[1].setTargetSite(turn_x0,turn_y0,z_up,TURN_SPEED);
      waitForAllLegs();
      legs[0].setTargetSite(x_default,y_start,z_default,TURN_SPEED);
      legs[1].setTargetSite(x_default,y_start,z_up,TURN_SPEED);
      legs[2].setTargetSite(x_default,y_start+y_step,z_default,TURN_SPEED);
      legs[3].setTargetSite(x_default,y_start+y_step,z_default,TURN_SPEED);
      waitForAllLegs();
      legs[1].setTargetSite(x_default,y_start,z_default,TURN_SPEED);
      waitForAllLegs();
    }
    else{
      legs[0].setTargetSite(x_default,y_start,z_up,TURN_SPEED);
      waitForAllLegs();
      legs[0].setTargetSite(turn_x0,turn_y0,z_up,TURN_SPEED);
      legs[1].setTargetSite(turn_x1,turn_y1,z_default,TURN_SPEED);
      legs[2].setTargetSite(turn_x0,turn_y0,z_default,TURN_SPEED);
      legs[3].setTargetSite(turn_x1,turn_y1,z_default,TURN_SPEED);
      waitForAllLegs();
      legs[0].setTargetSite(turn_x0,turn_y0,z_default,TURN_SPEED);
      waitForAllLegs();
      legs[0].setTargetSite(turn_x0,turn_y0,z_default,TURN_SPEED);
      legs[1].setTargetSite(turn_x1,turn_y1,z_default,TURN_SPEED);
      legs[2].setTargetSite(turn_x0,turn_y0,z_default,TURN_SPEED);
      legs[3].setTargetSite(turn_x1,turn_y1,z_default,TURN_SPEED);
      waitForAllLegs();
      legs[2].setTargetSite(turn_x0,turn_y0,z_up,TURN_SPEED);
      waitForAllLegs();
      legs[0].setTargetSite(x_default,y_start+y_step,z_default,TURN_SPEED);
      legs[1].setTargetSite(x_default,y_start+y_step,z_default,TURN_SPEED);
      legs[2].setTargetSite(x_default,y_start,z_up,TURN_SPEED);
      legs[3].setTargetSite(x_default,y_start,z_default,TURN_SPEED);
      waitForAllLegs();
      legs[2].setTargetSite(x_default,y_start,z_default,TURN_SPEED);
      waitForAllLegs();
    }
    }
    
  }

  void turn_right(unsigned int steps){
    while(steps-->0){
      if(legs[2].site_now[1]==y_start){
      legs[2].setTargetSite(x_default,y_start,z_up,TURN_SPEED);
      waitForAllLegs();
      legs[0].setTargetSite(turn_x0,turn_y0,z_default,TURN_SPEED);
      legs[1].setTargetSite(turn_x1,turn_y1,z_default,TURN_SPEED);
      legs[2].setTargetSite(turn_x0,turn_y0,z_up,TURN_SPEED);
      legs[3].setTargetSite(turn_x1,turn_y1,z_default,TURN_SPEED);
      waitForAllLegs();
      legs[2].setTargetSite(turn_x0,turn_y0,z_default,TURN_SPEED);
      waitForAllLegs();
      legs[0].setTargetSite(turn_x0,turn_y0,z_default,TURN_SPEED);
      legs[1].setTargetSite(turn_x1,turn_y1,z_default,TURN_SPEED);
      legs[2].setTargetSite(turn_x0,turn_y0,z_default,TURN_SPEED);
      legs[3].setTargetSite(turn_x1,turn_y1,z_default,TURN_SPEED);
      waitForAllLegs();
      legs[0].setTargetSite(turn_x0,turn_y0,z_up,TURN_SPEED);
      waitForAllLegs();
      legs[0].setTargetSite(x_default,y_start,z_up,TURN_SPEED);
      legs[1].setTargetSite(x_default,y_start,z_default,TURN_SPEED);
      legs[2].setTargetSite(x_default,y_start+y_step,z_default,TURN_SPEED);
      legs[3].setTargetSite(x_default,y_start+y_step,z_default,TURN_SPEED);
      waitForAllLegs();
      legs[0].setTargetSite(x_default,y_start,z_default,TURN_SPEED);
      waitForAllLegs();
    }
    else{
      legs[1].setTargetSite(x_default,y_start,z_up,TURN_SPEED);
      waitForAllLegs();
      legs[0].setTargetSite(turn_x1,turn_y1,z_default,TURN_SPEED);
      legs[1].setTargetSite(turn_x0,turn_y0,z_up,TURN_SPEED);
      legs[2].setTargetSite(turn_x1,turn_y1,z_default,TURN_SPEED);
      legs[3].setTargetSite(turn_x0,turn_y0,z_default,TURN_SPEED);
      waitForAllLegs();
      legs[1].setTargetSite(turn_x0,turn_y0,z_default,TURN_SPEED);
      waitForAllLegs();
      legs[0].setTargetSite(turn_x1,turn_y1,z_default,TURN_SPEED);
      legs[1].setTargetSite(turn_x0,turn_y0,z_default,TURN_SPEED);
      legs[2].setTargetSite(turn_x1,turn_y1,z_default,TURN_SPEED);
      legs[3].setTargetSite(turn_x0,turn_y0,z_default,TURN_SPEED);
      waitForAllLegs();
      legs[3].setTargetSite(turn_x0,turn_y0,z_up,TURN_SPEED);
      waitForAllLegs();
      legs[0].setTargetSite(x_default,y_start+y_step,z_default,TURN_SPEED);
      legs[1].setTargetSite(x_default,y_start+y_step,z_default,TURN_SPEED);
      legs[2].setTargetSite(x_default,y_start,z_default,TURN_SPEED);
      legs[3].setTargetSite(x_default,y_start,z_up,TURN_SPEED);
      waitForAllLegs();
      legs[3].setTargetSite(x_default,y_start,z_default,TURN_SPEED);
      waitForAllLegs();
    }
  }
  }
    
    
    // NOTE: turn_left, turn_right, step_back can be refactored similarly to step_forward
    // For brevity, they are omitted here but the logic would follow the same pattern:
    // replacing set_site() with legs[i].setTargetSite() and wait_all_reach() with waitForAllLegs().
};

// =============================================================================
// GLOBAL OBJECT AND TASKS
// =============================================================================

Robot myRobot; // Create a single global instance of the Robot

// Task1 (Core0): High-level autonomous logic
void Task1(void *pvParameters) {
    (void) pvParameters;
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    Serial.println("Task1: Starting autonomous routine...");
    myRobot.stand();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1) {
        myRobot.step_forward(6); // Shortened for demonstration
        vTaskDelay(500 / portTICK_PERIOD_MS);
        myRobot.step_backward(6);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        myRobot.turn_right(4);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        myRobot.turn_left(4);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        myRobot.sit();
        vTaskDelay(500 / portTICK_PERIOD_MS);
        myRobot.stand();
        vTaskDelay(500 / portTICK_PERIOD_MS);
        
        // The original complex sequence can be placed here using myRobot methods
        // myRobot.turn_left(2);
        // ...

        // Loop indefinitely after routine is complete
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

// Task2 (Core1): High-frequency servo update service
void Task2(void *pvParameters) {
    while (1) {
        rtc_wdt_feed();
        myRobot.update();
        vTaskDelay(15 / portTICK_PERIOD_MS); // ~66 Hz update rate
    }
}

// =============================================================================
// ARDUINO SETUP AND LOOP
// =============================================================================

void setup() {
    // Watchdog Configuration
    rtc_wdt_protect_off();
    rtc_wdt_enable();
    rtc_wdt_set_time(RTC_WDT_STAGE0, 10000); // 10 seconds timeout
    rtc_wdt_feed();

    // Serial Port Initialization
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nQuadruped Robot - OOP Version");
    
    // Robot Initialization
    Serial.println("Initializing Robot...");
    myRobot.init();
    Serial.println("Robot Initialization Complete.");

    // Brownout Detector Disable
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    // Create FreeRTOS Tasks
    xTaskCreatePinnedToCore(Task1, "Task1_AutoLogic", 10000, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(Task2, "Task2_ServoService", 10000, NULL, 2, NULL, 1); // Higher priority for servos

    Serial.println("Tasks created. Robot is running.");
}

void loop() {
    // The main loop is now idle as all logic is in FreeRTOS tasks.
    // We just feed the watchdog here as a fallback.
    rtc_wdt_feed();
    vTaskDelay(100);
}