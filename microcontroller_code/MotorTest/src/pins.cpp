#include "constants.h"


//pins go from front left wheel to rear right wheel left to right
// Updated for Raspberry Pi Pico GPIO pins

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/DIFF DRIVE)
    MOTOR3  MOTOR4  (4WD/MECANUM)
    MOTOR5  MOTOR6  (6WD Ackermann Rover system)  
         BACK
*/

int MOTOR_ESC_SPEED_PINS[MOTOR_COUNT] = {16, 17, 18, 19, 20, 21}; //encoder pins (interrupt capable)
int MOTOR_PWM_PINS[MOTOR_COUNT] = {2, 3, 4, 5, 6, 7}; // PWM capable pins
int MOTOR_DIRECTION_PINS[MOTOR_COUNT] = {9, 10, 11, 12, 13, 14}; // Direction control pins (avoiding pin 8 conflict with LED)
