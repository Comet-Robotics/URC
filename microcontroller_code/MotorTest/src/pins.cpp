#include "constants.h"


//pins go from front left wheel to rear right wheel left to right
// Updated for teensy 41 GPIO pins

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/DIFF DRIVE)
    MOTOR3  MOTOR4  (4WD/MECANUM)
    MOTOR5  MOTOR6  (6WD Ackermann Rover system)  
         BACK
*/

// index starts at motor 1 to motor 6
int MOTOR_ESC_SPEED_PINS[MOTOR_COUNT] = {21, 15, 20, 14, 17, 41}; //encoder pins (interrupt capable)
int MOTOR_PWM_PINS[MOTOR_COUNT] = {2, 3, 4, 5, 6, 7}; // PWM capable pins
int MOTOR_DIRECTION_PINS[MOTOR_COUNT] = {38, 37, 38, 37, 38, 37}; // Direction control pins (avoiding pin 8 conflict with LED)
int MOTOR_BRAKE_PINS[MOTOR_COUNT] = {34, 34, 34, 34, 34, 34}; // Brake control pins 
