#include "constants.h"


//pins go from front left wheel to rear right wheel left to right
// Updated for teensy 4.1 pins as of 2/26/2026

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/DIFF DRIVE)
    MOTOR3  MOTOR4  (4WD/MECANUM)
    MOTOR5  MOTOR6  (6WD Ackermann Rover system)  
         BACK
*/
// index starts at motor 1 to motor 6

// FIXME: Motors 5 seems to be sharing rpm data with motor 2 and 3

int MOTOR_ESC_SPEED_PINS[MOTOR_COUNT] = {15, 21, 17, 40, 20, 14}; //encoder pins (interrupt capable)
int MOTOR_PWM_PINS[MOTOR_COUNT] = {7, 6, 2, 3, 5, 4}; // PWM capable pins
int MOTOR_DIRECTION_PINS[MOTOR_COUNT] = {38, 37, 38, 37, 38, 37}; // Direction control pins (avoiding pin 8 conflict with LED) | UNUSED RN
int MOTOR_BRAKE_PINS[MOTOR_COUNT] = {34, 34, 34, 34, 34, 34}; // Brake control pins  | UNUSED RN


// Steering
/*
----------Motors----------
Motor 1         Motor 2


Motor 3         Motor 4
---------------------------
*/

// {AIN1, AIN2, BIN1, BIN2, sleepPin}
// TODO: update pin assignments for each motor
int STEERING_MOTOR_PINS[STEERING_MOTOR_COUNT][5] = {
    {36, 35, 34, 33, 25},   // SteeringMotor 1
    {7, 8, 9, 10, 11}, // SteeringMotor 2
    {12, 13, 14, 15, 16}, // SteeringMotor 3
    {17, 18, 19, 20, 21} // SteeringMotor 4
};
