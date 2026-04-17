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
// TODO: Check if encoder pins are correct
int MOTOR_ESC_SPEED_PINS[MOTOR_COUNT] = {15, 21, 17, 40}; //encoder pins (interrupt capable)
int MOTOR_PWM_PINS[MOTOR_COUNT] = {2, 3, 4, 5}; // PWM capable pins
int MOTOR_DIRECTION_PINS[MOTOR_COUNT] = {38, 37, 38, 37}; // Direction control pins 
int MOTOR_BRAKE_PINS[MOTOR_COUNT] = {34, 34, 34, 34}; // Brake control pins  | UNUSED RN


// Steering
/*
----------Motors----------
Motor 1         Motor 2


Motor 3         Motor 4
---------------------------
*/

// {AIN1, AIN2, BIN1, BIN2, sleepPin}
// TODO: Pin assignments do not behave as though they are correct. Double check these.
int STEERING_MOTOR_PINS[STEERING_MOTOR_COUNT][5] = {
    {27, 26, 24, 38, 25},   // SteeringMotor 1
    {12, 11, 9, 8, 10}, // SteeringMotor 2
    {32, 31, 29, 28, 30}, // SteeringMotor 3
    {33, 34, 36, 37, 35} // SteeringMotor 4
};
