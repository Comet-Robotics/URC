#include "constants.h"


//pins go from front left wheel to rear right wheel left to right
// Updated for Raspberry Pi Pico GPIO pins
int MOTOR_ESC_SPEED_PINS[MOTOR_COUNT] = {22, 23, 24, 25, 26, 27}; //encoder pins (interrupt capable)
int MOTOR_PWM_PINS[MOTOR_COUNT] = {2, 3, 4, 5, 6, 7}; // PWM capable pins
int MOTOR_DIRECTION_PINS[MOTOR_COUNT] = {28, 29, 30, 31, 32, 33}; // Direction control pins
