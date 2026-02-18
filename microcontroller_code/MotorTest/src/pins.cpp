#include "constants.h"


//pins go from front left wheel to rear right wheel left to right
// Updated for Raspberry Pi Pico GPIO pins
int MOTOR_ESC_SPEED_PINS[MOTOR_COUNT] = {16, 17, 18, 19, 20, 21}; //encoder pins (interrupt capable)
int MOTOR_PWM_PINS[MOTOR_COUNT] = {2, 3, 4, 5, 6, 7}; // PWM capable pins
int MOTOR_DIRECTION_PINS[MOTOR_COUNT] = {8, 9}; // Direction control pins
