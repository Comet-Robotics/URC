#include "constants.h"


//pins go from front left wheel to rear right wheel left to right
// Updated for Raspberry Pi Pico GPIO pins
int MOTOR_ESC_SPEED_PINS[MOTOR_COUNT] = {19, 19, 19, 19, 19, 19}; //encoder pins (interrupt capable)
int MOTOR_PWM_PINS[MOTOR_COUNT] = {5, 5, 5, 5, 5, 5}; // PWM capable pins
int MOTOR_DIRECTION_PINS[MOTOR_COUNT] = {16, 17, 18, 19, 20, 21}; // Direction control pins
