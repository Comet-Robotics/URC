#include "constants.h"
/*
----------Motors----------
Motor 1         Motor 2


Motor 3         Motor 4
---------------------------
*/

// {AIN1, AIN2, BIN1, BIN2, sleepPin}
int MOTOR_PINS[MOTOR_COUNT][5] = {
    {36, 35, 34, 33, 25},   // Motor 1
    {7, 8, 9, 10, 11}, // Motor 2
    {12, 13, 14, 15, 16}, // Motor 3
    {17, 18, 19, 20, 21} // Motor 4
};

