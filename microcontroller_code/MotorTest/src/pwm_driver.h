

#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H
#include "constants.h"


void setMotorPWM(MotorID motor, int pwmValue);
void setGroupDirection(Group group, MotorDirection direction);

#endif