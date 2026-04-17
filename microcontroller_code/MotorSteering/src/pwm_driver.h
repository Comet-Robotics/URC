

#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H
#include "constants.h"


void setMotorPWM(MotorID motor, int pwmValue);
void setMotorDirection(MotorID motorID, MotorDirection direction);
void setGroupDirection(Group group, MotorDirection direction);

#endif