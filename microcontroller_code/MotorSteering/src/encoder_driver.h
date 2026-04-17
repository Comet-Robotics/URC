#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H
#include "constants.h"

// Brushless Motor ESC Speed Signal Pin
// Using pin 2 (INT0) for hardware interrupt - most efficient for pulse counting
// Pin 2 = Arduino Uno's INT0, supports attachInterrupt() for rising/falling edge detection



// Function declarations
long readMotorPulses(MotorID motor);
void resetMotorPulses(MotorID motor);
void resetAllMotorPulses();
void initMotorSpeedReader();
float calculateMotorAngleDegrees(MotorID motor);

#endif // ENCODER_DRIVER_H