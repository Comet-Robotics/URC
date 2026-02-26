#include <Arduino.h>
#include "DRV8825.h"

DRV8825 frontLeftSteeringMotor;
DRV8825 frontRightSteeringMotor;
DRV8825 backLeftSteeringMotor;
DRV8825 backRightSteeringMotor;

/*
Pins connected to the Pico on the protoboard as of 2/25/2026 according to Pablo.
NOTE: GP pins are NOT the same as the physical pin numbers on the Pico.
use https://pico.pinout.xyz as a reference for mapping GP pins to physical pins.
  -------------------------------------------------------------
  - GP 2/3: direction + step for the first stepper driver
  - GP 4/5: direction + step for the second stepper driver
  - GP 6/7: direction + step for the third stepper driver
  - GP 8/9: direction + step for the fourth stepper driver

  - GP 10/11: sleep + reset for the fourth stepper driver
  - GP 12/13: sleep + reset for the third stepper driver
  - GP 14/15: sleep + reset for the second stepper driver
  - GP 18/17: sleep + reset for the first stepper driver
*/

#define UNUSED_PIN 255
void setup() {
  // The motor to stepper driver assigments are probably wrong but the sets of pins for each stepper driver should be mostly correct (needs testing).
  frontLeftSteeringMotor.begin(4, 5, UNUSED_PIN, 24, 22);
  frontRightSteeringMotor.begin(6, 7, UNUSED_PIN, 19,20);
  backLeftSteeringMotor.begin(9, 10, UNUSED_PIN, 16,17);
  backRightSteeringMotor.begin(11, 12, UNUSED_PIN, 14,15);
}

void loop() {
  frontLeftSteeringMotor.step();
  frontRightSteeringMotor.step();
  backLeftSteeringMotor.step();
  backRightSteeringMotor.step();
}