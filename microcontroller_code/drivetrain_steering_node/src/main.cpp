#include <Arduino.h>
#include "DRV8825.h"

DRV8825 frontLeftSteeringMotor;
DRV8825 frontRightSteeringMotor;
DRV8825 backLeftSteeringMotor;
DRV8825 backRightSteeringMotor;

// TODO: update pinouts
#define UNUSED_PIN 255
void setup() {
  frontLeftSteeringMotor.begin(1, 1, UNUSED_PIN, 1, 1);
  frontRightSteeringMotor.begin(1, 1, UNUSED_PIN, 1, 1);
  backLeftSteeringMotor.begin(1, 1, UNUSED_PIN, 1, 1);
  backRightSteeringMotor.begin(1, 1, UNUSED_PIN, 1, 1);
}

void loop() {
  frontLeftSteeringMotor.step();
  frontRightSteeringMotor.step();
  backLeftSteeringMotor.step();
  backRightSteeringMotor.step();
}