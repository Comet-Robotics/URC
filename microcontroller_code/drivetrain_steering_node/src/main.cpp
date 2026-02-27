#include <Arduino.h>
// currently using this library: https://github.com/RobTillaart/DRV8825/tree/master?tab=readme-ov-file
// I considered using this library: https://github.com/laurb9/StepperDriver but the current one is working fine so far
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
  // The motor to stepper driver assigments are probably wrong but the sets of pins for each stepper driver should be mostly correct (needs further testing).
  // So far we have only tested the back left steering motor, shich is connected to the first stepper driver.
  backLeftSteeringMotor.begin(2, 3, UNUSED_PIN, 18, 17);
  frontRightSteeringMotor.begin(4, 5, UNUSED_PIN, 14,15);
  frontLeftSteeringMotor.begin(6, 7, UNUSED_PIN, 12,13);
  backRightSteeringMotor.begin(8, 9, UNUSED_PIN, 18,17);


  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // the intention behind this was to make the LED blink but it doesn't work as expected yet and just stays on
  bool ledStatus = frontLeftSteeringMotor.getSteps() % 10000;
  digitalWrite(LED_BUILTIN, ledStatus < 100 ? HIGH : LOW);
  
  frontLeftSteeringMotor.step();
  frontRightSteeringMotor.step();
  backLeftSteeringMotor.step();
  backRightSteeringMotor.step();

  // Log front left steering motor
  Serial.print("FL,");
  Serial.print(frontLeftSteeringMotor.getPosition());
  Serial.print(",");
  Serial.print(frontLeftSteeringMotor.getSteps());
  Serial.print(",");
  Serial.print(frontLeftSteeringMotor.getStepPulseLength());
  Serial.print(",");
  Serial.print(frontLeftSteeringMotor.getStepsPerRotation());
  Serial.println();

  // Log front right steering motor
  Serial.print("FR,");
  Serial.print(frontRightSteeringMotor.getPosition());
  Serial.print(",");
  Serial.print(frontRightSteeringMotor.getSteps());
  Serial.print(",");
  Serial.print(frontRightSteeringMotor.getStepPulseLength());
  Serial.print(",");
  Serial.print(frontRightSteeringMotor.getStepsPerRotation());
  Serial.println();

  // Log back left steering motor
  Serial.print("BL,");
  Serial.print(backLeftSteeringMotor.getPosition());
  Serial.print(",");
  Serial.print(backLeftSteeringMotor.getSteps());
  Serial.print(",");
  Serial.print(backLeftSteeringMotor.getStepPulseLength());
  Serial.print(",");
  Serial.print(backLeftSteeringMotor.getStepsPerRotation());
  Serial.println();

  // Log back right steering motor
  Serial.print("BR,");
  Serial.print(backRightSteeringMotor.getPosition());
  Serial.print(",");
  Serial.print(backRightSteeringMotor.getSteps());
  Serial.print(",");
  Serial.print(backRightSteeringMotor.getStepPulseLength());
  Serial.print(",");
  Serial.print(backRightSteeringMotor.getStepsPerRotation());
  Serial.println();

  // one other interesting observation - stepper motor behavior changes based on whether I have the serial monitor open or not. 
}