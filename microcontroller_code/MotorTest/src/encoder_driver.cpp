#include <Arduino.h>
#include "constants.h"
#include "encoder_driver.h"

bool forward = true;
volatile long motor_pulse_count[6] = {0L};

//todo: update to handle all 6 motor interrupts 
//todo: add negative values for reverse direction

//function called every pulse


void motorSpeedISR1() {
  motor_pulse_count[0]++;
}
  
void motorSpeedISR2() {
  motor_pulse_count[1]++;
}

void motorSpeedISR3() {
  motor_pulse_count[2]++;
}

void motorSpeedISR4() {
  motor_pulse_count[3]++;
}

void motorSpeedISR5() {
  motor_pulse_count[4]++;
}

void motorSpeedISR6() {
  motor_pulse_count[5]++;
}

void initMotorSpeedReader() {
  //pinMode(MOTOR_ESC_SPEED_PINS[0], INPUT);  // No pull-up for direct connection test
  for(int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(MOTOR_ESC_SPEED_PINS[i], INPUT);
  }
  if (forward)
    for (int i = 0; i < MOTOR_COUNT; i++) {
      attachInterrupt(digitalPinToInterrupt(MOTOR_ESC_SPEED_PINS[i]), ++motor_pulse_count[i], CHANGE); // Try RISING instead of CHANGE
    }
  else
    for (int i = 0; i < MOTOR_COUNT; i++) {
      attachInterrupt(digitalPinToInterrupt(MOTOR_ESC_SPEED_PINS[i]), --motor_pulse_count[i], CHANGE); // Try RISING instead of CHANGE
    }
  // attachInterrupt(digitalPinToInterrupt(MOTOR_ESC_SPEED_PINS[0]), motorSpeedISR1, CHANGE);  // Try RISING instead of CHANGE
  // attachInterrupt(digitalPinToInterrupt(MOTOR_ESC_SPEED_PINS[1]), motorSpeedISR2, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(MOTOR_ESC_SPEED_PINS[2]), motorSpeedISR3, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(MOTOR_ESC_SPEED_PINS[3]), motorSpeedISR4, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(MOTOR_ESC_SPEED_PINS[4]), motorSpeedISR5, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(MOTOR_ESC_SPEED_PINS[5]), motorSpeedISR6, CHANGE);
}

long readMotorPulses(MotorID motor) {
  long count;
  noInterrupts();
  count = motor_pulse_count[motor];
  interrupts();
  return count;
}

void resetMotorPulses(MotorID motor) {
  noInterrupts();
  motor_pulse_count[motor] = 0L;
  interrupts();
}

void resetAllMotorPulses() {
  noInterrupts();
  for(int i = 0; i < MOTOR_COUNT; i++) {
    motor_pulse_count[i] = 0L;
  }
  interrupts();
}

float calculateMotorAngleDegrees(MotorID motor) {
  long pulses = readMotorPulses(motor);
  //that static cast thing is like the normal cast, but without the slowness of a normal cast like (float)
  float angle = (static_cast<float>(pulses) / PULSES_PER_REVOLUTION) * 360.0f;
  return angle;
}