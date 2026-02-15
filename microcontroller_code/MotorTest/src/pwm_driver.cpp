#include <Arduino.h>
#include "pwm_driver.h"


//arduino analogWrite range is 0-255
//todo: figure out a way to adapt pwm max for 
void setMotorPWM(MotorID motor, int pwmValue) {
  //error checking  
  if (motor < FRONT_LEFT_MOTOR || motor > REAR_RIGHT_MOTOR) {
        return;
  }

  int pwmPin = MOTOR_PWM_PINS[motor];
  if (pwmValue > PWM_MAX_VALUE)
    pwmValue = PWM_MAX_VALUE;
  analogWrite(pwmPin, pwmValue);
}