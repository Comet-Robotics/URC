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

void setGroupDirection(Group group, MotorDirection direction) {
  // Set direction for all motors on the specified group
  for(int i = 0; i < MOTOR_COUNT; i++) 
  {
    if ((group == LEFT && (i % 2 == 0)) || (group == RIGHT && (i % 2 == 1))) 
    {
      int dirPin = MOTOR_DIRECTION_PINS[i];
      digitalWrite(dirPin, direction == FORWARD ? HIGH : LOW);
    } 

    else if (group == ALL) 
    {
      int dirPin = MOTOR_DIRECTION_PINS[i];
      digitalWrite(dirPin, direction == FORWARD ? HIGH : LOW);
    }

  }
}