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

void setMotorDirection(MotorID motorID, MotorDirection direction) {
  int dirPin = MOTOR_DIRECTION_PINS[motorID];
  if(static_cast<int>(motorID) % 2 == 0) { // Left motors
    digitalWrite(dirPin, direction == FORWARD ? HIGH : LOW);
  }
  else { // Right motors
    digitalWrite(dirPin, direction == FORWARD ? LOW : HIGH); // Invert direction for right motors
  }
}

void setGroupDirection(Group group, MotorDirection direction) {
  // Set direction for all motors on the specified group
  if(group = ALL)
  {
    setGroupDirection(LEFT, direction);
    setGroupDirection(RIGHT, direction);
  }
  
  for(int i = 0; i < MOTOR_COUNT; i++) 
  {
    if ((group == LEFT && (i % 2 == 0))) 
    {
      setMotorDirection(static_cast<MotorID>(i), direction);
    } 

    else if ((group == RIGHT && (i % 2 == 1))) 
    {
      setMotorDirection(static_cast<MotorID>(i), direction);
    }

    // else if (group == ALL) 
    // {
    //   int dirPin = MOTOR_DIRECTION_PINS[i];
    //   digitalWrite(dirPin, direction == FORWARD ? HIGH : LOW);
    // }

  }
}