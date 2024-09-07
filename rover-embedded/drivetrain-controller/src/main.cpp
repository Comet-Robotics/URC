#define _PWM_LOGLEVEL_       4

#include "Teensy_PWM.h"
#include "motor.h"



//creates pwm instance
Teensy_PWM* PWM_Instance;

Motor LBMotor = Motor(5,50);
Motor LFMotor = Motor(6,50);
Motor RBMotor = Motor(9,50);
Motor RFMotor = Motor(10,50);


void setup()
{
  Serial.begin(115200);

  while (!Serial && millis() < 5000);


  Serial.print(F("\nStarting PWM_Basic using FlexTimers on "));
  Serial.println(BOARD_NAME);
  Serial.println(TEENSY_PWM_VERSION);


  if ( (!PWM_Instance) || !PWM_Instance->isPWMEnabled())
  {
    Serial.print(F("Stop here forever"));

    while (true)
      delay(10000);
  }
}



void loop()
{
  String a = Serial.readStringUntil('\n');



  
}
