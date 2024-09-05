#define _PWM_LOGLEVEL_       4

#include "Teensy_PWM.h"


  #define pinToUse       5
//creates pwm instance
Teensy_PWM* PWM_Instance;

float frequency = 50.0f;

float dutyCycle = 7.5f;


float setSpeed(int pin,float speed, bool reverse)
{
  if (reverse)
  {
    dutyCycle = 7.5f - (speed * 2.5f);
  }
  else
  {
    dutyCycle = 7.5f + (speed * 2.5f);
  }


  PWM_Instance->setPWM(pin, frequency, dutyCycle);
  return dutyCycle;
}





void setup()
{
  Serial.begin(115200);

  while (!Serial && millis() < 5000);

  delay(500);

  Serial.print(F("\nStarting PWM_Basic using FlexTimers on "));


  Serial.println(BOARD_NAME);
  Serial.println(TEENSY_PWM_VERSION);

  //assigns PWM frequency of 1.0 KHz and a duty cycle of 0%
  PWM_Instance = new Teensy_PWM(pinToUse, frequency, dutyCycle);

  if ( (!PWM_Instance) || !PWM_Instance->isPWMEnabled())
  {
    Serial.print(F("Stop here forever"));

    while (true)
      delay(10000);
  }
}



void loop()
{


  delay(5000);
  for (int i = 0; i <= 20; i++)
  {
    setSpeed(pinToUse,(1.0f/20.0f)*i, false);
    delay(500 );
  }

  PWM_Instance->setPWM(pinToUse, 0, 0);
  delay(1000);

  for (int i = 0; i <= 20; i++)
  {
    setSpeed(pinToUse,(1.0f/20.0f)*i, true);
    delay(500 );
  }
  PWM_Instance->setPWM(pinToUse, 0, 0);
  delay(1000);


  
}
