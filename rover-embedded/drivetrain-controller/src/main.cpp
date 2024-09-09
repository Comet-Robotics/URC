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

    char inputBuffer[32];
    
    if (Serial.available() > 0)
    {
      int bytesRead = Serial.readBytesUntil('\n', inputBuffer, sizeof(inputBuffer) - 1);
      
      inputBuffer[bytesRead] = '\0';
      
      int num1, num2;
      
      if (sscanf(inputBuffer, "%d:%d", &num1, &num2) == 2)
      {
        Serial.print("Parsed numbers: ");
        Serial.print(num1);
        Serial.print(", ");
        Serial.println(num2);
        num1 = max(-100, min(100, num1));
        num2 = max(-100, min(100, num2));
        float speed_1 = (float)num1 /100.0;
        float speed_2 = (float)num2 /100.0;
        LBMotor.set_speed(PWM_Instance, speed_1);
        LFMotor.set_speed(PWM_Instance, speed_1);
        RBMotor.set_speed(PWM_Instance, speed_2);
        RFMotor.set_speed(PWM_Instance, speed_2);
      }
      else
      {
        Serial.println("Failed to parse numbers");
      }
  }



  



  
}
