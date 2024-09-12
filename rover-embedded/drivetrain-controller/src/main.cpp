#define _PWM_LOGLEVEL_       4

#include "Teensy_PWM.h"
#include "motor.h"

// Comment about the robot's wiring for Connor to see later
// The robot's motor labels are incorrect. 
// The motor labelled "RR (right rear)" is in fact the left rear motor
// The motor labelled "FR (front right)" is in fact the front left motor
// The motor labelled "RL (front left)" is in fact the rear right motor
// The motor that is not labelled is in fact the front right motor

//creates pwm instance

Teensy_PWM* instanceR;
Teensy_PWM* instanceL;


void setup()
{
  Serial.begin(115200);

  while (!Serial && millis() < 5000);


  Serial.print(F("\nStarting PWM_Basic using FlexTimers on dubs "));
  Serial.println(BOARD_NAME);
  Serial.println(TEENSY_PWM_VERSION);
  instanceR = new Teensy_PWM(4, 50, 7.5f);
  instanceL = new Teensy_PWM(5, 50, 7.5f);

}

void loop() {
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
        Serial.print("Parsed speeds: ");
        Serial.print(speed_1);
        Serial.print(", ");
        Serial.println(speed_2);
        //LMotor.set_speed(speed_1);
        //RMotor.set_speed(speed_2);
        float dutyCycle1 = 7.5f + speed_1*2.5f;
        //Swapped the sign of the speed to make the motors go in the same direction
        float dutyCycle2 = 7.5f - speed_2*2.5f;
        instanceR->setPWM(4, 50, dutyCycle1);
        instanceL->setPWM(5, 50, dutyCycle2);

      }
      else
      {
        Serial.println("Failed to parse numbers");
      }
  }



  



  
}
