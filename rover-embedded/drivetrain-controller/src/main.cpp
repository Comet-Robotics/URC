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

Teensy_PWM* instanceRR;
Teensy_PWM* instanceLR;

Teensy_PWM* instanceRF;
Teensy_PWM* instanceLF;


void setup()
{
  Serial.begin(115200);

  while (!Serial && millis() < 5000);


  Serial.print(F("\nStarting PWM_Basic using FlexTimers on dubs "));
  Serial.println(BOARD_NAME);
  Serial.println(TEENSY_PWM_VERSION);
  instanceRR = new Teensy_PWM(4, 50, 7.5f);
  instanceLR = new Teensy_PWM(5, 50, 7.5f);
  instanceRF = new Teensy_PWM(6, 50, 7.5f);
  instanceLF = new Teensy_PWM(7, 50, 7.5f);

}

void loop() {
    char inputBuffer[32];
    
    if (Serial.available() > 0)
    {
      int bytesRead = Serial.readBytesUntil('\n', inputBuffer, sizeof(inputBuffer) - 1);
      
      inputBuffer[bytesRead] = '\0';
      
      int num1, num2,num3,num4;
      
      if (sscanf(inputBuffer, "%d:%d:%d:%d", &num1, &num2,&num3,&num4) == 4)
      {
    
        num1 = max(-100, min(100, num1));
        num2 = max(-100, min(100, num2));
        float speed_1 = (float)num1 /100.0;
        float speed_2 = (float)num2 /100.0;

        num3 = max(-100, min(100, num3));
        num4 = max(-100, min(100, num4));
        float speed_3 = (float)num3 /100.0;
        float speed_4 = (float)num4 /100.0;

        //LMotor.set_speed(speed_1);
        //RMotor.set_speed(speed_2);
        float dutyCycle1 = 7.5f + speed_1*2.5f;
        //Swapped the sign of the speed to make the motors go in the same direction
        float dutyCycle2 = 7.5f - speed_2*2.5f;
        float dutyCycle3 = 7.5f + speed_3*2.5f;
        //Swapped the sign of the speed to make the motors go in the same direction
        float dutyCycle4 = 7.5f - speed_4*2.5f;
        instanceRR->setPWM(4, 50, dutyCycle1);
        instanceLR->setPWM(5, 50, dutyCycle2);
        instanceRF->setPWM(6, 50, dutyCycle3);
        instanceLF->setPWM(7, 50, dutyCycle4);

      }
      else
      {
        Serial.println("Failed to parse numbers");
      }
  }



  



  
}
