#include "AccelStepper.h"
#include "PIDController.h"
#include "AS5600.h"
#include <Wire.h>


AS5600 encoder(&Wire1);  //  use Wire1
AS5600 encoder2;  //  use Wire

// Define stepper motor connections and motor interface type. 
// Motor interface type must be set to 1 when using a driver:
#define step1Pin 2
#define dir1Pin 3
#define step2Pin 6
#define dir2Pin 7
#define dir3Pin 6
#define step3Pin 7
#define motorInterfaceType 1

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, step1Pin, dir1Pin);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, step2Pin, dir2Pin);
// AccelStepper stepper3 = AccelStepper(motorInterfaceType, step3Pin, dir3Pin);

void setup() {
  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);
  stepper2.setMaxSpeed(2000);
  stepper2.setAcceleration(1000);
  // stepper3.setMaxSpeed(2000);
  // stepper3.setAcceleration(1000);
  
  Serial.begin(115200);
  Serial.println("Ready for commands");
  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();


  Wire1.setSDA(14);
  Wire1.setSCL(15);
  Wire1.begin();

  encoder.begin(10);  //  set direction pin.
  encoder.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.


  encoder2.begin(19);  //  set direction pin.
  encoder2.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.

  int b = encoder.isConnected();
  int b2 = encoder2.isConnected();
  Serial.print("Connect: ");

  Serial.print(b);
  Serial.print(" ");
  Serial.println(b2);

  delay(1000);
}
unsigned long previousMillis = 0;
const long interval = 1000;

void loop() {
  static uint32_t lastTime = 0;

  //  set initial position
  encoder.getCumulativePosition();

  // //  update every 100 ms
  // //  should be enough up to ~200 RPM
  if (millis() - lastTime >= 100)
  {
    lastTime = millis();
    Serial.print(encoder.getCumulativePosition());
    Serial.print(",");
    Serial.print(encoder2.getCumulativePosition());
    Serial.println(",0");

  }

  //  just to show how reset can be used
  // if (as5600.getRevolutions() >= 10)
  // {
  //   as5600.resetPosition();
  // }


  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    char motor = command.charAt(0);
    long position = command.substring(2).toInt();
    
    switch (motor) {
      case '1':
        stepper.moveTo(position);
        // Serial.print("Moving motor 1 to position: ");
        // Serial.println(position);
        break;
      case '2':
        stepper2.moveTo(position);
        // Serial.print("Moving motor 2 to position: ");
        // Serial.println(position);
        break;
      // case '3':
      //   stepper3.moveTo(position);
      //   Serial.print("Moving motor 3 to position: ");
      //   Serial.println(position);
      //   break;
    }
  }
  
  stepper.run();
  stepper2.run();
  // stepper3.run();
}