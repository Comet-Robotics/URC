#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
// #include "Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS; 
Adafruit_StepperMotor *myMotor;
Adafruit_StepperMotor *myMotor2;


void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(5000); // delay so i can see logs :p

  Serial.println("Setting up Wire");

  Wire.setSDA(0);  
  Wire.setSCL(1);
  Wire.begin();

  Wire.setClock(100000);  // 100 kHz instead of 400 kHz (default)
  
  AFMS = Adafruit_MotorShield();
  bool success = AFMS.begin(1600, &Wire);
  if (!success) {
    Serial.println("Could not find Motor Shield. Check wiring.");
  }
  else {
    Serial.println("Motor Shield found");
  }

  myMotor = AFMS.getStepper(200, 1);
  myMotor2 = AFMS.getStepper(200, 2);
  myMotor->setSpeed(15);
  myMotor2->setSpeed(15);
  Serial.println("Setup complete");
  
}

void loop() {
  myMotor->onestep(FORWARD, SINGLE);
  // myMotor2->onestep(FORWARD, SINGLE);
  
  // Serial.println("Stepping forward");
  // digitalWrite(LED_BUILTIN, HIGH);
  // myMotor->step(1000, FORWARD, SINGLE);
  // myMotor2->step(1000, FORWARD, SINGLE);
  // Serial.println("Stepping backward");
  // digitalWrite(LED_BUILTIN, LOW);
  // myMotor->step(1000, BACKWARD, SINGLE);
  // myMotor2->step(1000, BACKWARD, SINGLE);


  // Serial.print("Step time: ");
  // Serial.println(millis());
  delay(10);
}

