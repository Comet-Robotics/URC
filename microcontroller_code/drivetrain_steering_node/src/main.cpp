#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
// #include "Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS; 
Adafruit_StepperMotor *myMotor;
Adafruit_StepperMotor *myMotor2;


void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(5000);

  Serial.println("Setting up Wire");

  Wire.begin();
  Wire.setClock(100000);  // 100 kHz instead of 400 kHz
  
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
  myMotor->setSpeed(100);
  myMotor2->setSpeed(100);
  Serial.println("Setup complete");
  
}

void loop() {
  myMotor->onestep(FORWARD, DOUBLE);
  myMotor2->onestep(FORWARD, DOUBLE);
  
  // Serial.println("Stepping forward");
  // digitalWrite(LED_BUILTIN, HIGH);
  // myMotor->step(1000, FORWARD, DOUBLE);
  // myMotor2->step(1000, FORWARD, DOUBLE);
  // Serial.println("Stepping backward");
  // digitalWrite(LED_BUILTIN, LOW);
  // myMotor->step(1000, BACKWARD, DOUBLE);
  // myMotor2->step(1000, BACKWARD, DOUBLE);


  // Serial.print("Step time: ");
  // Serial.println(millis());
  delay(10);
}

