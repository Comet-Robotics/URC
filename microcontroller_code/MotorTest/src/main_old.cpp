#include <Arduino.h>
#include "encoder_driver.h"
#include "pwm_driver.h"

#define LED 8
#define SLOWDOWN 26

// Variables for speed calculation
unsigned long lastTime = 0;
long lastPulseCount = 0;
int pwmValue = 0;


void setup() {
  Serial.begin(115200);
  
  // Initialize the motor speed reader
  initMotorSpeedReader();
  
  pinMode(SLOWDOWN, INPUT_PULLUP);  // Changed to GPIO 24 for Teensy
  pinMode(LED, OUTPUT);
  // Serial.println("Brushless Motor ESC Speed Reader Initialized (Teensy  )");
  // Serial.println("Connect ESC speed signal to GPIO 2");
  
  lastTime = millis();
  digitalWrite(LED, LOW); // Turn on built-in LED
}

//testing program
void loop() {

  
  // Calculate and display motor speed every 500ms
  unsigned long currentTime = millis();
  
  digitalWrite(LED, !digitalRead(LED)); // Toggle built-in LED for visual feedback

  if (currentTime - lastTime >= 500) {
    long currentPulseCount = readMotorPulses(FRONT_LEFT_MOTOR);
    long pulseDiff = currentPulseCount - lastPulseCount;
    float timeInterval = (currentTime - lastTime) / 1000.0; // Convert to seconds
    

    digitalWrite(LED, !digitalRead(LED)); // Toggle built-in LED for visual feedback


    // Calculate pulses per second (Hz)


    float pulsesPerSecond = pulseDiff / timeInterval;
    
    // Note: Using RISING interrupt, so multiply by 2 for actual pulse rate
    float rpm = (pulsesPerSecond * 2.0 * 60.0) / PULSES_PER_REVOLUTION;

    bool slowDown = !digitalRead(SLOWDOWN);  // Changed to GPIO 24 for Teensy
    
    // DEBUG: Read encoder pin state directly
    // int encoderPinState = digitalRead(MOTOR_ESC_SPEED_PINS[0]);
    
    // Serial.print("Total Pulses: ");
    // Serial.print(currentPulseCount);
    // Serial.print(" | Pulses/sec: ");
    // Serial.print(pulsesPerSecond);
    // Serial.print(" Hz | Estimated RPM: ");
    // Serial.print(rpm);
    // Serial.print(" | Total rotations: ");
    // Serial.print(currentPulseCount / PULSES_PER_REVOLUTION);
    for (int i = 0; i < MOTOR_COUNT; i++) {
      int encoderPinState = readMotorPulses(static_cast<MotorID>(i));
      Serial.print(" | Encoder pin ");
      Serial.print(i + 1);
      Serial.print(" state: ");
      Serial.println(encoderPinState);
    }
    
    lastPulseCount = currentPulseCount;
    lastTime = currentTime;

    //setMotorPWM(FRONT_LEFT_MOTOR, pwmValue);
    for(int i = 0; i < MOTOR_COUNT; i++) {
      setMotorPWM(static_cast<MotorID>(i), pwmValue);
    }

    if(pwmValue < PWM_MAX_VALUE && !slowDown) {
      pwmValue+=3;
      Serial.print(" Speeding Up ");
      digitalWrite(LED, HIGH);
    }
    else if(pwmValue > 0 && slowDown) {
      pwmValue-=3;
      Serial.print(" Slowing Down ");
      digitalWrite(LED, LOW);
    }
    Serial.println(" | PWM Value Set To: " + String(pwmValue));
  }
  
  // Your motor control code can go here
}