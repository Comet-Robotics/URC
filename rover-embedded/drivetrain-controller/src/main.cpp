#define PWM_LOGLEVEL_ 3
#define DEBUG_MOTOR false
#include "Teensy_PWM.h"
#include "motor.h"

// Motor instances - corrected according to the actual wiring
Motor* leftRear;    // "RR" on rover
Motor* rightRear;   // "RL" on rover
Motor* leftFront;   // "FR" on rover
Motor* rightFront;  // "RF on rover

uint32_t lastTime = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);
    
    Serial.print(F("\nStarting PWM_Basic using FlexTimers on dubs "));
    Serial.println(BOARD_NAME);
    Serial.println(TEENSY_PWM_VERSION);
    pinMode(13, OUTPUT);
    // Initialize motors with correct pins and direction
    leftRear = new Motor(4);          // Pin 4
    rightRear = new Motor(5, true);   // Pin 5, inverted
    leftFront = new Motor(6);         // Pin 6
    rightFront = new Motor(7, true);  // Pin 7, inverted

    lastTime = millis();
}
bool heartbeat = false;
void loop() {
    char inputBuffer[32];
    
    // Check heartbeat
    if (millis() - lastTime > 1000) {
        Serial.println("Heartbeat not received");
        digitalWrite(13, HIGH);
        leftRear->stop();
        rightRear->stop();
        leftFront->stop();
        rightFront->stop();
        heartbeat = false;
        delay(100);
    }

    if (Serial.available() > 0) {
        int bytesRead = Serial.readBytesUntil('\n', inputBuffer, sizeof(inputBuffer) - 1);
        inputBuffer[bytesRead] = '\0';
        
        int speeds[4];
        
        if (heartbeat && sscanf(inputBuffer, "%d:%d:%d:%d", &speeds[0], &speeds[1], &speeds[2], &speeds[3]) == 4) {
            // Convert speeds from -100,100 range to -1,1 range and set motors
            leftRear->setSpeed(speeds[0] / 100.0f);
            rightRear->setSpeed(speeds[1] / 100.0f);
            leftFront->setSpeed(speeds[2] / 100.0f);
            rightFront->setSpeed(speeds[3] / 100.0f);
        }
        else if (strcmp(inputBuffer, "apple") == 0) {
            digitalWrite(13, LOW);

            lastTime = millis();
            heartbeat = true;
        }
        else {
            Serial.println("Failed to parse numbers");
        }
    }
}