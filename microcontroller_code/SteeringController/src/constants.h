#include <Arduino.h>
#ifndef CONSTANTS_H
#define CONSTANTS_H

#define MOTOR_COUNT 4


struct Motor{
    // Each motor has a step per rev of 200, and a gearbox of 14:1 for an steps per rev of 2800 on the output.
    int AIN1;
    int AIN2;
    int BIN1;
    int BIN2;
    int sleepPin;

    int targetStep = 0;
    int step = 0;

    int stepsTaken = 0; // Mental image of where we are

    Motor() {}
    Motor(int a1, int a2, int b1, int b2, int nSleep)
      : AIN1(a1), AIN2(a2), BIN1(b1), BIN2(b2), sleepPin(nSleep),
        targetStep(0), step(0), stepsTaken(0) {}

    void setup(){
        pinMode(AIN1, OUTPUT);
        pinMode(AIN2, OUTPUT);
        pinMode(BIN1, OUTPUT);
        pinMode(BIN2, OUTPUT);
        pinMode(sleepPin, OUTPUT);

        digitalWrite(sleepPin, HIGH); // Wake up the motor driver
    }

    void setStep(int step) {
        switch(step) {
            case 0: digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
                    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); break;

            case 1: digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
                    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); break;

            case 2: digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
                    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); break;

            case 3: digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
                    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); break;
        }
    }

    void setDeg(int deg)
    {
        int stepsPerRev = 2800; // 200 steps/rev * 14:1 gearbox
        targetStep = (deg * stepsPerRev) / 360; // Convert degrees to steps
    }

    void takeStep()
    {
        if (stepsTaken < targetStep) {
            // moving forward
            step = (step + 1) % 4;
            setStep(step);
            stepsTaken++;
        } else if (stepsTaken > targetStep) {
            // moving backward
            step = (step - 1 + 4) % 4;
            setStep(step);
            stepsTaken -= 1;
        }
    }

};

extern int MOTOR_PINS[MOTOR_COUNT][5]; // 2 for A, 2 for B, 1 for sleep


#endif