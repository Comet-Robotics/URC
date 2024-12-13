#ifndef MOTOR_H
#define MOTOR_H

#include "Teensy_PWM.h"

class Motor {
private:
    Teensy_PWM* pwmInstance;
    int pin;
    bool invertDirection;
    float baseSpeed = 7.5f;
    float maxSpeedDelta = 2.5f;

public:
    Motor(int pinNumber, bool invert = false) 
        : pin(pinNumber), invertDirection(invert) {
        pwmInstance = new Teensy_PWM(pin, 50, baseSpeed);
    }

    ~Motor() {
        delete pwmInstance;
    }

    void setSpeed(float speed) {
        // Clamp speed between -1 and 1
        speed = max(-1.0f, min(1.0f, speed));

        Serial.println("Setting speed to " + String(speed) + "For Pin "+ String(pin));
        // Invert speed if necessary
        if (invertDirection) {
            speed = -speed;
        }

        // Calculate duty cycle (7.5 ± 2.5)
        float dutyCycle = baseSpeed + (speed * maxSpeedDelta);
        pwmInstance->setPWM(pin, 50, dutyCycle);
    }

    void stop() {
        pwmInstance->setPWM(pin, 50, baseSpeed);
    }
};

#endif