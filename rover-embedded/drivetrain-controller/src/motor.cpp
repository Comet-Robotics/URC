#include "motor.h"
#include <Teensy_PWM.h>
Motor::Motor(int pin, int frequency) {
    this->pinToUse = pin;
    this->frequency = frequency;
    this->instance = new Teensy_PWM(pinToUse, frequency, 7.5f);
}


float Motor::set_speed(float speed){
    this->speed = speed;
    float dutyCycle = 7.5f + speed*2.5f;

    Serial.println(dutyCycle);
    Serial.println(this->frequency);

    this->instance->setPWM(this->pinToUse, this->frequency, dutyCycle);
}