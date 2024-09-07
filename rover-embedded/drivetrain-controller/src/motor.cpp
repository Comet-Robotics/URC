#include "motor.h"
#include <Teensy_PWM.h>
Motor::Motor(int pin, int frequncey) {
    this->pinToUse = pin;
    this->frequency = frequncey;
}


float Motor::set_speed(Teensy_PWM* PWM_Instance,float speed){
    this->speed = speed;
    float dutyCycle = 5.0f + speed*2.5f;

    PWM_Instance->setPWM(this->pinToUse, this->frequency, dutyCycle);
}