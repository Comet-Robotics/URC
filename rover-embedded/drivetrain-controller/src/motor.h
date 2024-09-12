#include <Teensy_PWM.h>
class Motor {      
    public:            
        float speed;       
        int pinToUse;
        float frequency;
        Teensy_PWM* instance;
        Motor();

        Motor(int pin, int frequncey);

        float set_speed (float speed);

};