#include <Teensy_PWM.h>
class Motor {      
    public:            
        float speed;       
        int pinToUse;
        float frequency;

        Motor(int pin, int frequncey);

        float set_speed (Teensy_PWM* PWM_Instance,float speed);

};