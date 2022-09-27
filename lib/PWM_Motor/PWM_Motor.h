#ifndef PWM_Motor_h_
#define PWM_Motor_h_

#include <Arduino.h>

#define MAX_DUTY 255

class PWM_Motor {

    public:
        PWM_Motor();
        void init(uint8_t pinCw, uint8_t pinCcw); // Set the hardware pins
        void set_duty(int16_t duty);    // [-1000,1000] mapped range
        uint8_t get_duty();
        uint16_t get_speed();

        int16_t _duty;
    private:
        uint8_t _pinCw;
        uint8_t _pinCcw;
        uint8_t _pinEn;
        uint16_t _speed;
};



#endif /* PWM_Motor_h_ */

