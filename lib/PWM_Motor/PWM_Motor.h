#ifndef PWM_Motor_h_
#define PWM_Motor_h_

#include <Arduino.h>

#define MAX_HARDWARE_PWM 4096

class PWM_Motor {

    public:
        PWM_Motor();
        void init(uint8_t pinCw, uint8_t pinCcw, uint8_t pinEn=0, bool enLogic=1); // Set the hardware pins
        
        void enable();
        void disable(); 
        
        void set_pwm(int16_t pwm); 
        void set_direction(bool direction); // Set the direction direction 
        
        // methods to return data 
        int16_t get_pwm();
        bool get_direction(); 
    
    protected: 
        int16_t _pwm;
    
    private:
        uint8_t _pinCw;
        uint8_t _pinCcw;
        uint8_t _pinEn;
        bool _direction;
        bool _enLogic;
};



#endif /* PWM_Motor_h_ */

