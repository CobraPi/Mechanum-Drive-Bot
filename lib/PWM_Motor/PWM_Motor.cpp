#include "PWM_Motor.h"

PWM_Motor::PWM_Motor() {}

// Initialize the M_RL with the specified pins (H-Bridge)
void PWM_Motor::init(uint8_t pinCw, uint8_t pinCcw, uint8_t pinEn=0, bool enLogic=1) {   
    _pinCw = pinCw;
    _pinCcw = pinCcw;
    pinMode(_pinCw, OUTPUT);
    pinMode(_pinCcw, OUTPUT);
    if(!pinEn) {
        _pinEn = pinEn;
        _enLogic = enLogic;
        pinMode(_pinEn, OUTPUT);
    }
}

void PWM_Motor::enable() {
    digitalWrite(_pinEn, _enLogic);
}

void PWM_Motor::disable() {
    digitalWrite(_pinEn, !_enLogic);
}
bool PWM_Motor::get_direction() {
    return _direction;
}

// Return the current duty cycle of the M_RL
int16_t PWM_Motor::get_pwm() {
    return _pwm;
}

void PWM_Motor::set_direction(bool direction) {
    _direction = direction;
}

// Set the duty cycle of the M_RL - function is set to accept
// values in the range: [-1000,1000], supporting reverse functionality
void PWM_Motor::set_pwm(int16_t pwm, bool direction=true) {
    if(pwm < 0 || !direction) {
        _pwm = abs(pwm);
        _direction = direction;
    }
    // The order of these is important
    if(_pwm == 0) {
        digitalWrite(_pinCw, _pwm);
        digitalWrite(_pinCcw, _pwm);
    }
    else if (_direction) {
        analogWrite(_pinCw, _pwm);
        digitalWrite(_pinCcw, LOW);
    }
    else if(!direction) {
        analogWrite(_pinCcw, _pwm);
        digitalWrite(_pinCw, LOW);
    }
}





