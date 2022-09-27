#include "PWM_Motor.h"

PWM_Motor::PWM_Motor() {}

// Initialize the M_RL with the specified pins (H-Bridge)
void PWM_Motor::init(uint8_t pinCw, uint8_t pinCcw) {   
    _pinCw = pinCw;
    _pinCcw = pinCcw;
    pinMode(_pinCw, OUTPUT);
    pinMode(_pinCcw, OUTPUT);
}

// Set the duty cycle of the M_RL - function is set to accept
// values in the range: [-1000,1000], supporting reverse functionality
void PWM_Motor::set_duty(int16_t duty) {
    _duty = duty;
    _speed = map(abs(_duty), 0, 1000, 0, MAX_DUTY);
    if (_duty > 0) {
        analogWrite(_pinCw, _speed);
        digitalWrite(_pinCcw, LOW);
    }
    else if(_duty < 0) {
        analogWrite(_pinCcw, _speed);
        digitalWrite(_pinCw, LOW);
    }
    else {
        digitalWrite(_pinCw, LOW);
        digitalWrite(_pinCcw, LOW);
    }
}


// Return the current duty cycle of the M_RL
uint8_t PWM_Motor::get_duty() {
    return _duty;
}

// Returns mapped duty to pwm value
uint16_t PWM_Motor::get_speed() {
    return _speed;
}