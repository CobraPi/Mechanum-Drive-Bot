#include "Motor.h"

Motor::Motor() {
    _ticks = 0;
    //pinMode(encA, INPUT);
    //pinMode(encB, INPUT);
}

void Motor::set_speed(int speed) {
    _speed = speed;
}

void Motor::set_direction(bool dir) {
    _dir = dir;
}


void Motor::run() {
    if(_dir) {
        analogWrite(_motAPin, _speed);
        digitalWrite(_motBPin, 0);
    }
    else {
        digitalWrite(_motAPin, 0);
        analogWrite(_motBPin, _speed);
    }
} 

void Motor::drive(int speed) {
    _speed = abs(speed);
    _motor.setSpeed(_speed);
    
    if(speed > 0) 
    {
        if(_dir) 
        {
            _motor.run(FORWARD);
        }
        else {
            _motor.run(BACKWARD);
        }
    }
    else if(speed < 0) {
        if(_dir) {
            _motor.run(BACKWARD);
        }
        else {
            _motor.run(FORWARD);
        }
    }
}

long Motor::get_ticks() {
    return _ticks;
}


bool Motor::get_direction() {
    return _dir;
}