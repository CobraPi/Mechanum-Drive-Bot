#include "Motor.h"
//#include <Encoder.h>

Motor::Motor(int motAPin, int motBPin)
{
    _ticks = 0;
    _motAPin = motAPin;
    _motBPin = motBPin;
    pinMode(motAPin, OUTPUT);
    pinMode(motBPin, OUTPUT);
    _curSpeed = 0;
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
    if(speed > 0) {
        _dir = true;
        analogWrite(_motAPin, _speed);
        digitalWrite(_motBPin, 0);
    } 
    else {
        _dir = false;
        digitalWrite(_motAPin, 0);
        analogWrite(_motBPin, _speed);
    }
}



bool Motor::get_direction() {
    return _dir;
}