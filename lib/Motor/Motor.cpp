#include "Motor.h"
//#include <Encoder.h>

Motor::Motor(int motAPin, int motBPin, int encAPin, int encBPin)
    : _encoder(encAPin, encBPin) {
    _ticks = 0;
    _motAPin = motAPin;
    _motBPin = motBPin;
    _encAPin = encAPin;
    _encBPin = encBPin;
    pinMode(motAPin, OUTPUT);
    pinMode(motBPin, OUTPUT);
    //pinMode(encA, INPUT);
    //pinMode(encB, INPUT);
}

void Motor::set_speed(int speed) {
    _speed = speed;
}

void Motor::set_direction(bool dir) {
    _dir = dir;
}

void Motor::move_rel(long deltaTicks, int speed) {
    if(deltaTicks > 0) {
        _dir = true;
    }
    else {
        _dir = false;
    }
    long tick_offset = abs(_ticks);
    //_ticks = 0; 
    _speed = speed; 
    while(abs(Motor::get_ticks()) - tick_offset < abs(deltaTicks)) {
        Motor::run();
    }
}

void Motor::run() {
    Motor::get_ticks(); 
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
    _speed = speed;
    Motor::run();
}


void Motor::reset_ticks() {
    _ticks = 0;
}

void Motor::poll() {
    _ticks = _encoder.read();
}

long Motor::get_ticks() {
    _ticks = _encoder.read(); 
    return _ticks;
}

bool Motor::get_direction() {
    return _dir;
}