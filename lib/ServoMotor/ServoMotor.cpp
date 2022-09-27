#include "ServoMotor.h"


ServoMotor::ServoMotor(uint8_t pinEnA, uint8_t pinEnB) {
    // initialize encoder with supplied pins  
    //_encoder = Encoder(); 
    _kp = P;
    _ki = I;
    _kd = D;
    _pid.setLimits(-1000, 1000);
    _pid.setPID(_kp,_ki,_kd);
    _sampleStartTime = millis();
    _direction = true;
}

void ServoMotor::get_pid(float &kp, float &ki, float &kd) {
    _kp = _pid.getP();
    _ki = _pid.getI();
    _kd = _pid.getD();
    kp = _kp;
    ki = _ki;
    kd = _kd;
}

float ServoMotor::get_p() {
    _kp = _pid.getP();
    return _kp;
}

float ServoMotor::get_i() {
    _ki = _pid.getI();
    return _ki;
}

float ServoMotor::get_d() {
    _kd = _pid.getD();
    return _kd;
}

void ServoMotor::set_direction(bool direction) {
    _direction = direction;
}

void ServoMotor::set_pid(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _pid.setPID(_kp, _ki, _kd);
}

// Set the speed in rad/s
void ServoMotor::set_rpm(float rpm) {
    _targetRpm = rpm;
}

float ServoMotor::get_rpm() {
    return _currentRpm;
}

int16_t ServoMotor::get_pwm() {
    return _duty;
}

long ServoMotor::get_ticks() {
    return _deltaTicks;
}

// Called continuously in main loop
void ServoMotor::tick(long ticks) {
    _curTicks = ticks;
    if(millis() - _sampleStartTime > _sampleTime) {
        _deltaTicks = _curTicks - _prevTicks;
        _radPerSec = (_wheelCircumference * M_PI * _deltaTicks) / (_encoderPulsesPerRev * (float)(_sampleTime / 1000.0));
        _currentRpm = _radPerSec * RAD_PER_SEC_TO_RPM;
        _prevTicks = _curTicks;
        _sampleStartTime = millis();
        _error = _targetRpm - _currentRpm;
        _pwmSpeed = _pid.compute(_error);
        set_duty(_pwmSpeed);
    }
}

int16_t ServoMotor::get_error() {
    return _error;
}

void ServoMotor::set_sample_time(float sampleTime) {
    _sampleTime = sampleTime;
}

void ServoMotor::set_wheel_circumference(float circumference) {
    _wheelCircumference = circumference;
}

void ServoMotor::set_encoder_pulses_per_rev(uint16_t pulses) {
    _encoderPulsesPerRev = pulses;
}