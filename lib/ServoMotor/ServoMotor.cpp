#include "ServoMotor.h"


/*
    Default constructor
*/
ServoMotor::ServoMotor() {
    // initialize encoder with supplied pins  
    _pid.setLimits(-MAX_HARDWARE_PWM, MAX_HARDWARE_PWM);
    _pid.setPID(0,0,0);
    _direction = true;
}
/*
    For use when implementing own encoder readings
*/
ServoMotor::ServoMotor(uint8_t pinEnA, uint8_t pinEnB) {
    _pinEnA = pinEnA;
    _pinEnB = pinEnB; 
    // initialize encoder with supplied pins  
    _pid.setLimits(-MAX_HARDWARE_PWM, MAX_HARDWARE_PWM);
    _pid.setPID(0,0,0);
    _direction = true;
}

/* 
    Set's the speed sample time for PID calculation
*/
void ServoMotor::set_sample_time_ms(long sampleTime) {
    _mode = LOW_RESOLUTION;
    _sampleTime = sampleTime;
    _sampleStartTime = millis();
}

void ServoMotor::set_sample_time_us(long sampleTime) {
    _mode = HIGH_RESOLUTION;
    _sampleTime = sampleTime;
    _sampleStartTime = micros();
}

void ServoMotor::set_wheel_circumference(float circumference) {
    _wheelCircumference = circumference;
}

void ServoMotor::set_encoder_pulses_per_rev(uint16_t pulses) {
    _encoderPulsesPerRev = pulses;
}

void ServoMotor::get_pid(float *pidBuffer) {
    pidBuffer[0] = _pid.getP();
    _ki = _pid.getI();
    _kd = _pid.getD();
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

void ServoMotor::set_pid(float *pidBuffer) {
    _kp = pidBuffer[0];
    _ki = pidBuffer[1];
    _kd = pidBuffer[2];
}
void ServoMotor::set_pid(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _pid.setPID(_kp, _ki, _kd);
}



float ServoMotor::get_rpm() {
    return _currentRpm;
}

// Set the speed in rad/s
void ServoMotor::set_rpm(float rpm) {
    _targetRpm = rpm;
}

// Called continuously in main loop
void ServoMotor::run(long ticks) {
    _curTicks = ticks;
    switch(_motorState){
        case STOP:
        case STOPPED:
            _pwm = 0;
            break; 
        case ACCELERATING:
        case DECELERATING:
            break;
        case RUNNING_CLOSED_LOOP:
            long timer = _motorMode == LOW_RESOLUTION ? millis() : micros(); 
            if(timer - _sampleStartTime > _sampleTime) {
                _deltaTicks = _curTicks - _prevTicks;
                _radPerSec = (_wheelCircumference * M_PI * _deltaTicks) / (_encoderPulsesPerRev * (float)(_sampleTime / 1000.0));
                _currentRpm = _radPerSec * RAD_PER_SEC_TO_RPM;
                _prevTicks = _curTicks;
                _error = _targetRpm - _currentRpm;
                _pwm = _pid.compute(_error);
                _sampleStartTime = _motorMode == LOW_RESOLUTION ? millis() : micros(); 
            }
            break;
        case RUNNING_OPEN_LOOP:
            _pwm = _openLoopPwm; 
            break;
    }
    set_pwm(_pwm);
} 
