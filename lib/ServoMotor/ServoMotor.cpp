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
    _motorMode = LOW_RESOLUTION;
    _sampleTime = sampleTime;
    _sampleStartTime = millis();
}

void ServoMotor::set_sample_time_us(long sampleTime) {
    _motorMode = HIGH_RESOLUTION;
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

float ServoMotor::get_speed() {
    float speed;
    switch(_motorState) {
        case RUNNING_CLOSED_LOOP:
            speed = _currentRpm;
            break;
        case RUNNING_OPEN_LOOP:
            speed = _openLoopPwm;
    }
    return speed;
}

// Set the speed in rad/s
void ServoMotor::set_speed(float speed) {
    switch(_motorState) {
        case RUNNING_CLOSED_LOOP:
            _targetRpm = speed;
            break;
        case RUNNING_OPEN_LOOP:
            _openLoopPwm = (int16_t) speed;
    } 
}

ServoMotorState ServoMotor::get_state() {
    return _motorState;
}

void ServoMotor::set_state(ServoMotorState state) {
    _motorState = state;
}

// Called continuously in main loop
void ServoMotor::run(long ticks) {
    _curTicks = ticks;
    _sample_speed(); 
    switch(_motorState){
        case START:
            _accelerationTarget = _targetRpm;
            _targetRpm = 1;
            _motorState = ACCELERATING;
            _accelerationStartTime = millis();
            _sampleStartTime = set_start_time_us_ms(_motorMode); //_motorMode == LOW_RESOLUTION ? millis() : micros(); 
        case ACCELERATING:
            if(millis() - _accelerationStartTime > _accelerationTime) {
                _targetRpm++;
                _accelerationStartTime = set_start_time_us_ms(_motorMode);//_motorMode == LOW_RESOLUTION ? millis() : micros(); 
            }
            if(_currentRpm >= _targetRpm) {
                _motorState = RUNNING_CLOSED_LOOP;
            }
            break;
        case RUNNING_CLOSED_LOOP:
            _pwm = _pid.compute(_error); 
            break;
        case STOP:
            _decelerationStartTime = set_start_time_us_ms(_motorMode);
            _motorState = DECELERATING; 
        case DECELERATING:
        
        case STOPPED:
            _pwm = 0;
        break;
        case RUNNING_OPEN_LOOP:
            _pwm = _openLoopPwm; 
            break;
    }
    set_pwm(_pwm);
} 

static long ServoMotor::set_start_time_us_ms(ServoMotorMode mode) {
    return mode == LOW_RESOLUTION ? millis() : micros();
}

void ServoMotor::_sample_speed() {
    long tmr = set_start_time_us_ms(_motorMode);//_motorMode == LOW_RESOLUTION ? millis() : micros(); 
        if(tmr - _sampleStartTime > _sampleTime) {
            _deltaTicks = _curTicks - _prevTicks;
            _radPerSec = (_wheelCircumference * M_PI * _deltaTicks) / (_encoderPulsesPerRev * (float)(_sampleTime / 1000.0));
            _currentRpm = _radPerSec * RAD_PER_SEC_TO_RPM;
            _prevTicks = _curTicks;
            _error = _targetRpm - _currentRpm;
            _sampleStartTime = set_start_time_us_ms(_motorMode); // 
        }
}