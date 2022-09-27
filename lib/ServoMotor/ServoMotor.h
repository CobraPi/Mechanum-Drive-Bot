#ifndef ServoMotor_h_
#define ServoMotor_h_

#include <Arduino.h>
#include <PWM_Motor.h>
#include <Encoder.h>
#include <PID.h>

#define ENCODER_PULSES_PER_REV 1000
#define RAD_PER_SEC_TO_RPM 9.55

#define P 0.9
#define I 0.01
#define D 0

class ServoMotor : public PWM_Motor { 

  
   public: 
        ServoMotor(uint8_t pinEnA, uint8_t pinEnB);
        void set_direction(bool direction);
        void set_rpm(float rpm); // set speed in rpm
        float get_rpm();
        int16_t get_pwm();
        void tick(long ticks);
        void set_sample_time(float sampleTime); // called often in the main loop to update PID and speed
        long get_ticks(); // returns delta ticks
        void set_pid(float kp, float ki, float kd);
        void get_pid(float &kp, float &ki, float &kd);
        float get_p();
        float get_i();
        float get_d();
        int16_t get_error();

    protected:
        PID _pid;
    private:
        bool _direction; 
        int16_t _pwmSpeed; // [-100, 100] 
        float _targetRpm; // desired speed in rpm
        float _currentRpm; // current speed in rpm
        float _error;
        float _radPerSec;
        float _sampleTime;
        long _sampleStartTime;
        long _curTicks, _prevTicks, _deltaTicks;     // encoder ticks
        long _distance; // distance traveled in feet
        float _kp, _ki, _kd;
};

#endif //ServoMotor_h_