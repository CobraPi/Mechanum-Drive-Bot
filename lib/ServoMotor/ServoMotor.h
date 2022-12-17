#ifndef ServoMotor_h_
#define ServoMotor_h_

#include <Arduino.h>
#include <PWM_Motor.h>
#include <Encoder.h>
#include <PID.h>

/*
Encoder Speed calculation
ω = 2πn/Nt

ω = angular speed (rad/s)
n = number of pulses
t = sampling period (s)
N = pulses per rotation  
*/

#define ENCODER_PULSES_PER_REV 1000
#define RAD_PER_SEC_TO_RPM 9.55

#define SET_START_TIME_US_MS(motorMode)(((motorMode) == LOW_RESOLUTION) ? millis() : micros()) 

enum ServoMotorMode{
    LOW_RESOLUTION,
    HIGH_RESOLUTION,
};

enum ServoMotorState {
    STOP,
    STOPPED,
    START, 
    ACCELERATING,
    DECELERATING,
    RUNNING_CLOSED_LOOP,
    RUNNING_OPEN_LOOP
};

class ServoMotor : public PWM_Motor { 
    // Sets the proper unit of time measurement  
   public: 
        ServoMotor();
        ServoMotor(uint8_t pinEnA, uint8_t pinEnB);
        
        // These four functions are meant to be called from the "setup" class
        void set_sample_time_ms(long sampleTime); // called often in the main loop to update PID and speed
        void set_sample_time_us(long sampleTime); // called often in the main loop to update PID and speed
        void set_wheel_circumference(float circumference);
        void set_encoder_pulses_per_rev(uint16_t pulses);
        
        float get_p();
        float get_i();
        float get_d();
        void get_pid(float *pidBuffer);
        void get_pid(float &kp, float &ki, float &kd);
        void get_pid_params(float *pidBuffer); // For pid tuning
        void set_p_param(float kd);
        void set_i_param(float ki);
        void set_d_param(float kd);
        void set_pid(float *pidBuffer);
        void set_pid(float kp, float ki, float kd);
        void set_open_loop_pwm(int16_t pwm);
        
        float get_speed();
        void set_speed(float speed); // set speed in rpm
        
        ServoMotorState get_state(); 
        void set_state(ServoMotorState state); 
        
        void run(long ticks);
    
    protected:
        PID _pid;
    
    private:
        void _sample_speed();
        static long set_start_time_us_ms(ServoMotorMode mode);
        // Running modes
        ServoMotorMode _motorMode; 
        ServoMotorState _motorState; 
        bool _direction; 
        // Machine geometry
        int16_t _encoderPulsesPerRev;
        float _wheelCircumference;
        float _targetRpm; // desired speed in rpm
        float _currentRpm; // current speed in rpm
        float _error;
        float _radPerSec;
        long _sampleStartTime, _sampleTime;
        long _curTicks, _prevTicks, _deltaTicks;     // encoder ticks
        long _totalDistanceMeters; // accumlative total of distance travelled 
        uint8_t _pinEnA, _pinEnB;
        unsigned long _accelerationStartTime, _accelerationTime; 
        unsigned long _decelerationStartTime, _decelerationTime; 
        float _accelerationTarget;
        float _kp, _ki, _kd;
        int16_t _openLoopPwm; 
};

#endif //ServoMotor_h_