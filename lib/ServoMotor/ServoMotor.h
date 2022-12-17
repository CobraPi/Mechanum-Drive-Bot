#ifndef ServoMotor_h_
#define ServoMotor_h_

#include <Arduino.h>
#include <PWM_Motor.h>
#include <Encoder.h>
#include <PID.h>

#define ENCODER_PULSES_PER_REV 1000
#define RAD_PER_SEC_TO_RPM 9.55

enum ServoMotorMode{
    LOW_RESOLUTION,
    HIGH_RESOLUTION,
};

enum ServoMotorState {
    STOP,
    STOPPED,
    ACCELERATING,
    DECELERATING,
    RUNNING_CLOSED_LOOP,
    RUNNING_OPEN_LOOP
};

class ServoMotor : public PWM_Motor { 
   public: 
        ServoMotor();
        ServoMotor(uint8_t pinEnA, uint8_t pinEnB);
        
        // These four functions are meant to be called from the "setup" class
        void set_sample_time_ms(int16_t sampleTime); // called often in the main loop to update PID and speed
        void set_sample_time_us(int16_t sampleTime); // called often in the main loop to update PID and speed
        void set_wheel_circumference(float circumference);
        void set_encoder_pulses_per_rev(float pulses);
        
        float get_p();
        float get_i();
        float get_d();
        void get_pid(float *pidBuffer);
        void get_pid(float &kp, float &ki, float &kd);
        void get_pid_params(*pidBuffer); // For pid tuning
        void set_p_param(float kd);
        void set_i_param(float ki);
        void set_d_param(float kd);
        void set_pid(float *pidBuffer)
        void set_pid(float kp, float ki, float kd);
        void set_open_loop_pwm(int16_t pwm);
        
        float get_rpm();
        void set_rpm(float rpm); // set speed in rpm
        
        ServoMotorState get_state(); 
        void set_state(ServoMotorState) 
        
        void run(long ticks);
    
    protected:
        PID _pid;
    
    private:
        // Running modes
        ServoMotorMode _motorMode; 
        ServoMotorState _motorState; 
        bool _direction; 
        // Machine geometry
        _encoderPulsesPerRev;
        _wheelCircumference;
        float _targetRpm; // desired speed in rpm
        float _currentRpm; // current speed in rpm
        float _error;
        float _radPerSec;
        long _sampleStartTime, _sampleTime;
        long _curTicks, _prevTicks, _deltaTicks;     // encoder ticks
        long _totalDistanceMeters; // accumlative total of distance travelled 
        float _kp, _ki, _kd;

        int16_t _openLoopPwm; 
};

#endif //ServoMotor_h_