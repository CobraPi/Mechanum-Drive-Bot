#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include <Encoder.h>
#include <AFMotor.h>

class Motor
{
  public:
    Motor();
    void drive(int speed); // signed pwm value [-255, 255] to indicate direction 
    void move_rel(long ticks, int speed);
    void set_speed(int speed);
    void set_direction(bool dir);
    bool get_direction(); 
    void run();
    long get_ticks();

  protected:
    AF_DCMotor _motor;

    private:
        int _motAPin, _motBPin, _enAPin, _enBPin;
        bool _prevEnChA, _prevEnChB, _enChA, _enChB, _dir;
        int _speed, _curSpeed;
        long _ticks;
};

#endif