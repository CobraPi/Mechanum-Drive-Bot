#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include <Encoder.h>

class Motor
{
  public:
    Motor(int motA, int motB);
    void drive(int speed); // signed pwm value [-255, 255] to indicate direction 
    void move_rel(long ticks, int speed);
    void set_speed(int speed);
    void set_direction(bool dir);
    bool get_direction(); 
    void run();
    
    private:
        int _motAPin, _motBPin, _encAPin, _encBPin;
        bool _prevEnChA, _prevEnChB, _enChA, _enChB, _dir;
        int _speed, _curSpeed;
        long _ticks;
};

#endif