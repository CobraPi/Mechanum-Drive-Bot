/*
  Motor library for l298 module
 The MIT License (MIT)
Copyright (c) 2014 Matouš Hýbl
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include <Encoder.h>

class Motor
{
  public:
    Motor(int motA, int motB, int encA, int encB);
    void drive(int speed);
    void move_rel(long ticks, int speed);
    void set_speed(int speed);
    void set_direction(bool dir);
    bool get_direction(); 
    void run();
    void poll();
    void calc_ticks(); 
    long get_ticks();
    void reset_ticks();
    
    protected:
        Encoder _encoder;
    
    private:
        int _motAPin, _motBPin, _encAPin, _encBPin;
        bool _prevEnChA, _prevEnChB, _enChA, _enChB, _dir;
        int _speed;
        long _ticks;
};

#endif
