#include <Arduino.h>
#include "Controller.h"


Controller::Controller() {}


void Controller::init() {
    //DDRC &= ~(1 << X_PIN); // set analog x-pin as input
    //DDRC &= ~(1 << Y_PIN); // set analog y-pin as input 
    pinMode(X_PIN, INPUT);
    pinMode(Y_PIN, INPUT); 
    //DDRD &= ~(1 << BTN_A_PIN); // set digital button-a-pin as input
    //DDRD &= ~(1 << BTN_B_PIN); // set digital button-b-pin as input
    //DDRD &= ~(1 << BTN_C_PIN); // set digital button-c-pin as input
    //DDRD &= ~(1 << BTN_D_PIN); // set digital button-d-pin as input
    //DDRD &= ~(1 << BTN_E_PIN); // set digital button-e-pin as input
    //DDRD &= ~(1 << BTN_F_PIN); // set digital button-f-pin as input
    pinMode(BTN_A_PIN, INPUT);
    pinMode(BTN_B_PIN, INPUT);
    pinMode(BTN_C_PIN, INPUT);
    pinMode(BTN_D_PIN, INPUT);
    pinMode(BTN_E_PIN, INPUT);
    pinMode(BTN_F_PIN, INPUT); 
    //DDRB &= ~(1 << PB0); // set digital button-xy-pin as input 
    pinMode(BTN_XY_PIN, INPUT); 
    _btnAdebounceTime = 0;
    _btnBdebounceTime = 0;
    _btnCdebounceTime = 0;
    _btnDdebounceTime = 0;
    _btnEdebounceTime = 0;
    _btnFdebounceTime = 0;
    _btnXYdebounceTime = 0;
    //pinMode(X_PIN, INPUT);
    //pinMode(Y_PIN, INPUT);
    _xWeight = DEFAULT_X_WEIGHT;
    _yWeight = DEFAULT_Y_WEIGHT;
    _alpha = ALPHA;
    _prevX = 0;
    _prevY = 0;
    _xTrim = 0;
    _yTrim = 0;
}

void Controller::poll() {
    _rawX = analogRead(X_PIN);
    _rawY = analogRead(Y_PIN);
    _x = _rawX - 512 + _xTrim;
    _y = _rawY - 512 + _yTrim;
    //_x = map(_rawX, 0, 1023, 0, 1023);
    //_y = map(_rawY, 0, 1023, 0, 1023); 
    // Read state of registers - buttons are active low         
    _btnA = digitalRead(BTN_A_PIN);//!(PIND & (1 << BTN_A_PIN));
    _btnB = digitalRead(BTN_B_PIN);//!(PIND & (1 << BTN_B_PIN));
    _btnC = digitalRead(BTN_C_PIN);//!(PIND & (1 << BTN_C_PIN));
    _btnD = digitalRead(BTN_D_PIN);//!(PIND & (1 << BTN_D_PIN));
    _btnE = digitalRead(BTN_E_PIN);//!(PIND & (1 << BTN_E_PIN));
    _btnF = digitalRead(BTN_F_PIN);//!(PIND & (1 << BTN_F_PIN));
    _btnXY = digitalRead(BTN_XY_PIN);//!(PINB & (1 << PB0));

    // Save the state of register bank for debouncing
    _prevA = _btnA;
    _prevB = _btnB;
    _prevC = _btnC;
    _prevD = _btnD;
    _prevE = _btnE;
    _prevF = _btnF;
    _prevXY = _btnXY;

    if(_btnA) {
        _btnAdebounceTime = micros();
    }
    if(_btnB) {
        _btnBdebounceTime = micros();
    }
    if(_btnC) {
        _btnCdebounceTime = micros();
    }
    if(_btnD) {
        _btnDdebounceTime = micros();
    }
    if(_btnE) {
        _btnEdebounceTime = micros();
    }
    if(_btnF) {
        _btnFdebounceTime = micros();
    }
    if(_btnXY) {
        _btnXYdebounceTime = micros();
    }

}

int16_t Controller::get_x() {
    return _x;
}

int16_t Controller::get_y() {
    return _y;
}

int16_t Controller::get_raw_x() {
    return _rawX;
}

int16_t Controller::get_raw_y() {
    return _rawY;
}

bool Controller::get_btn_a() {
    bool current = digitalRead(BTN_A_PIN);//!(PIND & (1 << BTN_A_PIN));
    return debounce(current, _prevA, _btnAdebounceTime);
}

bool Controller::get_btn_b() {
    bool current = digitalRead(BTN_B_PIN);//!(PIND & (1 << BTN_B_PIN));
    return debounce(current, _prevB, _btnBdebounceTime);
}
bool Controller::get_btn_c() {
    bool current = digitalRead(BTN_C_PIN);//!(PIND & (1 << BTN_C_PIN));
    return debounce(current, _prevC, _btnCdebounceTime);
}

bool Controller::get_btn_d() {
    bool current = digitalRead(BTN_D_PIN);//!(PIND & (1 << BTN_D_PIN));
    return debounce(current, _prevD, _btnDdebounceTime);
}

bool Controller::get_btn_e() {
    bool current = digitalRead(BTN_E_PIN);//!(PIND & (1 << BTN_E_PIN));
    return debounce(current, _prevE, _btnEdebounceTime);
}
bool Controller::get_btn_f() {
    bool current = digitalRead(BTN_F_PIN);//!(PIND & (1 << BTN_F_PIN));
    return debounce(current, _prevF, _btnFdebounceTime);
}

bool Controller::get_btn_xy() {
    bool current = digitalRead(BTN_XY_PIN);//!(PINB & (1 << PB0));
    return debounce(current, _prevXY, _btnXYdebounceTime);
}


void Controller::set_range_x(int16_t min=DEFAULT_MIN_X, int16_t max=DEFAULT_MAX_X) {  
    _minRangeX = min;
    _maxRangeX = max; 
}

void Controller::set_range_y(int16_t min=DEFAULT_MIN_Y, int16_t max=DEFAULT_MAX_Y) {
    _minRangeY = min;
    _maxRangeY = max;
}

void Controller::filter() {
    _x = analogRead(X_PIN) - 512 + _xTrim;
    _y = analogRead(Y_PIN) - 512 + _yTrim;
    _x = ((1 - _alpha) * _prevX) + (_alpha *_x);
    _y = ((1 - _alpha) * _prevY) + (_alpha * _y);
    _prevX = _x;
    _prevY = _y;
}

static bool Controller::debounce(bool current, bool previous, uint8_t &debounceTime) {
    if((current && previous) && 
      ((micros() - debounceTime) > BTN_DEBOUNCE_THRESHOLD)) {
          debounceTime = 0;
          return true;
      }
      else {
          return false;
      }
}

void Controller::trim_x(int16_t value) {
    _xTrim += value;
}

void Controller::trim_y(int16_t value) {
    _yTrim += value;
}