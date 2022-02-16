#ifndef Controller_H
#define Controller_H

#define DEFAULT_MIN_X 0
#define DEFAULT_MIN_Y 0
#define DEFAULT_MAX_X 1023
#define DEFAULT_MAX_Y 1023

#define DEFAULT_X_WEIGHT 50
#define DEFAULT_Y_WEIGHT 50

#define ALPHA (double)0.2 // 0.2*16 = 3 
#define ALPHA_SCALED (x) ((uint16_t)(x << 4))  //48

#define X_PIN A0
#define Y_PIN A1

#define BTN_DEBOUNCE_THRESHOLD 50 // button debounce time in us
#define BTN_A_PIN 2
#define BTN_B_PIN 3
#define BTN_C_PIN 4
#define BTN_D_PIN 5
#define BTN_E_PIN 6
#define BTN_F_PIN 7
#define BTN_XY_PIN 8 

enum Button {BTN_A=BTN_A_PIN};

class Controller {

private:
    int16_t _minRangeX;
    int16_t _minRangeY;
    int16_t _maxRangeX;
    int16_t _maxRangeY;

    double _alpha;
    uint8_t _xWeight;
    uint8_t _yWeight; 
    int16_t _prevX;
    int16_t _prevY;
    int16_t _x;
    int16_t _y;
    int16_t _rawX;
    int16_t _rawY;
    int16_t _xTrim;
    int16_t _yTrim;
    
    byte _prevStateBankD; // 8-bit snapshot of digital pin inputs on bank D
    byte _prevStateBankB; // 8-bit snapshot of digital pin inputs on bank B

     /* Buttons */
    bool _btnA, _prevA;
    bool _btnB, _prevB;
    bool _btnC, _prevC;
    bool _btnD, _prevD;
    bool _btnE, _prevE;
    bool _btnF, _prevF;
    bool _btnXY, _prevXY;
    
    uint8_t _btnAdebounceTime;
    uint8_t _btnBdebounceTime;
    uint8_t _btnCdebounceTime;
    uint8_t _btnDdebounceTime;
    uint8_t _btnEdebounceTime;
    uint8_t _btnFdebounceTime;
    uint8_t _btnXYdebounceTime;
    
protected:
    static bool debounce(bool current, bool previous, uint8_t &debounceTime);

public:
    Controller();
    void init(); 
    void poll();
    void filter();
    int16_t get_x();
    int16_t get_y();
    int16_t get_raw_x();
    int16_t get_raw_y();
    void set_range_x(int16_t max, int16_t min);
    void set_range_y(int16_t max, int16_t min);
    bool get_btn_a();
    bool get_btn_b();
    bool get_btn_c();
    bool get_btn_d();
    bool get_btn_e();
    bool get_btn_f();
    bool get_btn_xy();
    void trim_x(int16_t value);
    void trim_y(int16_t value);
};






#endif //Controller_H