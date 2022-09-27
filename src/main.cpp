/* Mechanum Drive Robot

Serial protocol = [, , direction(x) ]
*/



#include <Arduino.h>
//#include <Controller.h>
#include <Wire.h>
#include <SPI.h>
//#include <Motor.h>
//#include <Encoder.h>
//#include <DCMotorServo.h>
//#include <Motor.h>
//#include <PIDController.h>
#include <RF24.h>
#include <NRFLite.h>
#include <pin_map.h>
#include <radio_map.h>
#include <PWM_Motor.h>
#include <ServoMotor.h>
#include <CmdMessenger.h>
#include "../lib/CmdMessenger/CmdMessenger.h"

#define DC_MOTOR_ENCODER_1_INCH 1344
#define SIZE 32

long curTick, prevTick, timeout;
double kP, kI, kD, pOut, iOut, dOut, targetPos, targetSpeed, error, previousError, pidOut, currentPos;
int pos;
//PID motPID(&error, &pidOut, &targetPos, P, I, D, P_ON_M, DIRECT); 
//PID motPID(P, I, D, -255, 255);
//PIDController motPID;
//Motor m1();

//RF24 radio(2, 3); // CE, CSN
uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber = 1;
bool role = false; // True - RX-Mode  
char buffer[SIZE + 1];
uint8_t counter = 0;
uint16_t x, y;

//GMotor motor1(DRIVER2WIRE_2PWM, M1_A_PIN, M1_B_PIN);
//Motor motor2(4, 5);
NRFLite _radio;

#define FADE_DELAY 0
#define WIG_WAG_DELAY 35

const static uint8_t RADIO_ID = 0;       // Our radio's id.  The transmitter will send to this id.
const static uint8_t PIN_RADIO_CE = 9;
const static uint8_t PIN_RADIO_CSN = 10;

/*
Encoder Speed calculation
ω = 2πn/Nt

ω = angular speed (rad/s)
n = number of pulses
t = sampling period (s)
N = pulses per rotation - 1000
*/

ControlPacket _radioData;

//AF_DCMotor M_FR(1); // Front right M_RL
//AF_DCMotor M_FL(2); // Fornt left M_RL
//AF_DCMotor M_RR(3); // Rear right M_RL
//PWM_Motor mot;
ServoMotor M_RL(PIN_RL_EN_A, PIN_RL_EN_B); // Rear left M_RL
//ServoMotor M_RR(PIN_RR_EN_A, PIN_RR_EN_B);
//ServoMotor M_RL(PIN_RL_EN_A, PIN_RL_EN_B);
//AccelMotor M_RL(PIN_RL_EN_A, PIN_RL_EN_B);

//Encoder E_FL(PIN_FL_EN_A, PIN_FL_EN_B);
Encoder E_RL(PIN_RL_EN_A, PIN_RL_EN_B); // Rear left encoder
//Encoder E_FR(PIN_FR_EN_B, PIN_FR_EN_A);
//Encoder E_RR(PIN_RR_EN_A, PIN_RR_EN_B); // Rear right motor
#define DISPLAY_TIME_MS 300
#define MAX_RPM 150
bool radioMode; // TX = 1 RX = 0
bool dir = true;
int sign = -1;
int position;
int speed = 255;
bool alarm = false;
long pulseTime;

CmdMessenger ser = CmdMessenger(Serial,',',';','/');

void handle_set_pid(void);
void handle_set_rpm(void);
void handle_get_pid(void);
void handle_get_rpm(void);
void handle_get_pwm(void);
void handle_set_sample_time(void);
void handle_set_pwm(void);
void register_callbacks(void);

void makePayload(uint8_t i);

enum Motors {
    FRONT_LEFT,
    FRONT_RIGHT,
    REAR_LEFT,
    REAR_RIGHT
};

enum {
    SET_PID,
    SET_RPM,
    GET_RPM,
    GET_PID,
    GET_PWM,
    SET_SAMPLE_TIME,
    SET_PWM
};

void process_serial() {
  if(Serial.available() > 1) {
    char c = Serial.read();
    switch (c) {
      case SET_PID:
          break;
      case SET_RPM:
        alarm = false;
        speed = 0;
        break;

    }
  }
}
long pwmRunTime;
long encoderCounts;
long displayTime; // time in ms for serial printing
#define RPM 200
double radPerSec, rpm;
double pwm = 0;
bool direction;

void setup() {
    //mot.init(PIN_RL_PWM_CW, PIN_RL_PWM_CCW);
    //mot.set_duty(-100);
    // put your setup code here, to run once:
  //M_RR.init(PIN_RR_PWM_CW, PIN_RR_PWM_CCW); // initialize pwm M_RR pins
  Serial.begin(115200);
  //encoderCounts = E_RL.read();
  //displayTime = millis();
  M_RL.init(PIN_RL_PWM_CW, PIN_RL_PWM_CCW); // initialize pwm M_RL pins
  M_RL.set_encoder_pulses_per_rev(1000);
  M_RL.set_wheel_circumference(2.0);
  M_RL.set_sample_time(5);
  M_RL.set_pid(2, 0.7, 0.04);
  M_RL.set_rpm(0);
  //register_callbacks();
  pulseTime = millis();
}

void loop()
{
    if(millis() - pulseTime > 40) {
        if(rpm > MAX_RPM)
            dir= false;
        else if(rpm < -MAX_RPM)
            dir=true;
        if(dir)
            rpm++;
        else
            rpm--;

        M_RL.set_rpm(rpm);
        pulseTime = millis();
        Serial.println(rpm);
    }
    //ser.feedinSerialData();
    M_RL.tick(E_RL.read());
}


void register_callbacks(void) {
    ser.attach(SET_PID, handle_set_pid);
    ser.attach(SET_RPM, handle_set_rpm);
    ser.attach(GET_PID, handle_get_pid);
    ser.attach(GET_RPM, handle_get_rpm);
    ser.attach(GET_PWM, handle_get_pwm);
    ser.attach(SET_SAMPLE_TIME, handle_set_sample_time);
    ser.attach(SET_PWM, handle_set_pwm);
}

void handle_set_pwm(void) {
    int16_t motor = ser.readBinArg<int16_t>();
    int16_t pwm = ser.readBinArg<int16_t>();
    M_RL.set_duty(pwm);
}

void handle_set_pid(void) {
    int16_t motor = ser.readBinArg<int16_t>();
    float p = ser.readBinArg<float>();
    float i = ser.readBinArg<float>();
    float d = ser.readBinArg<float>();
    switch (motor) {
        case REAR_LEFT:
            M_RL.set_pid(p, i, d);
            break;
        case REAR_RIGHT:
            //M_RR.set_pid(p, i, d);
            break;
    }
};

void handle_set_rpm(void) {
    int16_t flRpm = ser.readBinArg<int16_t>();
    int16_t frRpm = ser.readBinArg<int16_t>();
    int16_t rlRpm = ser.readBinArg<int16_t>();
    int16_t rrRpm = ser.readBinArg<int16_t>();

    M_RL.set_rpm(rlRpm);
    //M_RR.set_rpm(rpm);
}

void handle_get_rpm(void) {
    int16_t motor = ser.readBinArg<int16_t>();
    float rpm;
    switch(motor) {
        case REAR_LEFT:
            rpm = M_RL.get_rpm();
            break;
        case REAR_RIGHT:
            //rpm = M_RR.get_rpm();
            break;
    }
    ser.sendCmdStart(GET_RPM);
    ser.sendCmdBinArg<float>(rpm);
    ser.sendCmdEnd();
}

void handle_get_pwm(void) {
    int16_t motor = ser.readBinArg<int16_t>();
    int16_t pwm;
    switch(motor) {
        case REAR_LEFT:
            pwm = M_RL.get_pwm();
            break;
        case REAR_RIGHT:
            //pwm = M_RR.get_pwm();
            break;
    }
    ser.sendCmdStart(GET_PWM);
    ser.sendCmdBinArg<int16_t>(pwm);
    ser.sendCmdEnd();
}

void handle_get_pid(void) {
    int16_t motor = ser.readBinArg<int16_t>();
    float p,i,d;//=M_RL.get_p();
    switch(motor) {
        case REAR_LEFT:
            p = M_RL.get_p();
            i = M_RL.get_i();
            d = M_RL.get_d();
            break;
        case REAR_RIGHT:
            //p = M_RR.get_p();
            //i = M_RR.get_i();
            //d = M_RR.get_d();
            break;
    }
    ser.sendCmdStart(GET_PID);
    ser.sendCmdBinArg<float>(p);
    ser.sendCmdBinArg<float>(i);
    ser.sendCmdBinArg<float>(d);
    ser.sendCmdEnd();
}

void handle_set_sample_time(void) {
    int16_t motor = ser.readBinArg<int16_t>();
    float sampleTime = ser.readBinArg<float>();
    switch(motor) {
        case REAR_LEFT:
            M_RL.set_sample_time(sampleTime);
            break;
        case REAR_RIGHT:
            //M_RR.set_sample_time(sampleTime);
            break;
    }
}

void makePayload(uint8_t i) {
  // Make a single payload based on position in stream.
  // This example employs function to save memory on certain boards.

  // let the first character be an identifying alphanumeric prefix
  // this lets us see which payload didn't get received
  buffer[0] = i + (i < 26 ? 65 : 71);
  for (uint8_t j = 0; j < SIZE - 1; ++j) {
    char chr = j >= (SIZE - 1) / 2 + abs((SIZE - 1) / 2 - i);
    chr |= j < (SIZE - 1) / 2 - abs((SIZE - 1) / 2 - i);
    buffer[j + 1] = chr + 48;
  }
}

/*

Demonstrates simple RX and TX operation.
Any of the Basic_RX examples can be used as a receiver.
Please read through 'NRFLite.h' for a description of all the methods available in the library.

Radio    Arduino
CE    -> 9
CSN   -> 10 (Hardware SPI SS)
MOSI  -> 11 (Hardware SPI MOSI)
MISO  -> 12 (Hardware SPI MISO)
SCK   -> 13 (Hardware SPI SCK)
IRQ   -> No connection
VCC   -> No more than 3.6 volts
GND   -> GND


#include <SPI.h>
#include <NRFLite.h>

const static uint8_t RADIO_ID = 0;       // Our radio's id.  The transmitter will send to this id.
const static uint8_t PIN_RADIO_CE = 2;
const static uint8_t PIN_RADIO_CSN = 3;



void setup()
{
    Serial.begin(115200);

    // By default, 'init' configures the radio to use a 2MBPS bitrate on channel 100 (channels 0-125 are valid).
    // Both the RX and TX radios must have the same bitrate and channel to communicate with each other.
    // You can run the 'ChannelScanner' example to help select the best channel for your environment.
    // You can assign a different bitrate and channel as shown below.
    //   _radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE2MBPS, 100) // THE DEFAULT
    //   _radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE1MBPS, 75)
    //   _radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE250KBPS, 0)
    
    
}

void loop()
{
}
*/