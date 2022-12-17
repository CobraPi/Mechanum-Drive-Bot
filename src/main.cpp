/* Mechanum Drive Robot

Serial protocol = [, , direction(x) ]
*/



#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <pin_map.h>
#include <PWM_Motor.h>
#include <ServoMotor.h>
#include <CmdMessenger.h>
#include <Encoder.h>




ServoMotor M_RL(PIN_RL_EN_A, PIN_RL_EN_B); // Rear left M_RL
Encoder E_RL(PIN_RL_EN_A, PIN_RL_EN_B); // Rear left encoder

CmdMessenger ser = CmdMessenger(Serial,',',';','/');

void handle_get_pid_params(void);
void handle_set_pid_params(void);
void handle_get_motor_params(void);
void handle_set_motor_params(void);

void handle_get_pid(void);
void handle_set_pid(void);
void handle_get_rpm(void);
void handle_set_rpm(void);

void handle_get_pwm(void);
void handle_set_sample_time(void);
void handle_set_pwm(void);
void register_callbacks(void);


enum {
    SET_PID,
    SET_RPM,
    GET_RPM,
    GET_PID,
    GET_PWM,
    SET_SAMPLE_TIME,
    SET_PWM,
    SET_MOTOR_PARAMS,
    GET_MOTOR_PARAMS,
};


void setup() {
  Serial.begin(115200);
  M_RL.init(PIN_RL_PWM_CW, PIN_RL_PWM_CCW); // initialize pwm M_RL pins
  M_RL.set_encoder_pulses_per_rev(1000);
  M_RL.set_wheel_circumference(2.0);
  M_RL.set_sample_time_us(300);
  M_RL.set_pid(5, 3, 0.04);
  M_RL.set_state(STOPPED);
  register_callbacks();
}

void loop()
{
    ser.feedinSerialData();
    M_RL.run(E_RL.read());
}

void register_callbacks(void) {
    ser.attach(SET_PID, handle_set_pid);
    ser.attach(SET_RPM, handle_set_rpm);
    ser.attach(GET_PID, handle_get_pid);
    ser.attach(GET_RPM, handle_get_rpm);
    ser.attach(GET_PWM, handle_get_pwm);
    ser.attach(SET_SAMPLE_TIME, handle_set_sample_time);
    ser.attach(SET_PWM, handle_set_pwm);
    ser.attach(SET_MOTOR_PARAMS, handle_set_motor_params);
    ser.attach(GET_MOTOR_PARAMS, handle_get_motor_params);
}

void handle_get_motor_params(void) {
    int16_t params[2]; 
    params[0] = (int16_t) M_RL.get_state(); 
    params[1] = M_RL.get_speed();
    ser.sendCmdStart(GET_MOTOR_PARAMS); 
    for(int i=0; i<2; i++) ser.sendCmdBinArg<int16_t>(params[i]);
    ser.sendCmdEnd();
}

void handle_set_motor_params(void) {
    ServoMotorState state = (ServoMotorState) ser.readBinArg<int16_t>();
    int16_t speed = ser.readBinArg<int16_t>();
    M_RL.set_state(state);
    M_RL.set_speed(speed);
}

void handle_set_pwm(void) {
    int16_t pwm = ser.readBinArg<int16_t>();
    M_RL.set_pwm(pwm);
}

void handle_set_pid(void) {
    float p = ser.readBinArg<float>();
    float i = ser.readBinArg<float>();
    float d = ser.readBinArg<float>();
    M_RL.set_pid(p, i, d);
};

void handle_set_rpm(void) {
    int16_t rlRpm = ser.readBinArg<int16_t>();
    M_RL.set_state(RUNNING_CLOSED_LOOP); 
    M_RL.set_speed(rlRpm);
}

void handle_get_rpm(void) {
    float rpm;
    rpm = M_RL.get_speed();
    ser.sendCmdStart(GET_RPM);
    ser.sendCmdBinArg<float>(rpm);
    ser.sendCmdEnd();
}

void handle_get_pwm(void) {
    int16_t pwm = M_RL.get_pwm();
    ser.sendCmdStart(GET_PWM);
    ser.sendCmdBinArg<int16_t>(pwm);
    ser.sendCmdEnd();
}

void handle_get_pid(void) {
    float pidBuf[3];
    pidBuf[0] = M_RL.get_p();
    pidBuf[1] = M_RL.get_i();
    pidBuf[2] = M_RL.get_d();
    M_RL.get_pid(pidBuf);     
    ser.sendCmdStart(GET_PID);
    for(int i = 0; i < 3; i++) ser.sendCmdBinArg<float>(pidBuf[i]);
    ser.sendCmdEnd();
}

void handle_set_sample_time(void) {
    long sampleTime = ser.readBinArg<long>();
    long unit = ser.readBinArg<long>(); 
    if(unit) M_RL.set_sample_time_ms(sampleTime); 
    else M_RL.set_sample_time_us(sampleTime);
}
