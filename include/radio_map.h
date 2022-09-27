#include <Arduino.h>

// Radio packet that transmits data related to kinematic control of robot
struct ControlPacket {
  int16_t  FR_Pwm; // PWM value range [-100,100]
  int16_t  FL_Pwm;
  int16_t  RR_Pwm;
  int16_t  RL_Pwm;
};

struct SensorPacket {
  uint16_t Front_Distance_Ultrasonic; // Reading from ultrasonic sensor
  uint16_t Front_Distance_TOF;
  uint16_t Heading;        // Magnatometer reading from IMU
  int8_t   Pitch;            // Pitch of robot in degrees
  int8_t   Roll;            // Roll of robot in degrees
};

enum RadioMode {RX, TX};