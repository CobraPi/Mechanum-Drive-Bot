#include <Arduino.h>
//#include <Controller.h>
#include <Wire.h>
#include <SPI.h>
#include <Motor.h>
#include <Encoder.h>
#include <PID.h>
#include <DCMotorServo.h>
//#include <PIDController.h>

#define P 0.5 
#define I 0.0//0.
#define D 0.0//0.3
#define DC_MOTOR_ENCODER_1_INCH 1344

long curTick, prevTick, timeout;
double kP, kI, kD, pOut, iOut, dOut, targetPos, error, previousError, pidOut, currentPos;
PID motPID(&error, &pidOut, &targetPos, P, I, D, P_ON_M, DIRECT); 
//PID motPID(P, I, D, -255, 255);
//PIDController motPID;
//Encoder encoder(6, 7);

DCMotorServo servo = DCMotorServo(4, 5, 6, 7);
//Motor m1(4, 5);
bool dir;
int sign = -1;
int position;

void process_serial();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  servo.myPID->SetTunings(P, I, D);
  servo.setPWMSkip(50);
  servo.setAccuracy(20);
  dir = true;
  timeout = millis();
}

void loop() {
  //wait 1s before starting
  static unsigned long motor_timeout = millis() + 1000;
  static bool motor_go = 0;
  if(dir) {
    position = DC_MOTOR_ENCODER_1_INCH;
  }
  else {
    position = 0;//-DC_MOTOR_ENCODER_1_INCH;
  }
  servo.moveTo(position);
  servo.run();
}

/*
void process_serial() {
  if(Serial.available()) {
      char c = Serial.read();
      float p, i, d; 
      int choice;
      switch(c) {
        
        case 'q':
          //d = motPID.GetKd(); 
          p = Serial.parseFloat();
          //i = motPID.GetKi();
          //motPID.SetTunings(p, i, d);
          motPID.setKp(p);  
          Serial.print("D: ");
          Serial.println(motPID.GetKp());
          break;

        case 'w':
          //d = motPID.GetKd(); 
          //p = motPID.GetKp();
          i = Serial.parseFloat();
          //motPID.SetTunings(p, i, d);
          motPID.setKi(i); 
          Serial.print("D: ");
          Serial.println(motPID.GetKi());
          break; 
        
        case 'e':
          d = Serial.parseFloat();
          //p = motPID.GetKp();
          //i = motPID.GetKi();
          //motPID.SetTunings(p, i, d);
          motPID.setKd(d); 
          Serial.print("D: ");
          Serial.println(motPID.GetKd());
          break; 
        
        case 'a':
          targetPos = Serial.parseInt();
          Serial.print("Target Position: ");
          Serial.println(targetPos);
          break;

        case 'x':
          Serial.print("Encoder: "); Serial.print(encoder.read());
          Serial.print(" - Current Position: "); Serial.print(currentPos);
          Serial.print(" - Target Position: "); Serial.print(targetPos);
          Serial.print(" - Error: "); Serial.print(error);
          Serial.print(" - PID Out: "); Serial.print(pidOut);
          Serial.println(""); 
          break;

        case 'c':
          encoder.readAndReset();
          targetPos = 0;
          break;

        default:
          Serial.print(" P: ");
          Serial.print(motPID.GetKp());
          Serial.print(" I: ");
          Serial.print(motPID.GetKi());
          Serial.print(" D: ");
          Serial.println(motPID.GetKd());
    }
  }
}
*/