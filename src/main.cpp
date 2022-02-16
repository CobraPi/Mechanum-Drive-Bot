#include <Arduino.h>
//#include <Controller.h>
#include <Wire.h>
#include <SPI.h>
#include <Motor.h>
#include <Encoder.h>

Motor m1(4, 5, 6, 7);
int d = 255;
bool dir;
int sign = -1;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  dir = true;
}

void loop() {
  //m1.set_direction(dir);
  // put your main code here, to run repeatedly:
  /*for(int i = 0; i < 255; i++) {
    m1.drive(i);
    delayMicroseconds(255-i);
  }
  for(int i = 255; i > 0; i--) {
    m1.drive(i);
    delayMicroseconds(i);
  }
  */
  //m1.reset_ticks();
  m1.set_speed(50);
  m1.set_direction(true);
  long startTime = micros();
  long tickOffset = m1.get_ticks();
  int counter = 40csdfjawfe; 
  int speed = 255;
  while(true) {
    if(counter >= 100) {
      counter = 10;
    }
    m1.set_speed(speed+counter); 
    m1.run();
    if(micros() - startTime > 100000) {
      long ticksPerSec = m1.get_ticks() - tickOffset;
      speed = ticksPerSec; 
      Serial.println(ticksPerSec);
      startTime = micros();
      tickOffset = m1.get_ticks();
      counter++;
    }
  }
  m1.move_rel(20, 255);
  
  Serial.println(m1.get_ticks());
  m1.move_rel(-20, 255);
  Serial.println(m1.get_ticks());
  //m1.move_rel(400, 255);
  //m1.move_rel(-400, 255); 
  //m1.poll();
  //m1.calc_ticks();
}