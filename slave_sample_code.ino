// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>
#define n_of_pole 12

volatile int pwm_value1 = 1500, pwm_value2 = 1500;
volatile int rpm1 = 0, rpm2 = 0;
volatile int prev_time1 = 0, prev_time2 = 0;

void setup() {
  Wire.begin(0x66 | 0x67); // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  TWAMR = (0x66 | 0x67) << 1;
  //Serial.begin(115200);          // start serial for output
  attachInterrupt(digitalPinToInterrupt(2), rising1, RISING);
  attachInterrupt(digitalPinToInterrupt(3), rising2, RISING);
}

void loop() {

}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  /*while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  Serial.println(x); */        // print the integer
}

void requestEvent() {
  int adr = TWDR >> 1;
  if (adr == 0x66) {
    Wire.write(highByte(pwm_value1));
    Wire.write(lowByte(pwm_value1));
  }
  if (adr == 0x67) {
    Wire.write(highByte(pwm_value2));
    Wire.write(lowByte(pwm_value2));
  }
}

void rising1() {
  prev_time1 = micros();
  attachInterrupt(digitalPinToInterrupt(2), falling1, FALLING);
}

void falling1() {
  pwm_value1 = micros() - prev_time1;
  //rpm1 = (64*(1000000)/n_of_pole)/pwm_value1;
  //Serial.print("RPM1: ");
  //Serial.println(rpm1);
  attachInterrupt(digitalPinToInterrupt(2), rising1, RISING);
}
void rising2() {
  prev_time2 = micros();
  attachInterrupt(digitalPinToInterrupt(3), falling2, FALLING);
}

void falling2() {
  pwm_value2 = micros() - prev_time2;
  //rpm2 = (64*(1000000)/n_of_pole)/pwm_value2;
  //Serial.print("RPM2: ");
  //Serial.println(rpm2);
  attachInterrupt(digitalPinToInterrupt(3), rising2, RISING);
}
