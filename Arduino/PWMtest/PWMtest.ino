#include "TimerOne.h"

TimerOne pwm;

int pwmPin = 5;      // LED connected to digital pin 9

void setup() {
  pinMode(pwmPin, OUTPUT);  // sets the pin as output
  pwm.pwm(pwmPin, 10, 100);
}

void loop() {

  
}
