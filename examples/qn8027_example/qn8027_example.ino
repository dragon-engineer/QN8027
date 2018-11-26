
#include <Arduino.h>
#include <QN8027.h>
#include <Wire.h>

QN8027 fm;
float frequency = 89.5f;

void setup() {
  fm.begin(frequency);
}

void loop() {
  // put your main code here, to run repeatedly:
}
