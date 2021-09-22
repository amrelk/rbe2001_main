#include "RBE1001Lib.h"
#include "BlueMotor.h"

LeftMotor leftMotor;
RightMotor rightMotor;

BlueMotor blueMotor;
float deg, lastDeg;
long lastMillis;

void setup() {
  Serial.begin(9600);
  pinMode(BOOT_FLAG_PIN, INPUT_PULLUP);
  blueMotor.setEffort(150);
}

void loop() {
  lastDeg = deg;
  deg = blueMotor.getPosition();
  if (!digitalRead(BOOT_FLAG_PIN)) {
    Serial.print(millis());
    Serial.print(", ");
    Serial.print(blueMotor.getCount());
    Serial.print(", ");
    Serial.println(((deg-lastDeg)/(millis()-lastMillis))*166.7);
  }
  lastMillis = millis();
  delay(100);
}