#include "RBE1001Lib.h"
#include "BlueMotor.h"
//#include "util/Odometry.h"
#include "util/RollAvg.h"

LeftMotor leftMotor;
RightMotor rightMotor;

BlueMotor blueMotor;
float deg, lastDeg;
long lastMillis;
RollAvg<float, 40> speedAvg;

//Odometry* odometry;

void setup() {
  Serial.begin(9600);
  pinMode(BOOT_FLAG_PIN, INPUT_PULLUP);
  //odometry = new Odometry(&leftMotor, &rightMotor);
  //odometry->start();
  blueMotor.startPid();
  blueMotor.Kp = 100;
}

void loop() {
  /* lastDeg = deg;
  deg = blueMotor.getPosition();
  if (!digitalRead(BOOT_FLAG_PIN)) {
    blueMotor.setEffort(255);
    Serial.print(millis());
    Serial.print(" ");
    Serial.print(blueMotor.getCount());
    Serial.print(" ");
    Serial.println(((deg-lastDeg)/(millis()-lastMillis))*166.7);
  } else blueMotor.setEffort(191);
  lastMillis = millis();
  delay(100); */
  if (blueMotor.getCount() > 2000) {
    blueMotor.setTarget(-10);
  } else if (blueMotor.getCount() < 0) {
    blueMotor.setTarget(2010);
  } else {
    delay(20);
    lastDeg = deg;
    deg = blueMotor.getPosition();
    speedAvg.addRead(((deg-lastDeg>0?deg-lastDeg:lastDeg-deg)/(millis()-lastMillis))*166.7);
    Serial.println(speedAvg.getAvg());
    lastMillis = millis();
  }
}

void step() {

}