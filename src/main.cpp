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

// (160/255)(effort) + 95 = deadband thing

void setup() {
  Serial.begin(9600);
  pinMode(BOOT_FLAG_PIN, INPUT_PULLUP);
  //odometry = new Odometry(&leftMotor, &rightMotor);
  //odometry->start();
  //blueMotor.startPid();
  blueMotor.Kp = 100;
}

void setEffortWithoutDB(int effort) {
  blueMotor.setEffort((int)(((151.0 / 255) * effort) - 104));
}

void loop() {
  for(int effort = -200; effort >= -255; effort -= 10) {
    lastDeg = deg;
    deg = blueMotor.getPosition();
    setEffortWithoutDB(effort);
    Serial.print(millis());
    Serial.print(" ");
    Serial.print(effort);
    Serial.print(" ");
    Serial.print(((151.0 / 255) * effort) - 104);
    Serial.print(" ");
    Serial.println(((deg - lastDeg) / (millis() - lastMillis)) * 166.7);
    lastMillis = millis();
    delay(500);
  }
}