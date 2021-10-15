#define _GLIBCXX_USE_C99 1

#include "RBE1001Lib.h"
#include "BlueMotor.h"
#include "util/Odometry.h"
#include "util/RollAvg.h"
#include "RemoteConstants.h"
#include "LineSensor.h"
#include "Drive.h"
#include <IRdecoder.h>
#include <string>

typedef enum {
  CALIBRATING,
  FOLLOWING_LINE,
  CROSSING,
  TURNING_LEFT,
  TURNING_RIGHT,
  TURNING_180,
  APPROACHING_45,
  APPROACHING_25,
  APPROACHING_PLATFORM,
  FINAL_PLATFORM,
  GRABBING_PANEL,
  RELEASING_PANEL,
  LIFTING,
  LEAVING,
  PAUSED,
  STOP
} State;

const uint8_t IR_DETECTOR_PIN = 15;

LeftMotor leftMotor;
RightMotor rightMotor;
int linePins[] = {36, 39};
LineSensor lineSens = LineSensor(linePins);
Rangefinder ultra;
Drive drive = Drive(&leftMotor, &rightMotor, &ultra, &lineSens);
Servo gripper;
IRDecoder decoder(IR_DETECTOR_PIN);


BlueMotor blueMotor;
float deg, lastDeg;
long lastMillis, calLastCount = 100;
RollAvg<float, 40> speedAvg;

State state = CALIBRATING;
State nextState = APPROACHING_45, resumeState = FOLLOWING_LINE;

const float targetVoltage = 4.5;

//---- ARM ENCODER VAULES
const int placeDeg45 = 1100;
const int grabDeg45 = 1050;
const int placeDeg25 = 1550;
const int grabDeg25 = 1550;

//---- ULTRASONIC VALUES
const float ultraGrab45 = 4;
const float ultraPlace45 = 4;
const float ultraGrab25 = 3.00;
const float ultraPlace25 = 3.00;

const int servoPin = 33;
const int servoOpen = 160;
const int servoClosed = 75;

int interval = 100;



//Odometry* odometry;

// (160/255)(effort) + 95 = deadband thing

void setup() {
  Serial.begin(9600);
  pinMode(BOOT_FLAG_PIN, INPUT_PULLUP);
  pinMode(MOTOR_DISABLE, OUTPUT);
  digitalWrite(MOTOR_DISABLE, LOW);
  //odometry = new Odometry(&leftMotor, &rightMotor);
  //odometry->start();
  //blueMotor.startPid();
  blueMotor.Kp = 4;
  blueMotor.Ki = 0.000001;
  blueMotor.Kd = 10;
  ultra.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
  gripper.attach(servoPin);
  decoder.init();
}

void stateMachine(){
  int code = decoder.getKeyCode();
  if (code!=-1)
    Serial.println(code);
  if (code == 6) {
    resumeState = state;
    state = PAUSED;
    Serial.println(code);
  }
  switch (state) {
  case CALIBRATING:
    gripper.write(70);
    blueMotor.setEffortWithoutDB(-20);
    if ((blueMotor.getPosition() - calLastCount)>0?(blueMotor.getPosition() - calLastCount):-(blueMotor.getPosition() - calLastCount) < 1) {
      blueMotor.reset();
      blueMotor.startPid();
      blueMotor.setTarget(100);
      state = APPROACHING_45;
      interval = 20;
    } else {
      calLastCount = blueMotor.getPosition();
    }
    break;
  case FOLLOWING_LINE:
    drive.lineFollow();
    if(lineSens.isAtCross()) {
      state = CROSSING;
      nextState = APPROACHING_45;
    }
    break;
  case CROSSING:
    drive.manualForwards(25);
    if (drive.isIdle()) {
      state = TURNING_RIGHT;
    }
    break;
  case TURNING_RIGHT:
    drive.turn(-95);
    blueMotor.setTarget(500);
    if (drive.isIdle()) {
      state = nextState;
    }
    break;
  case TURNING_LEFT:
    drive.turn(90);
    blueMotor.setTarget(500);
    if (drive.isIdle()) {
      state = nextState;
    }
    break;
  case TURNING_180:
    drive.turn(180);
    blueMotor.setTarget(500);
    if (drive.isIdle()) {
      state = FOLLOWING_LINE;
    }
    break;
  case APPROACHING_45:
    gripper.write(servoClosed);
    blueMotor.setTarget(placeDeg45);
    drive.approach(ultraPlace45);
    if (ultra.getDistanceCM() < 4) {
      state = GRABBING_PANEL;
    }
    break;
  case GRABBING_PANEL:
    gripper.write(servoOpen);
    //blueMotor.setTarget(placeDeg45-25);
    //drive.manualForwards(5);
    if (blueMotor.getErr() < 20) {
      while (decoder.getKeyCode() != 17) {delay(10);}
      state = LEAVING;
    }
    break;
  case LEAVING:
    drive.approach(32);
    if (ultra.getDistanceCM() > 30) {
      state = TURNING_RIGHT;
      nextState = APPROACHING_PLATFORM;
    }
    break;
  case APPROACHING_PLATFORM:
    drive.approach(3.75);
    blueMotor.setTarget(200);
    if (ultra.getDistanceCM() < 4) {
      state = FINAL_PLATFORM;
    }
    break;
  case FINAL_PLATFORM:
    blueMotor.setTarget(100);
    drive.manualForwards(12);
    gripper.write(110);
    while (decoder.getKeyCode() != 16) {}
    state = TURNING_180;
    break;
  case PAUSED:
    if (code == 4) {
      state = resumeState;
      Serial.println("resumed");
    }
    break;
  case STOP:
    break;
  }
}

void loop() {
  /*if(Serial.available() > 0) {
    std::string in = Serial.readString().c_str();
    switch(in.at(0)) {
      case 'm':
        blueMotor.setTarget(std::stoi(in.substr(1)));
        Serial.print("motor: ");
        Serial.println(std::stoi(in.substr(1)));
        break;
      case 's':
        gripper.write(std::stof(in.substr(1)));
        Serial.print("servo: ");
        Serial.println(std::stof(in.substr(1)));
        break;
      case 'a':
        drive.approach(std::stof(in.substr(1)));
        Serial.print("approach: ");
        Serial.println(std::stof(in.substr(1)));
        break;
      case 'l':
        drive.lineFollow();
        break;
      case 't':
        drive.turn(std::stof(in.substr(1)));
        Serial.print("turn: ");
        Serial.println(std::stof(in.substr(1)));
    }
    Serial.read();
  }
  delay(25);/**/
  //Serial.println(decoder.getKeyCode());
  stateMachine();

  delay(interval);
}