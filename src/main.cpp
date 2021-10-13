#include "RBE1001Lib.h"
#include "BlueMotor.h"
#include "util/Odometry.h"
#include "util/RollAvg.h"
#include "RemoteConstants.h"
#include <IRdecoder.h>

typedef enum {
  FOLLOW_LINE,
 // TURNING,
  RAISE_ARM,
  GRAB_PANEL,
 // LOWER_ARM,
 // ROOF_RESTART,
 // PLACE_PANEL,
 // SWITCH_SIDES,
 // TRADE_PANEL,
 // STAGING_RESTARt
} State;

const uint8_t IR_DETECTOR_PIN = 15;

LeftMotor leftMotor;
RightMotor rightMotor;
ESP32AnalogRead leftSensor;
ESP32AnalogRead rightSensor;
Rangefinder ultra;
Servo gripper;
IRDecoder decoder(IR_DETECTOR_PIN);


BlueMotor blueMotor;
float deg, lastDeg;
long lastMillis;
RollAvg<float, 40> speedAvg;

const float targetVoltage = 4.5;

//---- ARM ENCODER VAULES
const int placeDeg45 = 1200;
const int grabDeg45 = 1050;
const int placeDeg25 = 1550;
const int grabDeg25 = 1550;

//---- ULTRASONIC VALUES
const float ultraGrab45 = 5.12;
const float ultraPlace45 = 3.32;
const float ultraGrab25 = 3.00;
const float ultraPlace25 = 3.00;

const int servoPin = 33;
const int servoOpen = 90;
const int servoClosed = 0;



//Odometry* odometry;

// (160/255)(effort) + 95 = deadband thing

void setup() {
  Serial.begin(9600);
  pinMode(BOOT_FLAG_PIN, INPUT_PULLUP);
  //odometry = new Odometry(&leftMotor, &rightMotor);
  //odometry->start();
  blueMotor.startPid();
  blueMotor.Kp = 4;
  blueMotor.Ki = 0.000001;
  blueMotor.Kd = 10;
  leftSensor.attach(36);
  rightSensor.attach(39);
  ultra.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
  gripper.attach(servoPin);
}

void setBlueEffortWithoutDB(int effort) {
  blueMotor.setEffort((int)(((151.0 / 255) * effort) - 104));
}

State state;

void StateMachine(State state){
  switch (state) {
  case FOLLOW_LINE:
    leftMotor.setEffort((targetVoltage - leftSensor.readVoltage()) * 0.05);
    rightMotor.setEffort((targetVoltage - rightSensor.readVoltage()) * 0.05);
    if(ultra.getDistanceCM() <= ultraGrab45) {
      state = RAISE_ARM;
    }
    break;
  case RAISE_ARM:
    blueMotor.setTarget(placeDeg45);
    leftMotor.setEffort((targetVoltage - leftSensor.readVoltage()) * 0.05);
    rightMotor.setEffort((targetVoltage - rightSensor.readVoltage()) * 0.05);
    gripper.write(servoOpen);
    if(ultra.getDistanceCM() < grabDeg45 && blueMotor.getPosition() == placeDeg45) {
      state = GRAB_PANEL;
    }
  case GRAB_PANEL:
    gripper.write(servoClosed);
    blueMotor.setTarget(placeDeg45 + 100);
    int16_t keypress = decoder.getKeyCode();

  

  
}
}

void loop() {
  gripper.write(servoOpen);
}