#pragma once
#include "RBE1001Lib.h"
#include "LineSensor.h"

typedef enum {
  OFF,
  MANUAL,
  LINE,
  APPROACH,
  TURN
} DriveState;

class Drive
{
private:
  static void driveTask(void* drive);

  void lineFollowerLoop(float speed);
  void ultrasonicSensorPid();

  LeftMotor* left;
  RightMotor* right;
  Rangefinder* ranger;
  LineSensor* lineSensor;
  DriveState state = OFF;
  float rangeSP = 5;
public:
  Drive(LeftMotor* left, RightMotor* right, Rangefinder* ranger, LineSensor* lineSensor);
  ~Drive();

  void lineFollow();
  void approach(float distance);
  void turn(float degrees);
  void manualForwards(float cm);
  bool isIdle();

  float followerSpeed = 100;
  float followerKp = 30;
};
